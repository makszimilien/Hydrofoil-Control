#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <QuickPID.h>
#include <TickTwo.h>
#include <WebServer.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/rmt.h>
#include <vector>
#include <algorithm>
#include <freertos/semphr.h>
#include "filehandling.h"
#include "sensor_wait.h"
#include "controller_validation.h"
#include "calibration_session.h"
#include "peer_protocol.h"
#include "servo_test_protocol.h"
#include "remote_setup_protocol.h"
#include "bench_recording.h"
#include "controller_assets.h"

constexpr int servoPin=1, capacitancePin=6, ledPin=8, resetPin=5;
extern WebServer benchServer;
extern bool benchFilesystem,benchAutomatic,benchSingle;
extern String benchHostname;
extern bench_recording::Recorder benchRecording;
void setupBenchRoutes();
void serviceBenchSamples();
void stopBenchSamples(bench_recording::FinishReason reason=bench_recording::FinishReason::Stopped);

bool first=false,slave=false,settingsReady=false,benchActive=false,otaInProgress=false;
bool servoCenterTest=false;
bool simulateSensorTimeout=false,sensorHealthy=false,positionValid=false;
uint32_t sensorTimeouts=0,sensorGoodSamples=0,sensorLastTicks=0;
std::vector<int> rawValues;
hw_timer_t *timer=nullptr;
int median=0,position=0,servoPos=1500;
float setpoint=1500,input=1500,output=0;
int control=0,pwmRead=0;
uint32_t receiverLastValid=0;
bool receiverValid=false;
constexpr uint32_t kReceiverTimeoutMs=250;
Servo elevator;
dataStruct controlParams{};
allBoards boardsParams{};
CalibrationSession calibrationSession;
String calibrationNotice;
bool calibrationSampling(){return settingsReady&&controlParams.calibration==1&&calibrationSession.active;}
String macAddresses[2];
String boardSelector="master";
const char *jsonConfigsPath="/configs.json",*jsonWifiPath="/wifi.json",*jsonAddressesPath="/addresses.json";
QuickPID elevatorPid(&input,&output,&setpoint,0,0,0,
 QuickPID::pMode::pOnError,QuickPID::dMode::dOnError,
 QuickPID::iAwMode::iAwClamp,QuickPID::Action::direct);
bool servoTimeOut=false;
uint32_t servoTimeOutStart=0;
constexpr uint32_t servoTimeOutInterval=2000;
RingbufHandle_t rb=nullptr;
uint32_t receiverClockHz=2000000;
struct ReceivedSetup {uint8_t sender[6];SetupPacket packet;};
QueueHandle_t receivedSetup=nullptr;
struct SetupTransfer {SetupPacket request{},snapshot{};bool pending=false,available=false;unsigned attempts=0;uint32_t sentAt=0,receivedAt=0;String calibrationStatus="";};
SetupTransfer setupTransfers[2];
uint32_t setupSequence=0,setupSession=0;
SetupPacket lastCalibrationRequest{};uint8_t lastCalibrationSender[6]={};bool haveLastCalibration=false;uint32_t lastCalibrationResult=0;
struct IdentityPacket {uint32_t magic=0x48464935,kind=1,nonce=0;char name[33]={};};
struct ReceivedIdentity {uint8_t sender[6];IdentityPacket packet;};
QueueHandle_t receivedIdentities=nullptr;
String peerNames[2];uint32_t identityNonce[2]={},identitySentAt[2]={};unsigned identityAttempts[2]={};bool identityPending[2]={};
struct ReceivedTest { uint8_t sender[6]; ServoTestPacket packet; };
QueueHandle_t receivedTests=nullptr;
struct TestTransfer { ServoTestPacket packet{}; bool pending=false; unsigned attempts=0; uint32_t sentAt=0; String status="Not requested"; };
TestTransfer testTransfers[2];
ServoTestPacket lastTest{};uint8_t lastTestSender[6]={};bool haveLastTest=false;
uint32_t testSequence=0;
struct ReceivedPeer { uint8_t sender[6]; PeerPacket packet; };
QueueHandle_t receivedConfigs=nullptr;
struct Transfer { PeerPacket packet{}; bool pending=false; unsigned attempts=0; uint32_t sentAt=0; String status="Not sent"; };
Transfer transfers[2];
uint32_t transferSequence=0;
PeerPacket lastApplied{}; uint8_t lastSender[6]={}; bool haveLastApplied=false;
bool espNowReady=false;
uint32_t rejectedPackets=0,receivedPackets=0;
int lastPeerSendResult=0;
bool hideHotspot=false,hotspotVisible=true,restartPending=false;
uint32_t hideHotspotAt=0,restartAt=0;
String controllerError;
SemaphoreHandle_t controlMutex=nullptr;
TaskHandle_t controlTaskHandle=nullptr;
class ControlLock {
 public: ControlLock(){if(controlMutex)xSemaphoreTakeRecursive(controlMutex,portMAX_DELAY);}
 ~ControlLock(){if(controlMutex)xSemaphoreGiveRecursive(controlMutex);}
};


void enterBenchMode() {
 ControlLock lock;
 if(servoCenterTest){servoCenterTest=false;elevator.detach();}
 if(!benchActive) {
  benchActive=true;
  elevatorPid.SetMode(QuickPID::Control::manual);
  elevator.detach();
  stopBenchSamples();
  servoTimeOut=false;
 }
}
bool holdServoTest(uint32_t pulse) {
 ControlLock lock;
 if(pulse<1000||pulse>2000||otaInProgress||first||!settingsReady||controlParams.calibration)return false;
 enterBenchMode();stopBenchSamples();
 elevatorPid.SetMode(QuickPID::Control::manual);
 elevator.attach(servoPin,1000,2000);
 if(!elevator.attached()){controllerError="Servo output could not start";return false;}
 // Deliberately bypass endpoint mapping, reversal, receiver input and all trim.
 servoPos=pulse;elevator.writeMicroseconds(pulse);
 servoCenterTest=true;controllerError="";return true;
}
uint32_t directedTestPulse(uint32_t requested){return controlParams.servoMax<controlParams.servoMin?3000-requested:requested;}
bool centerServoTest(){return holdServoTest(1500);}
void writeServoCommand(int pulse) {
 if(benchActive||otaInProgress||!settingsReady||first||!elevator.attached())return;
 const int lo=std::min(controlParams.servoMin,controlParams.servoMax);
 const int hi=std::max(controlParams.servoMin,controlParams.servoMax);
 servoPos=std::max(lo,std::min(hi,pulse));
 elevator.writeMicroseconds(servoPos);
}
bool resumeController() {
 ControlLock lock;
 if(!settingsReady||!timer||first||otaInProgress||!controlTaskHandle)return false;
 if(controlParams.calibration||!usableCalibrationRange(controlParams.minMeasured,controlParams.maxMeasured)){controllerError="Finish sensor calibration before resuming control";return false;}
 stopBenchSamples();
 servoCenterTest=false;
 benchActive=false;
 // Discard receiver pulses buffered while control was paused; only new pulses
 // may re-establish a live receiver command.
 if(rb){size_t bytes=0;void *item;while((item=xRingbufferReceive(rb,&bytes,0)))vRingbufferReturnItem(rb,item);}
 receiverValid=false;control=0;
 output=0;input=1500;setpoint=controlParams.setpoint;
 elevatorPid.SetMode(QuickPID::Control::manual);
 elevatorPid.SetSampleTimeUs(10000);
 elevatorPid.SetTunings(controlParams.p,controlParams.i,controlParams.d);
 elevatorPid.SetOutputLimits(-500,500);
 elevator.attach(servoPin,1000,2000);
 if(!elevator.attached()){benchActive=true;controllerError="Servo output could not start";return false;}
 writeServoCommand((controlParams.servoMin+controlParams.servoMax)/2);
 elevatorPid.SetMode(QuickPID::Control::timer);
 controllerError="";return true;
}
void addControllerDiagnostics(JsonObject d) {
 ControlLock lock;
 d["sensor_healthy"]=sensorHealthy;d["sensor_timeouts"]=sensorTimeouts;
 d["sensor_good_samples"]=sensorGoodSamples;d["last_raw_ticks"]=sensorLastTicks;
 d["servo_attached"]=elevator.attached();
 d["controller_mode"]=servoCenterTest?"center-test":benchActive?"bench":"controller";
 d["settings_valid"]=settingsReady;
 d["first_setup"]=first;d["slave"]=slave;
 d["station_mac"]=WiFi.macAddress();d["ssid"]=benchHostname;
 d["position_valid"]=positionValid;d["median20_ticks"]=median;
 d["servo_us"]=servoPos;d["receiver_valid"]=receiverValid;
 d["receiver_pulse_us"]=pwmRead;d["receiver_offset"]=control;
 d["controller_error"]=controllerError;d["esp_now_ready"]=espNowReady;
 d["esp_now_received"]=receivedPackets;d["esp_now_rejected"]=rejectedPackets;
 d["last_peer_send_result"]=lastPeerSendResult;
 d["normal_sample_target_hz"]=500;d["pid_interval_ms"]=10;
 d["ota_in_progress"]=otaInProgress;
 d["calibration_active"]=calibrationSession.active;
 d["calibration_enabled"]=controlParams.calibration==1;
 d["calibration_samples"]=calibrationSession.samples;
 d["calibration_range_ready"]=calibrationSession.usable();
 d["calibration_saved_valid"]=usableCalibrationRange(controlParams.minMeasured,controlParams.maxMeasured);
 d["calibration_notice"]=calibrationNotice;
}
int getMedian(const std::vector<int> &values) {
  std::vector<int> temp = values;
  std::sort(temp.begin(), temp.end());
  return temp[temp.size() / 2];
}
void startMeasurement() {
  if (timer == nullptr) {
    sensorHealthy = false;
    positionValid = false;
    rawValues.clear();
    return;
  }
  pinMode(capacitancePin, OUTPUT);
  digitalWrite(capacitancePin, LOW);
  delayMicroseconds(50);
  noInterrupts();
  pinMode(capacitancePin, INPUT);
  timerRestart(timer);
  const auto result = waitForSensorHigh(
      []() { return !simulateSensorTimeout && digitalRead(capacitancePin) == HIGH; },
      []() { return timerRead(timer); });
  interrupts();

  sensorLastTicks = static_cast<uint32_t>(result.ticks);
  if (result.timedOut) {
    sensorTimeouts++;
    // Fault counts are reported by diagnostics; never block control on UART.
    sensorHealthy = false;
    positionValid = false;
    rawValues.clear(); // Do not feed stale samples into the PID.
    return;
  }
  sensorHealthy = true;
  sensorGoodSamples++;
  rawValues.push_back(static_cast<int>(result.ticks));
  if (rawValues.size() > 20) {
    rawValues.erase(rawValues.begin());
  }
  if(calibrationSession.active&&rawValues.size()==20&&!simulateSensorTimeout)
    {median=getMedian(rawValues);calibrationSession.observe(median);}
}
void calculatePosition() {
  positionValid = false;
  if (!sensorHealthy || rawValues.size() < 20) {
    return;
  }

  median = getMedian(rawValues);
  const int minimum=calibrationSession.active?calibrationSession.minimum:controlParams.minMeasured;
  const int maximum=calibrationSession.active?calibrationSession.maximum:controlParams.maxMeasured;
  if (!usableCalibrationRange(minimum,maximum) || (calibrationSession.active&&!calibrationSession.usable()))
    return;
  float progress = static_cast<float>(median - minimum) / (maximum - minimum);

  // Scale the progress value between 1000 and 2000us
  position = 1000 + 1000 * progress;
  positionValid = true;
}
void calculatePid() {
  static bool pausedForSensor = false;
  calculatePosition();
  if (!positionValid) {
    elevatorPid.SetMode(QuickPID::Control::manual);
    pausedForSensor = true;
    return; // Existing PWM remains at its last command; do not invent a safe angle.
  }

  input = position;
  if (pausedForSensor) {
    elevatorPid.SetMode(QuickPID::Control::timer);
    pausedForSensor = false;
  }
  setpoint = controlParams.setpoint + control;
  if (setpoint < 1000)
    setpoint = 1000;
  else if (setpoint > 2000)
    setpoint = 2000;
  elevatorPid.Compute();
  servoPos =
      map(output, -500, 500, controlParams.servoMin, controlParams.servoMax);
  writeServoCommand(servoPos);
}

void getPulseDuration() {
 if(rb) {
  size_t bytes=0; rmt_item32_t *items=nullptr;
  while((items=(rmt_item32_t*)xRingbufferReceive(rb,&bytes,0))) {
   for(size_t n=0;n<bytes/sizeof(rmt_item32_t);++n) {
    const auto &p=items[n];
    uint32_t ticks=p.level0?p.duration0:(p.level1?p.duration1:0);
    int pulse=receiverClockHz?uint64_t(ticks)*1000000/receiverClockHz:0;
    if(pulse>=985&&pulse<=2015){pwmRead=pulse;receiverLastValid=millis();receiverValid=true;}
   }
   vRingbufferReturnItem(rb,items);
  }
 }
 if(receiverValid&&uint32_t(millis()-receiverLastValid)>kReceiverTimeoutMs)receiverValid=false;
 control=receiverValid?(pwmRead-1500)*controlParams.factor/100:0;
}
void setupRMTReceiver() {
 rmt_config_t c={};c.rmt_mode=RMT_MODE_RX;c.channel=RMT_CHANNEL_3;
 c.gpio_num=GPIO_NUM_3;c.clk_div=40;c.mem_block_num=1;
 c.rx_config.filter_en=true;c.rx_config.filter_ticks_thresh=100;
 c.rx_config.idle_threshold=25000;
 if(rmt_config(&c)!=ESP_OK||rmt_driver_install(c.channel,2048,0)!=ESP_OK)return;
 if(rmt_get_ringbuf_handle(c.channel,&rb)!=ESP_OK){rb=nullptr;return;}
 rmt_get_counter_clock(c.channel,&receiverClockHz);
 rmt_rx_start(c.channel,true);
}
TickTwo measurementTicker([](){startMeasurement();},2,0,MILLIS);
// Normal control always follows the sensor; receiver influence offsets its target.
TickTwo pidTicker([](){calculatePid();},10,0,MILLIS);
TickTwo rmtTicker([](){getPulseDuration();},20,0,MILLIS);

void onDataRecv(const uint8_t *sender,const uint8_t *data,int len) {
 SetupPacket setup;
 if(sender&&decodeSetupPacket(data,len,setup)){
  ReceivedSetup rx{};std::memcpy(rx.sender,sender,6);rx.packet=setup;
  if(!receivedSetup||xQueueSend(receivedSetup,&rx,0)!=pdTRUE)++rejectedPackets;return;
 }
 if(sender&&data&&len==sizeof(IdentityPacket)&&receivedIdentities){
  ReceivedIdentity rx{};std::memcpy(&rx.packet,data,sizeof(rx.packet));
  if(rx.packet.magic==0x48464935&&(rx.packet.kind==1||rx.packet.kind==2)&&rx.packet.name[32]==0){std::memcpy(rx.sender,sender,6);xQueueSend(receivedIdentities,&rx,0);return;}
 }
 ServoTestPacket test;
 if(sender&&decodeServoTestPacket(data,len,test)){
  ReceivedTest rx{};std::memcpy(rx.sender,sender,6);rx.packet=test;
  if(!receivedTests||xQueueSend(receivedTests,&rx,0)!=pdTRUE)++rejectedPackets;
  return;
 }
 ReceivedPeer received{};
 if(!sender||!decodePeerPacket(data,len,received.packet)||!receivedConfigs){++rejectedPackets;return;}
 std::memcpy(received.sender,sender,6);
 if(xQueueSend(receivedConfigs,&received,0)!=pdTRUE)++rejectedPackets;
}
void addPeer(const String &address) {
 uint8_t mac[6];if(!espNowReady||!parseMacAddress(address.c_str(),mac))return;
 if(esp_now_is_peer_exist(mac))return;
 esp_now_peer_info_t peer={};std::copy(mac,mac+6,peer.peer_addr);
 peer.channel=1;peer.ifidx=WIFI_IF_STA;peer.encrypt=false;
 lastPeerSendResult=esp_now_add_peer(&peer);
}
void transmitIdentity(unsigned i){uint8_t mac[6];if(!parseMacAddress(macAddresses[i].c_str(),mac))return;
 IdentityPacket p;p.nonce=identityNonce[i];addPeer(macAddresses[i]);esp_now_send(mac,reinterpret_cast<uint8_t*>(&p),sizeof(p));identitySentAt[i]=millis();++identityAttempts[i];
}
void requestIdentities(){if(slave||!espNowReady)return;for(unsigned i=0;i<2;++i){uint8_t mac[6];
 if(parseMacAddress(macAddresses[i].c_str(),mac)&&!identityPending[i]&&(!identityAttempts[i]||uint32_t(millis()-identitySentAt[i])>=30000)){
  identityNonce[i]=esp_random();identityAttempts[i]=0;identityPending[i]=true;transmitIdentity(i);
 }
}}
void serviceIdentities(){ReceivedIdentity rx;
 while(receivedIdentities&&xQueueReceive(receivedIdentities,&rx,0)==pdTRUE){
  auto &p=rx.packet;
  if(slave&&p.kind==1){p.kind=2;snprintf(p.name,sizeof(p.name),"%s",benchHostname.c_str());
   char address[18];snprintf(address,sizeof(address),"%02X:%02X:%02X:%02X:%02X:%02X",rx.sender[0],rx.sender[1],rx.sender[2],rx.sender[3],rx.sender[4],rx.sender[5]);addPeer(String(address));esp_now_send(rx.sender,reinterpret_cast<uint8_t*>(&p),sizeof(p));
  }else if(!slave&&p.kind==2){for(unsigned i=0;i<2;++i){uint8_t mac[6];if(identityPending[i]&&p.nonce==identityNonce[i]&&parseMacAddress(macAddresses[i].c_str(),mac)&&!std::memcmp(mac,rx.sender,6)){
    peerNames[i]=String(p.name);identityPending[i]=false;
  }}}
 }
 for(unsigned i=0;i<2;++i)if(identityPending[i]&&uint32_t(millis()-identitySentAt[i])>=1000){if(identityAttempts[i]>=3){identityPending[i]=false;peerNames[i]="";}else transmitIdentity(i);}
}
void transmitPeer(unsigned index) {
 auto &t=transfers[index];uint8_t mac[6];
 if(!parseMacAddress(macAddresses[index].c_str(),mac))return;
 addPeer(macAddresses[index]);
 lastPeerSendResult=esp_now_send(mac,reinterpret_cast<uint8_t*>(&t.packet),sizeof(t.packet));
 ++t.attempts;t.sentAt=millis();
}
void sendPeer(unsigned index,unsigned kind=1) {
 setupTransfers[index].available=false;if(setupTransfers[index].request.action==0)setupTransfers[index].pending=false;
 auto &t=transfers[index];
 t.packet={};t.packet.magic=kPeerMagic;t.packet.version=1;t.packet.kind=kind;
 static const uint32_t bootSession=esp_random();
 t.packet.session=bootSession;t.packet.sequence=++transferSequence;
 t.packet.params=index?boardsParams.slave2:boardsParams.slave1;
 // Calibration is always local, never a side effect of applying tuning.
 t.packet.params.calibration=0;t.packet.params.minMeasured=0;t.packet.params.maxMeasured=0;
 t.pending=true;t.attempts=0;t.status="Waiting for board confirmation";transmitPeer(index);
}
void servicePeerRetries() {
 for(unsigned i=0;i<2;++i){auto &t=transfers[i];if(!t.pending||uint32_t(millis()-t.sentAt)<1000)continue;
  if(t.attempts>=3){t.pending=false;t.status="Unconfirmed: board may be offline or running older firmware. Check board before retrying.";}
  else transmitPeer(i);
 }
}
bool performServoTest(uint32_t action) {
 if(action==1)return centerServoTest();
 if(action>=1000&&action<=2000)return holdServoTest(directedTestPulse(action));
 if(action==2){enterBenchMode();return true;}
 if(action==3)return resumeController();
 return false;
}
String testActionName(uint32_t action,uint32_t actual){if(action>=1000&&action<=2000)return String("requested ")+String(action)+" us; actual pulse "+String(actual)+" us";return action==1?"holding 1500 us":action==2?"paused, output off":"normal control started";}
void transmitServoTest(unsigned i){
 auto &t=testTransfers[i];uint8_t mac[6];if(!parseMacAddress(macAddresses[i].c_str(),mac))return;
 addPeer(macAddresses[i]);esp_now_send(mac,reinterpret_cast<uint8_t*>(&t.packet),sizeof(t.packet));
 ++t.attempts;t.sentAt=millis();
}
void sendServoTest(unsigned i,uint32_t action){
 auto &t=testTransfers[i];t.packet=ServoTestPacket{};static const uint32_t session=esp_random();
 t.packet.session=session;t.packet.sequence=++testSequence;t.packet.action=action;
 t.pending=true;t.attempts=0;t.status="Waiting for confirmation";transmitServoTest(i);
}
void serviceServoTests(){
 ReceivedTest rx;
 while(receivedTests&&xQueueReceive(receivedTests,&rx,0)==pdTRUE){
  auto &p=rx.packet;
  if(!slave){
   for(unsigned i=0;i<2;++i){auto &t=testTransfers[i];uint8_t mac[6];
    if(t.pending&&parseMacAddress(macAddresses[i].c_str(),mac)&&!std::memcmp(mac,rx.sender,6)&&servoTestAckMatches(p,t.packet)){
     t.pending=false;t.status=p.result?"Rejected: check board locally":String("Confirmed: ")+testActionName(p.action,p.actualPulse);
    }
   }
   continue;
  }
  if(p.kind!=1)continue;
  ControlLock lock;
  bool duplicate=haveLastTest&&!std::memcmp(lastTestSender,rx.sender,6)&&sameServoTest(p,lastTest);
  bool stale=haveLastTest&&!std::memcmp(lastTestSender,rx.sender,6)&&p.session==lastTest.session&&int32_t(p.sequence-lastTest.sequence)<=0;
  uint32_t result=1;
  if(duplicate){
   // Never replay motion after a later local Pause, Start, calibration or OTA.
   result=((p.action==1||p.action>=1000)?servoCenterTest&&elevator.attached()&&servoPos==int(p.action==1?1500:directedTestPulse(p.action)):p.action==2?benchActive&&!servoCenterTest&&!elevator.attached():!benchActive&&elevator.attached())?0:1;
  }else if(!stale){
   lastTest=p;std::memcpy(lastTestSender,rx.sender,6);haveLastTest=true;
   result=performServoTest(p.action)?0:1;
  }
  ServoTestPacket ack=p;ack.kind=2;ack.result=result;ack.actualPulse=(!result&&(p.action==1||p.action>=1000))?servoPos:0;
  char address[18];snprintf(address,sizeof(address),"%02X:%02X:%02X:%02X:%02X:%02X",rx.sender[0],rx.sender[1],rx.sender[2],rx.sender[3],rx.sender[4],rx.sender[5]);
  addPeer(String(address));esp_now_send(rx.sender,reinterpret_cast<uint8_t*>(&ack),sizeof(ack));
 }
 for(unsigned i=0;i<2;++i){auto &t=testTransfers[i];if(!t.pending||uint32_t(millis()-t.sentAt)<1000)continue;
  if(t.attempts>=3){t.pending=false;t.status="UNCONFIRMED — check board; R5.4.1 required";}else transmitServoTest(i);
 }
}
void requestServoTest(){
 String action=benchServer.arg("action");String scope=benchServer.arg("scope");
 uint32_t command=action=="center"?1:action=="pause"?2:action=="resume"?3:0;
 if(action=="pulse"){
  String value=benchServer.arg("pulse");
  if(value.length()!=4){benchServer.send(400,"text/plain","Enter an integer pulse from 1000 to 2000 us");return;}
  for(unsigned i=0;i<value.length();++i)if(value[i]<'0'||value[i]>'9'){benchServer.send(400,"text/plain","Invalid pulse");return;}
  command=value.toInt();if(command<1000||command>2000){benchServer.send(400,"text/plain","Pulse must be 1000 to 2000 us");return;}
 }
 int peer=scope=="slave-1"?0:scope=="slave-2"?1:-1;
 if(!command||(scope!="local"&&scope!="all"&&peer<0)||(slave&&scope!="local")||(action=="pulse"&&scope=="all")){benchServer.send(400,"text/plain","Invalid test target or command");return;}
 if(peer>=0){uint8_t mac[6];if(!parseMacAddress(macAddresses[peer].c_str(),mac)){benchServer.send(400,"text/plain","No board configured for this target");return;}}
 if(otaInProgress){benchServer.send(409,"text/plain","Update in progress");return;}
 for(unsigned i=0;i<2;++i)if(testTransfers[i].pending&&command!=2&&(scope=="all"||peer==int(i))){benchServer.send(409,"text/plain","Wait for pending test confirmation");return;}
 if(peer>=0){sendServoTest(peer,command);benchServer.send(200,"text/plain","Sent to selected slave; check confirmation");return;}
 if(!performServoTest(command)){benchServer.send(409,"text/plain","Local command refused; check setup/calibration");return;}
 if(scope=="all")for(unsigned i=0;i<2;++i){uint8_t mac[6];if(parseMacAddress(macAddresses[i].c_str(),mac))sendServoTest(i,command);}
 benchServer.send(200,"text/plain","Command applied locally; check each peer confirmation");
}

void setupEspNow() {
 receivedSetup=xQueueCreate(6,sizeof(ReceivedSetup));setupSession=esp_random();
 receivedIdentities=xQueueCreate(4,sizeof(ReceivedIdentity));
 receivedTests=xQueueCreate(4,sizeof(ReceivedTest));
 receivedConfigs=xQueueCreate(4,sizeof(ReceivedPeer));
 espNowReady=esp_now_init()==ESP_OK;
 if(!espNowReady)return;
 esp_now_register_recv_cb(onDataRecv);
 if(!slave){for(unsigned i=0;i<2;i++)addPeer(macAddresses[i]);}
}
void applyLocalParams(const dataStruct &next) {
 ControlLock lock;
 bool minChanged=next.servoMin!=controlParams.servoMin;
 bool maxChanged=next.servoMax!=controlParams.servoMax;
 if(minChanged||maxChanged)enterBenchMode();
 dataStruct merged=next;merged.enable=1; // Retained field for legacy settings/packet compatibility.
 if(next.calibration&&!controlParams.calibration){
  calibrationSession.begin();rawValues.clear();calibrationNotice="Collecting range: raise and lower sensor in water, then turn calibration OFF";
 }else if(!next.calibration&&controlParams.calibration){
  calibrationNotice=calibrationSession.usable()?"Calibration saved":
   usableCalibrationRange(next.minMeasured,next.maxMeasured)?"No usable new range; previous calibration retained":"No usable range; calibration OFF, calibrate before resuming";
  calibrationSession.finish();
 }
 controlParams=merged;
 if(!next.calibration&&!usableCalibrationRange(next.minMeasured,next.maxMeasured))enterBenchMode();
 elevatorPid.SetTunings(next.p,next.i,next.d);
 if(!benchActive&&(minChanged||maxChanged)){
  writeServoCommand(minChanged?next.servoMin:next.servoMax);
  servoTimeOut=true;servoTimeOutStart=millis();
 }
}
void serviceReceivedConfigs() {
 ReceivedPeer rx;
 while(receivedConfigs&&xQueueReceive(receivedConfigs,&rx,0)==pdTRUE){
  auto &packet=rx.packet;
  if(!slave){
   bool matched=false;
   for(unsigned i=0;i<2;++i){uint8_t mac[6];auto &t=transfers[i];
    if(parseMacAddress(macAddresses[i].c_str(),mac)&&!std::memcmp(mac,rx.sender,6)&&
       t.pending&&ackMatches(packet,t.packet)){
     t.pending=false;t.status=packet.result==0?"Confirmed: board saved settings":"Board rejected settings; check its calibration or storage";matched=true;
    }
   }
   if(!matched)++rejectedPackets;continue;
  }
  if(packet.kind!=1&&packet.kind!=3&&packet.kind!=5){++rejectedPackets;continue;}
  ControlLock transaction;
  uint8_t result=0;
  bool duplicate=haveLastApplied&&!std::memcmp(lastSender,rx.sender,6)&&packet.kind==lastApplied.kind&&sameRequest(packet,lastApplied);
  bool stale=haveLastApplied&&!std::memcmp(lastSender,rx.sender,6)&&packet.session==lastApplied.session&&int32_t(packet.sequence-lastApplied.sequence)<=0;
  if(!duplicate&&stale)result=1;
  if(!duplicate&&!stale){
   dataStruct next=packet.params;
   if(next.enable==0)next.enable=1; // Retire receiver-only mode before saving.
   if(packet.kind==3){next.servoMin=controlParams.servoMin;next.servoMax=controlParams.servoMax;}
   if(packet.kind==5){const int lo=std::min(next.servoMin,next.servoMax),hi=std::max(next.servoMin,next.servoMax);bool reverse=controlParams.servoMin>controlParams.servoMax;next.servoMin=reverse?hi:lo;next.servoMax=reverse?lo:hi;}
   next.calibration=controlParams.calibration;next.minMeasured=controlParams.minMeasured;next.maxMeasured=controlParams.maxMeasured;
   // Keep calibration sessions isolated from remote tuning.
   if(next.calibration||!validControlParams(next))result=1;
   else {allBoards proposed=boardsParams;proposed.slave1=next;
    if(!writeStructJson(SPIFFS,jsonConfigsPath,proposed))result=2;
    else {boardsParams=proposed;applyLocalParams(next);lastApplied=packet;std::memcpy(lastSender,rx.sender,6);haveLastApplied=true;++receivedPackets;}
   }
  }
  PeerPacket ack=packet;ack.kind=packet.kind==5?6:packet.kind==3?4:2;ack.result=result;
  char address[18];snprintf(address,sizeof(address),"%02X:%02X:%02X:%02X:%02X:%02X",rx.sender[0],rx.sender[1],rx.sender[2],rx.sender[3],rx.sender[4],rx.sender[5]);
  addPeer(String(address));esp_now_send(rx.sender,reinterpret_cast<uint8_t*>(&ack),sizeof(ack));
 }
}
void sendJson(JsonDocument &d) {
 if(d.overflowed()){benchServer.send(500,"text/plain","Response too large");return;}
 String body;serializeJson(d,body);benchServer.sendHeader("Cache-Control","no-store");
 benchServer.send(200,"application/json",body);
}
void redirect(const char *path) {benchServer.sendHeader("Location",path);benchServer.send(303);}
int selectedIndex() {
 if(slave)return 1;
 if(boardSelector=="master")return 0;
 return boardSelector=="slave-1"?1:2;
}
dataStruct selectedParams() {int n=selectedIndex();
 if(!slave&&benchServer.hasArg("board")){String x=benchServer.arg("board");n=x=="slave-1"?1:x=="slave-2"?2:0;}return n==0?boardsParams.master:n==1?boardsParams.slave1:boardsParams.slave2;}
bool parseNumber(const char *name,float &v) {
 if(!benchServer.hasArg(name))return false;
 String s=benchServer.arg(name);char *end=nullptr;
 if(!s.length())return false;v=strtof(s.c_str(),&end);
 return end!=s.c_str()&&*end=='\0'&&std::isfinite(v);
}
bool parseInteger(const char *name,int &v) {float x;if(!parseNumber(name,x)||x!=floorf(x)||x<-100000||x>100000)return false;v=int(x);return true;}
void setSliders() {
 ControlLock transaction;
 if(!settingsReady){benchServer.send(503,"text/plain","Settings unavailable");return;}
 String selector=benchServer.arg("board-selector");
 if(selector!="master"&&selector!="slave-1"&&selector!="slave-2"){benchServer.send(400,"text/plain","Unknown board");return;}
 int index=slave?1:selector=="master"?0:selector=="slave-1"?1:2;
 dataStruct next=index==0?boardsParams.master:index==1?boardsParams.slave1:boardsParams.slave2;
 bool ok=parseNumber("slider-p",next.p)&&parseNumber("slider-i",next.i)&&parseNumber("slider-d",next.d)&&
 parseInteger("slider-setpoint",next.setpoint)&&parseInteger("slider-factor",next.factor)&&
 parseInteger("slider-enable",next.enable)&&parseInteger("slider-calibration",next.calibration)&&
 parseInteger("slider-servo-min",next.servoMin)&&parseInteger("slider-servo-max",next.servoMax);
 if(!ok){benchServer.send(400,"text/plain","Invalid slider values");return;}
 if(next.enable==0)next.enable=1;
 bool local=slave||index==0;
 if(local){next.minMeasured=controlParams.minMeasured;next.maxMeasured=controlParams.maxMeasured;}
 if(!local&&(!espNowReady||macAddresses[index-1].length()==0)){benchServer.send(409,"text/plain","Add the board MAC and check radio setup first");return;}
 if(!local&&(transfers[index-1].pending||(setupTransfers[index-1].pending&&setupTransfers[index-1].request.action!=0))){benchServer.send(409,"text/plain","Wait for the previous transfer to finish");return;}
 if(local){next.calibration=controlParams.calibration;}else{next.calibration=0;}
 if(!validControlParams(next)){benchServer.send(400,"text/plain","Invalid slider values or incomplete calibration");return;}
 allBoards proposed=boardsParams;
 if(index==0)proposed.master=next;else if(index==1)proposed.slave1=next;else proposed.slave2=next;
 if(!writeStructJson(SPIFFS,jsonConfigsPath,proposed)){benchServer.send(500,"text/plain","Could not save settings");return;}
 boardsParams=proposed;boardSelector=selector;
 if(local)applyLocalParams(next);else sendPeer(index-1);
 benchServer.send(200,"text/plain","OK");
}
void copySharedTravel(dataStruct &to,const dataStruct &from){
 const int lo=std::min(from.servoMin,from.servoMax),hi=std::max(from.servoMin,from.servoMax);const bool reverse=to.servoMin>to.servoMax;to.servoMin=reverse?hi:lo;to.servoMax=reverse?lo:hi;
}
void copySharedTuning(dataStruct &to,const dataStruct &from){
 to.p=from.p;to.i=from.i;to.d=from.d;to.setpoint=from.setpoint;to.factor=from.factor;to.enable=1;
}
void setSharedTuning(){
 ControlLock lock;
 if(slave||!settingsReady||otaInProgress||controlParams.calibration){benchServer.send(409,"text/plain","Use an available master outside calibration");return;}
 bool paired=false;
 for(unsigned i=0;i<2;++i){uint8_t mac[6];if(parseMacAddress(macAddresses[i].c_str(),mac)){paired=true;if(transfers[i].pending||(setupTransfers[i].pending&&setupTransfers[i].request.action!=0)){benchServer.send(409,"text/plain","A settings or calibration transfer is still pending");return;}}}
 if(!paired||!espNowReady){benchServer.send(409,"text/plain","No paired board or radio unavailable");return;}
 dataStruct next=controlParams;
 const bool travel=benchServer.arg("include-travel")=="1";
 bool ok=parseNumber("slider-p",next.p)&&parseNumber("slider-i",next.i)&&parseNumber("slider-d",next.d)&&parseInteger("slider-setpoint",next.setpoint)&&parseInteger("slider-factor",next.factor)&&parseInteger("slider-enable",next.enable);
 if(travel)ok=ok&&parseInteger("slider-servo-min",next.servoMin)&&parseInteger("slider-servo-max",next.servoMax);
 if(!ok||!validControlParams(next)|| (travel&&next.servoMin==next.servoMax)){benchServer.send(400,"text/plain","Invalid tuning or travel values");return;}
 allBoards proposed=boardsParams;copySharedTuning(proposed.master,next);if(travel)copySharedTravel(proposed.master,next);
 for(unsigned i=0;i<2;++i){uint8_t mac[6];if(parseMacAddress(macAddresses[i].c_str(),mac)){auto &target=i?proposed.slave2:proposed.slave1;copySharedTuning(target,next);if(travel)copySharedTravel(target,next);}}
 if(!writeStructJson(SPIFFS,jsonConfigsPath,proposed)){benchServer.send(500,"text/plain","Could not save master tuning");return;}
 boardsParams=proposed;applyLocalParams(proposed.master);
 for(unsigned i=0;i<2;++i){uint8_t mac[6];if(parseMacAddress(macAddresses[i].c_str(),mac))sendPeer(i,travel?5:3);}
 benchServer.send(200,"text/plain","Master saved; waiting for each slave");
}
// Explicit recovery for incomplete legacy configs. Never called during boot.
void restoreControllerDefaults() {
 ControlLock lock;
 if(!benchActive||otaInProgress){benchServer.send(409,"text/plain","Pause the controller before restoring defaults");return;}
 if(!benchFilesystem){benchServer.send(503,"text/plain","Filesystem unavailable");return;}
 if(benchServer.arg("confirm")!="reset-controller-settings"){
  benchServer.send(400,"text/plain","Explicit controller-default confirmation required");return;
 }
 uint8_t expected[6],actual[6];
 if(!parseMacAddress(benchServer.arg("expected-mac").c_str(),expected)||
    !parseMacAddress(WiFi.macAddress().c_str(),actual)||std::memcmp(expected,actual,6)!=0){
  benchServer.send(409,"text/plain","Station MAC does not match the requested board");return;
 }
 String firstFlag=readFileJson(SPIFFS,jsonWifiPath,"first"),slaveFlag=readFileJson(SPIFFS,jsonWifiPath,"slave");
 auto validFlag=[](const String &v){return v=="true"||v=="false"||v=="True"||v=="False"||v=="0"||v=="1";};
 auto truth=[](const String &v){return v=="true"||v=="True"||v=="1";};
 if(!validFlag(firstFlag)||!validFlag(slaveFlag)){
  benchServer.send(503,"text/plain","Saved device role flags are unavailable");return;
 }
 // Same eleven factory values per board as data/configs.json, including all
 // six fields lost by the original undersized JSON allocation on slave2.
 const dataStruct defaults={1.0f,0.0f,0.0f,1500,1,1000,2000,40,0,2000,24000};
 allBoards proposed={defaults,defaults,defaults};
 stopBenchSamples();
 elevatorPid.SetMode(QuickPID::Control::manual);
 elevator.detach();
 servoTimeOut=false;
 if(!writeStructJson(SPIFFS,jsonConfigsPath,proposed)){
  controllerError="Controller defaults could not be saved";
  benchServer.send(500,"text/plain",controllerError);return;
 }
 allBoards checked{};
 if(!readStructJson(SPIFFS,jsonConfigsPath,checked)||
    !validControlParams(checked.master)||!validControlParams(checked.slave1)||!validControlParams(checked.slave2)||
    std::memcmp(&checked,&proposed,sizeof(checked))!=0){
  controllerError="Controller defaults readback failed; controller remains paused";
  benchServer.send(500,"text/plain",controllerError);return;
 }
 boardsParams=checked;
 calibrationSession.finish();calibrationNotice="";
 first=truth(firstFlag);slave=truth(slaveFlag);
 controlParams=slave?boardsParams.slave1:boardsParams.master;
 if(controlParams.calibration==1)calibrationNotice="Calibration interrupted by restart. Turn OFF, then ON to start a new calibration";
 settingsReady=true;
 rawValues.clear();sensorHealthy=false;positionValid=false;median=0;position=0;
 receiverValid=false;control=0;pwmRead=0;
 output=0;input=1500;setpoint=controlParams.setpoint;
 elevatorPid.Reset();
 elevatorPid.SetSampleTimeUs(10000);
 elevatorPid.SetTunings(controlParams.p,controlParams.i,controlParams.d);
 elevatorPid.SetOutputLimits(-500,500);
 controllerError="";
 benchServer.send(200,"text/plain","Controller defaults saved and verified. Controller remains paused; resume separately.");
}
uint32_t changeCalibration(bool enabled) {
 ControlLock transaction;
 if(!settingsReady||otaInProgress)return 1;
 dataStruct next=controlParams;next.calibration=enabled;
 if(next.calibration==controlParams.calibration)return 0;
 enterBenchMode();
 const bool usable=calibrationSession.usable();
 if(!next.calibration&&usable){next.minMeasured=calibrationSession.minimum;next.maxMeasured=calibrationSession.maximum;}
 allBoards proposed=boardsParams;if(slave)proposed.slave1=next;else proposed.master=next;
 if(!writeStructJson(SPIFFS,jsonConfigsPath,proposed))return 2;
 boardsParams=proposed;applyLocalParams(next);
 if(!enabled&&!usable)return usableCalibrationRange(next.minMeasured,next.maxMeasured)?4:5;
 return 0;
}
void setCalibration() {
 String value=benchServer.arg("enabled");if(value!="0"&&value!="1"){benchServer.send(400,"text/plain","Choose calibration on or off");return;}
 const uint32_t result=changeCalibration(value=="1");
 benchServer.send(result==1?409:result==2?500:200,"text/plain",result==1?"Board is not ready":result==2?"Calibration could not be saved":"OK");
}
#include "remote_setup_impl.h"

void configureRoutes() {
 benchServer.on("/board-state",HTTP_GET,sendBoardStates);
 benchServer.on("/remote-calibration",HTTP_POST,requestRemoteCalibration);
 benchServer.on("/board-identities",HTTP_GET,[](){requestIdentities();StaticJsonDocument<768>d;for(unsigned i=0;i<2;++i){auto o=d.createNestedObject(i?"slave-2":"slave-1");o["name"]=peerNames[i];o["station_mac"]=macAddresses[i];o["pending"]=identityPending[i];}sendJson(d);});
 benchServer.on("/shared-tuning",HTTP_POST,setSharedTuning);
 benchServer.on("/servo-test",HTTP_POST,requestServoTest);
 benchServer.on("/servo-test-status",HTTP_GET,[](){StaticJsonDocument<1024>d;
  d["local"]=servoCenterTest?String("Holding ")+String(servoPos)+" us":String(benchActive?"Paused, output off":"Normal control");
  for(unsigned i=0;i<2;++i){auto o=d.createNestedObject(i?"slave-2":"slave-1");o["address"]=macAddresses[i];o["pending"]=testTransfers[i].pending;o["status"]=testTransfers[i].status;}sendJson(d);
 });
 benchServer.on("/calibration",HTTP_POST,setCalibration);
 benchServer.on("/peer-status",HTTP_GET,[](){StaticJsonDocument<1024>d;for(unsigned i=0;i<2;++i){auto o=d.createNestedObject(i?"slave-2":"slave-1");o["pending"]=transfers[i].pending;o["status"]=transfers[i].status;o["sequence"]=transfers[i].packet.sequence;}sendJson(d);});
 benchServer.on("/",HTTP_GET,[](){benchServer.sendHeader("Cache-Control","no-store");benchServer.send_P(200,"text/html",kControllerIndex);});
 benchServer.on("/script.js",HTTP_GET,[](){benchServer.sendHeader("Cache-Control","no-store");benchServer.send_P(200,"application/javascript",kControllerScript);});
 benchServer.on("/style.css",HTTP_GET,[](){benchServer.sendHeader("Cache-Control","no-store");benchServer.send_P(200,"text/css",kControllerStyle);});
 benchServer.on("/setup",HTTP_GET,[](){benchServer.send_P(200,"text/html",kControllerSetup);});
 benchServer.on("/get-mac",HTTP_GET,[](){benchServer.send(200,"text/plain",WiFi.macAddress());});
 benchServer.on("/device-status",HTTP_GET,[](){StaticJsonDocument<512>d;d["ap_mac"]=WiFi.softAPmacAddress();d["ssid"]=benchHostname;d["slave"]=slave;d["first"]=first;d["controller_mode"]=servoCenterTest?"center-test":benchActive?"bench":"controller";d["settings_valid"]=settingsReady;d["servo_us"]=servoPos;d["error"]=controllerError;sendJson(d);});
 benchServer.on("/setup",HTTP_POST,[](){
  if(!benchFilesystem){benchServer.send(503,"text/plain","Filesystem unavailable");return;}
  String role=benchServer.arg("role");if(role!="master"&&role!="slave"){benchServer.send(400,"text/plain","Invalid role");return;}
  enterBenchMode();
  if(!writeFileJson(SPIFFS,jsonWifiPath,"slave",role=="slave"?"True":"False")||!writeFileJson(SPIFFS,jsonWifiPath,"first","False")){
   benchServer.send(500,"text/plain","Could not save role");return;}
  benchServer.send(200,"text/html","<meta name='viewport' content='width=device-width'><p>Role saved. Reconnect to the same HF-Bench network after restart, then open <a href='/'>Controller</a>.</p>");
  restartPending=true;restartAt=millis()+1000;
 });
 benchServer.on("/controller/resume",HTTP_POST,[](){if(!resumeController()){benchServer.send(409,"text/plain","Controller cannot start; check diagnostics and device setup");return;}redirect("/");});
 benchServer.on("/controller/pause",HTTP_POST,[](){enterBenchMode();redirect("/bench");});
 benchServer.on("/controller/defaults",HTTP_POST,restoreControllerDefaults);
 benchServer.on("/get-settings",HTTP_GET,[](){
  auto p=selectedParams();StaticJsonDocument<1024>d;
  d["slider-p"]=p.p;d["slider-i"]=p.i;d["slider-d"]=p.d;d["slider-setpoint"]=p.setpoint;
  d["slider-factor"]=p.factor;d["slider-enable"]=p.enable;d["slider-calibration"]=p.calibration;
  d["slider-servo-min"]=p.servoMin;d["slider-servo-max"]=p.servoMax;d["board-selector"]=boardSelector;sendJson(d);
 });
 benchServer.on("/get-values",HTTP_GET,[](){StaticJsonDocument<1024>d;{ControlLock lock;
  d["PID-input"]=input;d["PID-output"]=output;d["PID-setpoint"]=setpoint;d["PWM-input"]=pwmRead;d["Servo-position"]=servoPos;
  d["Min-measured-value"]=calibrationSession.active?calibrationSession.minimum:controlParams.minMeasured;d["Max-measured-value"]=calibrationSession.active?calibrationSession.maximum:controlParams.maxMeasured;d["Actual-measured-value"]=median;}sendJson(d);
 });
 benchServer.on("/get-addresses",HTTP_GET,[](){StaticJsonDocument<512>d;d["broadcastAddress0"]=macAddresses[0];d["broadcastAddress1"]=macAddresses[1];sendJson(d);});
 benchServer.on("/select-board",HTTP_POST,[](){String v=benchServer.arg("board-selector");if(v!="master"&&v!="slave-1"&&v!="slave-2"){benchServer.send(400,"text/plain","Unknown board");return;}boardSelector=v;benchServer.send(200,"text/plain","OK");});
 benchServer.on("/set-sliders",HTTP_POST,setSliders);
 benchServer.on("/add-mac",HTTP_POST,[](){
  if(slave){benchServer.send(409,"text/plain","Add peers on the master");return;}
  uint8_t bytes[6];String address=benchServer.arg("mac");address.replace('-',':');address.toUpperCase();
  if(!parseMacAddress(address.c_str(),bytes)){benchServer.send(400,"text/plain","Invalid MAC address");return;}
  if(address==macAddresses[0]||address==macAddresses[1]){benchServer.send(200,"text/plain","Already added");return;}
  int n=macAddresses[0].isEmpty()?0:macAddresses[1].isEmpty()?1:-1;
  if(n<0){benchServer.send(409,"text/plain","Two peers already configured");return;}
  String proposed[2]={macAddresses[0],macAddresses[1]};proposed[n]=address;
  if(!writeArrayJson(SPIFFS,jsonAddressesPath,"addresses",proposed,2)){benchServer.send(500,"text/plain","Could not save peer");return;}
  macAddresses[n]=address;setupTransfers[n]=SetupTransfer{};peerNames[n]="";identityPending[n]=false;identityAttempts[n]=0;transfers[n]=Transfer{};addPeer(address);benchServer.send(200,"text/plain","OK");
 });
 benchServer.on("/remove-slaves",HTTP_POST,[](){
  if(!benchServer.hasArg("remove-slaves")){benchServer.send(400,"text/plain","Missing action");return;}
  if(transfers[0].pending||transfers[1].pending){benchServer.send(409,"text/plain","Wait for pending transfers before removing peers");return;}
  String empty[2];if(!writeArrayJson(SPIFFS,jsonAddressesPath,"addresses",empty,2)){benchServer.send(500,"text/plain","Could not save peers");return;}
  transfers[0]=Transfer{};transfers[1]=Transfer{};for(unsigned i=0;i<2;++i){setupTransfers[i]=SetupTransfer{};peerNames[i]="";identityPending[i]=false;identityAttempts[i]=0;}
  for(auto &address:macAddresses){uint8_t mac[6];if(parseMacAddress(address.c_str(),mac))esp_now_del_peer(mac);address="";}benchServer.send(200,"text/plain","OK");
 });
 benchServer.on("/wifi-off",HTTP_POST,[](){if(!benchServer.hasArg("wifi-off")){benchServer.send(400,"text/plain","Missing action");return;}
  benchServer.send(200,"text/plain","Hotspot will hide until restart; ESP-NOW stays on");hideHotspot=true;hideHotspotAt=millis()+1000;
 });
}
void runControllerTask(void*) {
 for(;;) {
  {
   ControlLock lock;
   if(!benchActive&&!otaInProgress&&settingsReady&&!first) {
    rmtTicker.update();measurementTicker.update();
    if(servoTimeOut&&uint32_t(millis()-servoTimeOutStart)>=servoTimeOutInterval)servoTimeOut=false;
    if(!servoTimeOut)pidTicker.update();
    digitalWrite(ledPin,positionValid&&abs(int(input)-int(setpoint))<30?LOW:HIGH);
   }
  }
  vTaskDelay(1);
 }
}
void useWaterHeightControl(allBoards &params) {
 params.master.enable=1;params.slave1.enable=1;params.slave2.enable=1;
}
void setup() {
 Serial.begin(115200);pinMode(capacitancePin,INPUT);pinMode(ledPin,OUTPUT);pinMode(resetPin,INPUT_PULLUP);
 controlMutex=xSemaphoreCreateRecursiveMutex();
 rawValues.reserve(20);
 benchFilesystem=initFS();
 bool configs=benchFilesystem&&readStructJson(SPIFFS,jsonConfigsPath,boardsParams);
 String firstFlag=readFileJson(SPIFFS,jsonWifiPath,"first"),slaveFlag=readFileJson(SPIFFS,jsonWifiPath,"slave");
 auto validFlag=[](const String &v){return v=="true"||v=="false"||v=="True"||v=="False"||v=="0"||v=="1";};
 auto truth=[](const String &v){return v=="true"||v=="True"||v=="1";};
 first=truth(firstFlag);slave=truth(slaveFlag);
 settingsReady=configs&&validControlParams(boardsParams.master)&&validControlParams(boardsParams.slave1)&&validControlParams(boardsParams.slave2)&&validFlag(firstFlag)&&validFlag(slaveFlag);
 // Preserve saved tuning/calibration; retire the legacy receiver-only flag in memory.
 // The next explicit save persists it. Boot performs no settings write.
 if(settingsReady)useWaterHeightControl(boardsParams);
 if(!readArrayJson(SPIFFS,jsonAddressesPath,"addresses",macAddresses,2))controllerError="Saved peer addresses unavailable";
 controlParams=slave?boardsParams.slave1:boardsParams.master;
 if(controlParams.calibration==1)calibrationNotice="Calibration interrupted by restart. Turn OFF, then ON to start a new calibration";
 if(!settingsReady)controllerError="Saved settings unavailable or invalid; controller paused";
 WiFi.mode(WIFI_AP_STA);WiFi.setSleep(false);
 String suffix=WiFi.softAPmacAddress();suffix.replace(":","");
 benchHostname="HF-Bench-"+suffix;
 WiFi.softAP(benchHostname.c_str(),nullptr,1);
 timer=timerBegin(0,2,true);
 setupEspNow();setupRMTReceiver();
 benchActive=true;
 configureRoutes();setupBenchRoutes();benchServer.begin();
 measurementTicker.start();pidTicker.start();rmtTicker.start();
 if(controlMutex&&xTaskCreate(runControllerTask,"foil-control",4096,nullptr,2,&controlTaskHandle)==pdPASS) {
  // Use the same validated startup path as the explicit Start button.
  if(!resumeController()&&controllerError.length()==0)controllerError="Automatic start unavailable; check device setup";
 }else controllerError="Control scheduler unavailable; bench access retained";
 // No automatic push on boot: applying remote settings is explicit.
 // No startup settings writes or factory reset. Existing router credentials stay stored.
}
void loop() {
 ArduinoOTA.handle();benchServer.handleClient();
 if(restartPending&&int32_t(millis()-restartAt)>=0)ESP.restart();
 if(hideHotspot&&int32_t(millis()-hideHotspotAt)>=0){WiFi.softAPdisconnect(true);hideHotspot=false;hotspotVisible=false;}
 // A brief jumper-to-ground gesture restores the hotspot without changing saved settings.
 if(!hotspotVisible&&digitalRead(resetPin)==LOW){WiFi.softAP(benchHostname.c_str(),nullptr,1);hotspotVisible=true;}
 if(!otaInProgress){
  serviceReceivedConfigs();servicePeerRetries();serviceServoTests();serviceIdentities();serviceRemoteSetup();
  if(benchActive){ControlLock lock;serviceBenchSamples();}

 }
 delay(1);
}
