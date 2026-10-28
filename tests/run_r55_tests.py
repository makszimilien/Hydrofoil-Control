from pathlib import Path
import ast,re,tempfile,subprocess
R=Path(__file__).resolve().parents[1];P=R;source=(P/'src/main.cpp').read_text()+'\n'+(P/'include/remote_setup_impl.h').read_text()
tree=ast.parse((R/'tests/run_controller_r5_tests.py').read_text())
def fixture(n):
 for x in tree.body:
  if isinstance(x,ast.Assign) and any(isinstance(a,ast.Name) and a.id==n for a in x.targets):return ast.literal_eval(x.value)
def fn(n):
 m=re.search(r'^[\w:*& <>]+\b'+n+r'\([^;]*?\)\s*\{',source,re.M);a=source.index('{',m.start());b=a+1;depth=1
 while depth:depth+=(source[b]=='{')-(source[b]=='}');b+=1
 return source[m.start():b]
def run(name,code):
 with tempfile.TemporaryDirectory() as t:
  p=Path(t);(p/'Arduino.h').write_text(fixture('filesystem_arduino_stub'));(p/'SPIFFS.h').write_text(fixture('filesystem_fs_stub'));(p/'t.cpp').write_text(code)
  subprocess.run(['clang++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-I',str(p),'-I',str(P/'include'),'-I',str(P/'src'),str(p/'t.cpp'),'-o',str(p/'t')],check=True)
  subprocess.run([str(p/'t')],check=True)
 print('PASS',name)
stub=fixture('settings_stubs').replace('void sendPeer(unsigned index){++sends;lastSent=index;}','')
base=stub+'''
#include "peer_protocol.h"
#include "calibration_session.h"
#include "remote_setup_protocol.h"
struct SetupTransfer {SetupPacket request{};bool available=false,pending=false;};SetupTransfer setupTransfers[2];
#include <vector>
#include <deque>
CalibrationSession calibrationSession;
struct ReceivedPeer {uint8_t sender[6];PeerPacket packet;};
struct Transfer {PeerPacket packet{};bool pending=false;unsigned attempts=0;uint32_t sentAt=0;String status="Not sent";};
Transfer transfers[2];
String macAddresses[2]={"DC:06:75:32:AB:B4",""};
uint32_t transferSequence=0,now=0;bool espNowReady=true,otaInProgress=false;
uint32_t millis(){return now;}uint32_t esp_random(){return 42;}
int lastPeerSendResult=0;unsigned receivedPackets=0,rejectedPackets=0;
PeerPacket lastApplied{};uint8_t lastSender[6]={};bool haveLastApplied=false;
int receivedConfigs=1;std::deque<ReceivedPeer> queue;std::vector<PeerPacket> wire;
const int pdTRUE=1;
int xQueueReceive(int,ReceivedPeer*p,int){if(queue.empty())return 0;*p=queue.front();queue.pop_front();return 1;}
void addPeer(const String&){}
int esp_now_send(const uint8_t*,const uint8_t*p,size_t){PeerPacket copy;std::memcpy(&copy,p,sizeof(copy));wire.push_back(copy);return 0;}
void enterBenchMode(){}
'''
code=base+'\n'.join(fn(n) for n in ['transmitPeer','sendPeer','servicePeerRetries','serviceReceivedConfigs','parseNumber','parseInteger','setSliders','changeCalibration','setCalibration'])+r'''
int main(){
 // No transmission until explicit action; remote settings ignore calibration.
 assert(wire.empty());goodRequest();benchServer.args["board-selector"]="slave-1";benchServer.args["slider-calibration"]="1";benchServer.args["slider-enable"]="0";setSliders();
 assert(benchServer.code==200&&wire.size()==1&&transfers[0].pending);assert(!wire[0].params.calibration&&!wire[0].params.minMeasured);
 assert(wire[0].params.enable==1);auto request=wire[0];setSliders();assert(benchServer.code==409&&wire.size()==1);
 PeerPacket decoded;assert(decodePeerPacket((uint8_t*)&request,sizeof(request),decoded));
 assert(!decodePeerPacket((uint8_t*)&request.params,sizeof(dataStruct),decoded));
 auto bad=request;bad.version=99;assert(!decodePeerPacket((uint8_t*)&bad,sizeof(bad),decoded));
 bad=request;bad.params.p=NAN;assert(!decodePeerPacket((uint8_t*)&bad,sizeof(bad),decoded));
 // Slave persists once, duplicate request only re-acknowledges.
 ReceivedPeer rx{};parseMacAddress("CC:BA:97:47:BE:28",rx.sender);rx.packet=request;slave=true;
 auto count=saves;queue.push_back(rx);serviceReceivedConfigs();assert(saves==count+1&&wire.back().result==0&&wire.back().kind==2);
 assert(controlParams.minMeasured==2000&&controlParams.maxMeasured==24000);
 queue.push_back(rx);serviceReceivedConfigs();assert(saves==count+1);
 // Storage failure and active calibration reject instead of claiming success.
 rx.packet.sequence++;saveSucceeds=false;queue.push_back(rx);serviceReceivedConfigs();assert(wire.back().result==2);saveSucceeds=true;
 controlParams.calibration=1;queue.push_back(rx);serviceReceivedConfigs();assert(wire.back().result==1);controlParams.calibration=0;
 // A delayed older retry must not overwrite a newer successful apply.
 rx.packet.sequence=request.sequence+2;queue.push_back(rx);serviceReceivedConfigs();assert(wire.back().result==0);auto saved=saves;
 rx.packet=request;queue.push_back(rx);serviceReceivedConfigs();assert(wire.back().result==1&&saves==saved);
 // Wrong sender and wrong transaction cannot confirm; correct receipt can.
 slave=false;rx.packet=request;rx.packet.kind=2;queue.push_back(rx);serviceReceivedConfigs();assert(transfers[0].pending);
 parseMacAddress("DC:06:75:32:AB:B4",rx.sender);rx.packet.sequence++;queue.push_back(rx);serviceReceivedConfigs();assert(transfers[0].pending);
 rx.packet.sequence=request.sequence;queue.push_back(rx);serviceReceivedConfigs();assert(!transfers[0].pending);
 // Exactly three attempts, then unconfirmed. No further background push.
 wire.clear();sendPeer(0);now=1000;servicePeerRetries();now=2000;servicePeerRetries();assert(wire.size()==3);now=3000;servicePeerRetries();assert(!transfers[0].pending);now=9999;servicePeerRetries();assert(wire.size()==3);
 // A slider submission never changes the local calibration flag or range.
 slave=true;controlParams.calibration=1;goodRequest();setSliders();assert(benchServer.code==200&&controlParams.calibration==1&&controlParams.minMeasured==2000);
 // Dedicated calibration stop commits locally, never sends a packet.
 calibrationSession.begin();calibrationSession.observe(1000);calibrationSession.observe(3000);benchServer.args={{"enabled","0"}};auto sent=wire.size();setCalibration();assert(benchServer.code==200&&controlParams.minMeasured==1000&&controlParams.maxMeasured==3000&&wire.size()==sent);
 saveSucceeds=false;benchServer.args={{"enabled","1"}};setCalibration();assert(benchServer.code==500&&!controlParams.calibration);
 // An old HTTP form cannot restore receiver-only mode locally.
 saveSucceeds=true;goodRequest();benchServer.args["slider-enable"]="0";setSliders();assert(benchServer.code==200&&controlParams.enable==1&&boardsParams.slave1.enable==1);
 goodRequest();benchServer.args["slider-enable"]="2";auto priorSaves=saves;setSliders();assert(benchServer.code==400&&saves==priorSaves);
}
'''
run('actual transfer, retry, ACK sender/transaction checks, persistence failure, duplicate handling, calibration isolation',code)
# Retain actual acquisition coverage from R5.2 test, whose sensing is unchanged.
a=(R/'tests/run_controller_r52_calibration_tests.py').read_text();start=a.index('sensor=fixture(');end=a.index('bench=(P/',start)
exec(a[start:end].replace('function(SOURCE,n)','fn(n)'))
assert 'sendPeer(0);sendPeer(1)' not in source
assert 'macAddresses[n]=address;sendPeer' not in source
print('PASS no boot/pairing settings push')
# Exercise the real calibration transition and endpoint-pause implementation.
s=fixture('settings_stubs').replace('void applyLocalParams(const dataStruct& next){++applies;controlParams=next;}','')+'''
#include "calibration_session.h"
#include "remote_setup_protocol.h"
struct SetupTransfer {SetupPacket request{};bool available=false,pending=false;};SetupTransfer setupTransfers[2];
#include <vector>
CalibrationSession calibrationSession;String calibrationNotice;
std::vector<int> rawValues;bool otaInProgress=false,benchActive=false,servoTimeOut=false;
unsigned servoTimeOutStart=0,moves=0;
unsigned millis(){return 123;}void enterBenchMode(){benchActive=true;}
void writeServoCommand(int){++moves;}
struct FakePid{enum class Control{manual,timer};void SetTunings(float,float,float){}void SetMode(Control){}} elevatorPid;
using QuickPID=FakePid;
'''
s+='\n'.join(fn(n) for n in ['applyLocalParams','changeCalibration','setCalibration'])+r'''
void toggle(int n){benchServer.args={{"enabled",n?"1":"0"}};setCalibration();}
int main(){
 toggle(1);assert(benchServer.code==200&&benchActive&&calibrationSession.active);calibrationSession.observe(1000);calibrationSession.observe(9000);
 saveSucceeds=false;toggle(0);assert(benchServer.code==500&&calibrationSession.active&&controlParams.calibration);
 saveSucceeds=true;toggle(0);assert(!calibrationSession.active&&!controlParams.calibration&&controlParams.minMeasured==1000&&controlParams.maxMeasured==9000);
 toggle(1);toggle(0);assert(controlParams.minMeasured==1000&&controlParams.maxMeasured==9000);
 benchActive=false;auto next=controlParams;next.servoMin=2000;next.servoMax=1000;applyLocalParams(next);assert(benchActive&&moves==0);
}
'''
run('real calibration start/finish/failed save/empty range, reversal pauses with no endpoint movement',s)

# Actual boot normalization and actual scheduler callback: old mode flags cannot
# bypass sensor control; the on-disk schema and other per-board settings survive.
normal=base+fn('useWaterHeightControl')+r"""
int sensorSteps=0;
void calculatePid(){++sensorSteps;}
constexpr int MILLIS=1;
struct TickTwo {void(*step)();TickTwo(void(*f)(),int period,int count,int units):step(f){assert(period==10&&count==0&&units==MILLIS);}};
"""+source[source.index('TickTwo pidTicker('):source.index('TickTwo rmtTicker(')]+r"""
int main(){
 (void)pdTRUE;auto before=boardsParams;before.master.enable=0;before.slave1.enable=0;before.slave2.enable=0;
 before.master.factor=0;before.slave1.factor=37;before.slave2.factor=100;before.slave1.servoMin=1800;before.slave1.servoMax=1200;
 auto expected=before;expected.master.enable=expected.slave1.enable=expected.slave2.enable=1;
 useWaterHeightControl(before);assert(std::memcmp(&before,&expected,sizeof(expected))==0&&saves==0);
 controlParams.enable=0;pidTicker.step();assert(sensorSteps==1);
 controlParams.enable=1;pidTicker.step();assert(sensorSteps==2);
}
"""
run('legacy boot mode retired without writes or tuning/calibration loss; scheduler always selects sensor control',normal)
