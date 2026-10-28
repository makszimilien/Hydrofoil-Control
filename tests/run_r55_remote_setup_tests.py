"""Exercise actual R5.5 remote setup implementation, using fake radio/storage only."""
from pathlib import Path
import re
R=Path(__file__).resolve().parents[1]
scope={'__file__':str(R/'tests/run_r55_tests.py')}
exec((R/'tests/run_r55_tests.py').read_text().split('code=base+')[0],scope)
run,fn,base,source=scope['run'],scope['fn'],scope['base'],scope['source']
base=base.replace('struct SetupTransfer {SetupPacket request{};bool available=false,pending=false;};SetupTransfer setupTransfers[2];','')
base=base.replace('void applyLocalParams(const dataStruct& next){++applies;controlParams=next;}','')
a=base.index('int esp_now_send(')
base=base[:a]+r'''
using QueueHandle_t=int;
'''+source[source.index('struct ReceivedSetup {'):source.index('struct IdentityPacket {')].replace('receivedSetup=nullptr','receivedSetup=0')+r'''
std::deque<ReceivedSetup> setupQueue;std::vector<SetupPacket> setupWire;
int xQueueReceive(int,ReceivedSetup*p,int){if(setupQueue.empty())return 0;*p=setupQueue.front();setupQueue.pop_front();return 1;}
int esp_now_send(const uint8_t*,const uint8_t*p,size_t n){assert(n==sizeof(SetupPacket));SetupPacket copy;std::memcpy(&copy,p,n);setupWire.push_back(copy);return 0;}
bool benchActive=false,servoCenterTest=false,sensorHealthy=true,positionValid=true,receiverValid=false,servoTimeOut=false;
uint32_t sensorLastTicks=12345,sensorGoodSamples=456,sensorTimeouts=2,servoTimeOutStart=0;
int median=12000,position=1510,servoPos=1500,pwmRead=1480;
String calibrationNotice,controllerError;std::vector<int> rawValues;
struct Servo {bool attached(){return !benchActive||servoCenterTest;}} elevator;
struct FakePid{enum class Control{manual,timer};void SetTunings(float,float,float){}void SetMode(Control){}} elevatorPid;
using QuickPID=FakePid;
void enterBenchMode(){benchActive=true;servoCenterTest=false;}
void writeServoCommand(int){assert(false&&"Calibration must not command a servo");}
bool calibrationSampling(){return settingsReady&&controlParams.calibration&&calibrationSession.active;}
'''
code=base+'\n'.join(fn(n) for n in ['applyLocalParams','changeCalibration','captureSetupState','calibrationResult','transmitSetup','beginSetupRequest','answerSetupRequest','serviceRemoteSetup'])+r'''
ReceivedSetup packetFrom(const char*mac,SetupPacket p){ReceivedSetup rx{};parseMacAddress(mac,rx.sender);rx.packet=p;return rx;}
int main(){
 static_assert(sizeof(SetupPacket)<=250,"radio limit");receivedSetup=1;setupSession=777;
 const char*master="CC:BA:97:47:BE:28";const char*peer="DC:06:75:32:AB:B4";
 // Master asks for data; browsing must not write settings or change modes.
 slave=false;assert(beginSetupRequest(0,0));auto query=setupWire.back();auto saveCount=saves;auto oldMode=benchActive;
 assert(!beginSetupRequest(0,0));
 slave=true;controlParams=boardsParams.slave1;controlParams.p=2.7;controlParams.servoMin=1201;controlParams.servoMax=1796;boardsParams.slave1=controlParams;
 auto rx=packetFrom(master,query);answerSetupRequest(rx);auto snapshot=setupWire.back();
 assert(saves==saveCount&&benchActive==oldMode);assert(snapshot.settings.p==2.7f&&snapshot.settings.servoMin==1201&&snapshot.raw==12345&&snapshot.good==456);
 SetupPacket decoded;assert(decodeSetupPacket((uint8_t*)&snapshot,sizeof(snapshot),decoded));
 auto bad=snapshot;bad.version++;assert(!decodeSetupPacket((uint8_t*)&bad,sizeof(bad),decoded));bad=snapshot;bad.notice[63]='x';assert(!decodeSetupPacket((uint8_t*)&bad,sizeof(bad),decoded));bad=snapshot;bad.settings.p=NAN;assert(!decodeSetupPacket((uint8_t*)&bad,sizeof(bad),decoded));
 slave=false;rx=packetFrom(master,snapshot);setupQueue.push_back(rx);serviceRemoteSetup();assert(setupTransfers[0].pending);
 rx=packetFrom(peer,snapshot);rx.packet.action=1;setupQueue.push_back(rx);serviceRemoteSetup();assert(setupTransfers[0].pending);
 rx.packet=snapshot;rx.packet.sequence++;setupQueue.push_back(rx);serviceRemoteSetup();assert(setupTransfers[0].pending);
 rx.packet=snapshot;setupQueue.push_back(rx);serviceRemoteSetup();assert(!setupTransfers[0].pending&&setupTransfers[0].available&&setupTransfers[0].snapshot.settings.p==2.7f);
 // Start calibration and retain the slave's own tuning/direction. Duplicate retry never resets collected samples.
 assert(beginSetupRequest(0,1));auto start=setupWire.back();assert(!beginSetupRequest(0,2));slave=true;rx=packetFrom(master,start);answerSetupRequest(rx);
 assert(controlParams.calibration&&calibrationSession.active&&benchActive);assert(controlParams.servoMin==1201&&controlParams.p==2.7f);
 calibrationSession.observe(3000);calibrationSession.observe(15000);saveCount=saves;answerSetupRequest(rx);assert(saves==saveCount&&calibrationSession.samples==2);
 auto startAck=setupWire.back();slave=false;rx=packetFrom(peer,startAck);setupQueue.push_back(rx);serviceRemoteSetup();assert(!setupTransfers[0].pending&&setupTransfers[0].snapshot.flags&SetupCalActive);
 // Finish saves the measured range on the slave only. No caller-supplied range is trusted.
 auto masterBefore=boardsParams.master;assert(beginSetupRequest(0,2));auto finish=setupWire.back();finish.settings.minMeasured=1;finish.settings.maxMeasured=2;
 slave=true;rx=packetFrom(master,finish);answerSetupRequest(rx);assert(!calibrationSession.active&&!controlParams.calibration);assert(controlParams.minMeasured==3000&&controlParams.maxMeasured==15000);assert(!std::memcmp(&masterBefore,&boardsParams.master,sizeof(masterBefore)));assert(benchActive&&setupWire.back().result==0);
 saveCount=saves;answerSetupRequest(rx);assert(saves==saveCount);
 rx.packet=start;answerSetupRequest(rx);assert(setupWire.back().result==3&&!controlParams.calibration&&saves==saveCount);
 // Storage failure keeps the prior state and is acknowledged as failure, not success.
 rx.packet=finish;rx.packet.sequence++;rx.packet.action=1;saveSucceeds=false;answerSetupRequest(rx);assert(setupWire.back().result==2&&!controlParams.calibration);saveCount=saves;answerSetupRequest(rx);assert(saves==saveCount);saveSucceeds=true;
 rx.packet.sequence++;answerSetupRequest(rx);assert(calibrationSession.active);rx.packet.sequence++;rx.packet.action=2;answerSetupRequest(rx);assert(setupWire.back().result==4&&controlParams.minMeasured==3000&&!controlParams.calibration);
 // Three bounded sends, no false confirmation, and late replies cannot revive expired state.
 slave=false;setupTransfers[0].pending=false;setupWire.clear();now=0;assert(beginSetupRequest(0,1));auto expiring=setupWire.back();now=750;serviceRemoteSetup();now=1500;serviceRemoteSetup();assert(setupWire.size()==3);now=2250;serviceRemoteSetup();assert(!setupTransfers[0].pending&&!setupTransfers[0].available);
 expiring.kind=2;rx=packetFrom(peer,expiring);setupQueue.push_back(rx);serviceRemoteSetup();assert(!setupTransfers[0].available);now=9999;serviceRemoteSetup();assert(setupWire.size()==3);
 // A calibration command supersedes an in-flight read; old read replies cannot finish that command.
 assert(beginSetupRequest(0,0));auto oldRead=setupWire.back();assert(beginSetupRequest(0,2));oldRead.kind=2;rx=packetFrom(peer,oldRead);setupQueue.push_back(rx);serviceRemoteSetup();assert(setupTransfers[0].pending&&setupTransfers[0].request.action==2);
}
'''
run('actual remote settings snapshot, local calibration start/finish/save failures, duplicate/stale protection, packet validation, matching ACKs and bounded retries',code)
