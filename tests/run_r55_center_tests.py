from pathlib import Path
import ast
R=Path(__file__).resolve().parents[1]
# Reuse only host harness definitions; execute the actual new firmware functions.
scope={'__file__':str(R/'tests/run_r55_tests.py')}
exec((R/'tests/run_r55_tests.py').read_text().split("code=base+")[0],scope)
original_fixture=scope['fixture']
def extended_fixture(n):
 text=original_fixture(n)
 if n=='filesystem_arduino_stub':
  text=text.replace('String(const std::string& text): value(text) {}','String(const std::string& text): value(text) {}\n  String(unsigned int n):value(std::to_string(n)){}\n  long toInt() const{return std::stol(value);}')
 return text
scope['fixture']=extended_fixture
run,fn=scope['run'],scope['fn']
tree=ast.parse((R/'tests/run_r55_regression_tests.py').read_text())
def literal(name):
 for n in tree.body:
  if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets):return ast.literal_eval(n.value)
code=literal('modes_stubs')+'\n#include "controller_validation.h"\n#include "calibration_session.h"\n'+'\n'.join(fn(n) for n in ['enterBenchMode','holdServoTest','directedTestPulse','centerServoTest','writeServoCommand','resumeController','performServoTest'])+r'''
int main(){
 for(auto endpoints: {std::pair<int,int>{1000,2000},{1809,1207},{1700,1900},{1200,1000}}){
  controlParams.servoMin=endpoints.first;controlParams.servoMax=endpoints.second;
  auto before=controlParams;
  assert(centerServoTest());assert(benchActive&&servoCenterTest&&elevator.attached());
  assert(servoPos==1500&&elevator.value==1500&&elevatorPid.mode==QuickPID::Control::manual);
  assert(!std::memcmp(&before,&controlParams,sizeof(before)));
  writeServoCommand(1000);assert(elevator.value==1500);
  enterBenchMode();assert(!servoCenterTest&&!elevator.attached());
 }
 for(unsigned pulse: {1000u,1400u,1500u,1600u,2000u}){assert(holdServoTest(pulse));assert(servoPos==int(pulse)&&elevator.value==int(pulse));writeServoCommand(1800);assert(elevator.value==int(pulse));}
 for(bool reversed: {false,true}){controlParams.servoMin=reversed?1809:1207;controlParams.servoMax=reversed?1207:1809;
  for(unsigned requested: {1000u,1400u,1500u,1600u,2000u}){assert(performServoTest(requested));assert(elevator.value==int(reversed?3000-requested:requested));}
  assert(centerServoTest());assert(elevator.value==1500);
 }
 assert(!holdServoTest(999)&&!holdServoTest(2001));
 assert(centerServoTest());assert(elevator.value==1500);assert(resumeController());assert(!servoCenterTest&&!benchActive);
 assert(centerServoTest());enterBenchMode();otaInProgress=true;assert(!centerServoTest());otaInProgress=false;
 controlParams.calibration=1;assert(!centerServoTest());controlParams.calibration=0;
 elevator.failAttach=true;assert(!centerServoTest()&&!servoCenterTest&&!elevator.attached());
 elevator.failAttach=false;controlParams.minMeasured=0;controlParams.maxMeasured=0;
 assert(centerServoTest());assert(!resumeController());assert(servoCenterTest&&elevator.value==1500);
 assert(performServoTest(2));assert(!elevator.attached());assert(!performServoTest(99));
}
'''
run('fixed center independent of endpoints; pause, resume, calibration, OTA and attach failure',code)
base=scope['base']+r'''
#include "servo_test_protocol.h"
struct ReceivedTest{uint8_t sender[6];ServoTestPacket packet;};
struct TestTransfer{ServoTestPacket packet{};bool pending=false;unsigned attempts=0;uint32_t sentAt=0;String status="Not requested";};
TestTransfer testTransfers[2];ServoTestPacket lastTest{};uint8_t lastTestSender[6]={};bool haveLastTest=false;
uint32_t testSequence=0;int receivedTests=2;std::deque<ReceivedTest> testQueue;std::vector<ServoTestPacket> testWire;
int xQueueReceive(int,ReceivedTest*p,int){if(testQueue.empty())return 0;*p=testQueue.front();testQueue.pop_front();return 1;}
bool servoCenterTest=false,benchActive=true;int servoPos=0,actions=0;bool succeed=true;
struct Servo {bool attached(){return servoCenterTest||!benchActive;}} elevator;
bool performServoTest(uint32_t a){++actions;if(!succeed)return false;servoCenterTest=a==1||a>=1000;benchActive=a!=3;servoPos=a>=1000?(controlParams.servoMax<controlParams.servoMin?3000-a:a):1500;return true;}
'''
# Replace only the fake transport, so real send/receive/retry functions run unchanged.
a=base.index('int esp_now_send(');b=base.index('\nvoid enterBenchMode',a)
base=base[:a]+'''int esp_now_send(const uint8_t*,const uint8_t*p,size_t n){assert(n==sizeof(ServoTestPacket));ServoTestPacket t;std::memcpy(&t,p,n);testWire.push_back(t);return 0;}\n'''+base[b:]
# Function references testWire defined later; move declaration ahead of transport.
base=base.replace('std::vector<ServoTestPacket> testWire;','')
base=base.replace('int esp_now_send(', '#include "servo_test_protocol.h"\nstd::vector<ServoTestPacket> testWire;\nint esp_now_send(')
code=base+'\n'.join(fn(n) for n in ['directedTestPulse','testActionName','transmitServoTest','sendServoTest','serviceServoTests','requestServoTest'])+r'''
int main(){
 sendServoTest(0,1);auto request=testWire.back();ServoTestPacket decoded;
 assert(decodeServoTestPacket((uint8_t*)&request,sizeof(request),decoded));
 auto old=request;old.version=1;assert(!decodeServoTestPacket((uint8_t*)&old,sizeof(old),decoded));
 auto ack=request;ack.kind=2;ack.actualPulse=1500;assert(decodeServoTestPacket((uint8_t*)&ack,sizeof(ack),decoded));
 ack.actualPulse=1600;assert(!decodeServoTestPacket((uint8_t*)&ack,sizeof(ack),decoded));
 ack.action=1600;ack.actualPulse=1400;assert(decodeServoTestPacket((uint8_t*)&ack,sizeof(ack),decoded));
 ack.actualPulse=1300;assert(!decodeServoTestPacket((uint8_t*)&ack,sizeof(ack),decoded));
 auto bad=request;bad.action=4;assert(!decodeServoTestPacket((uint8_t*)&bad,sizeof(bad),decoded));
 assert(!decodeServoTestPacket((uint8_t*)&request,sizeof(request)-1,decoded));
 ReceivedTest rx{};parseMacAddress("CC:BA:97:47:BE:28",rx.sender);rx.packet=request;slave=true;
 testQueue.push_back(rx);serviceServoTests();assert(actions==1&&servoCenterTest&&testWire.back().result==0);
 testQueue.push_back(rx);serviceServoTests();assert(actions==1&&testWire.back().result==0);
 servoCenterTest=false;testQueue.push_back(rx);serviceServoTests();assert(actions==1&&testWire.back().result==1);
 rx.packet.sequence++;rx.packet.action=2;testQueue.push_back(rx);serviceServoTests();assert(actions==2&&!servoCenterTest);
 rx.packet=request;testQueue.push_back(rx);serviceServoTests();assert(actions==2&&testWire.back().result==1);
 slave=false;rx.packet=request;rx.packet.kind=2;parseMacAddress("DC:06:75:32:AB:B5",rx.sender);
 testQueue.push_back(rx);serviceServoTests();assert(testTransfers[0].pending);
 parseMacAddress("DC:06:75:32:AB:B4",rx.sender);rx.packet.action=3;testQueue.push_back(rx);serviceServoTests();assert(testTransfers[0].pending);
 rx.packet.action=1;testQueue.push_back(rx);serviceServoTests();assert(!testTransfers[0].pending);
 sendServoTest(0,1);now=1000;serviceServoTests();now=2000;serviceServoTests();now=3000;serviceServoTests();assert(!testTransfers[0].pending&&testTransfers[0].attempts==3);
 assert(std::strstr(testTransfers[0].status.c_str(),"UNCONFIRMED")!=nullptr);
 // Manual requests move only the selected board and ACK matches the pulse.
 benchServer.args={{"action","pulse"},{"scope","slave-1"},{"pulse","1600"}};auto before=actions;
 requestServoTest();assert(benchServer.code==200&&actions==before&&testWire.back().action==1600);
 rx.packet=testWire.back();rx.packet.kind=2;rx.packet.action=1500;testQueue.push_back(rx);serviceServoTests();assert(testTransfers[0].pending);
 rx.packet.action=1600;testQueue.push_back(rx);serviceServoTests();assert(!testTransfers[0].pending);
 benchServer.args["scope"]="local";requestServoTest();assert(benchServer.code==200&&servoPos==1600);
 for(const char* value: {"999","2001","1500.5","1e03","-1500","abcd",""}){benchServer.args["pulse"]=value;requestServoTest();assert(benchServer.code==400);}
 slave=true;benchServer.args={{"action","center"},{"scope","all"}};requestServoTest();assert(benchServer.code==400);
 benchServer.args["scope"]="local";requestServoTest();assert(benchServer.code==200&&servoCenterTest);
 controlParams.servoMin=1809;controlParams.servoMax=1207;
 rx.packet=request;rx.packet.sequence=100;rx.packet.action=1600;rx.packet.kind=1;
 parseMacAddress("CC:BA:97:47:BE:28",rx.sender);testQueue.push_back(rx);serviceServoTests();
 assert(servoPos==1400&&testWire.back().actualPulse==1400&&testWire.back().result==0);
 testQueue.push_back(rx);serviceServoTests();assert(testWire.back().result==0&&testWire.back().actualPulse==1400);
 assert(decodeServoTestPacket((uint8_t*)&testWire.back(),sizeof(ServoTestPacket),decoded));
}
'''
run('test radio decode, sender/action ACK checks, duplicate/stale protection, bounded retry and scope',code)
