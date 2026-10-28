"""Host checks for the R5.1 explicit controller-default recovery handler.

Extracts the actual handler into an instrumented host harness. Persistence and
Arduino primitives are stubs; the preserved R5 suite separately tests the real
file parser/writer. This script never contacts or uploads to a physical board.
"""
import ast
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT
SOURCE = (PROJECT / "src/main.cpp").read_text()


def function(source, name):
    match = re.search(r"^[\w:*& <>]+\b" + re.escape(name) + r"\([^;]*?\)\s*\{", source, re.M)
    if not match:
        raise AssertionError("Missing actual firmware function: " + name)
    opening = source.index("{", match.start())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def baseline_fixture(name):
    """Read a literal fixture without importing/running the larger R5 suite."""
    tree = ast.parse((ROOT / "tests/run_controller_r5_tests.py").read_text())
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in statement.targets
        ):
            return ast.literal_eval(statement.value)
    raise AssertionError("Missing preserved host fixture: " + name)


arduino_stub = baseline_fixture("filesystem_arduino_stub")
fs_stub = baseline_fixture("filesystem_fs_stub")

stubs = r'''
#include "controller_validation.h"
#include "calibration_session.h"
CalibrationSession calibrationSession;String calibrationNotice;
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <vector>
FakeSerial Serial;
fs::FS SPIFFS;
int lockDepth=0;
struct ControlLock {ControlLock(){++lockDepth;}~ControlLock(){--lockDepth;}};
bool benchActive=true,otaInProgress=false,benchFilesystem=true,settingsReady=false;
bool first=false,slave=false,restartPending=false;
bool sensorHealthy=true,positionValid=true,receiverValid=true,servoTimeOut=true;
bool simulateSensorTimeout=false,benchAutomatic=true,benchSingle=true;
int control=180,pwmRead=1900,median=12000,position=1450,servoPos=1850;
uint32_t receiverLastValid=1234;
float input=1450,output=120,setpoint=1700;
std::vector<int> rawValues(20,12000);
const char *jsonConfigsPath="/configs.json",*jsonWifiPath="/wifi.json";
String controllerError="Invalid settings",boardSelector="slave-2";
String macAddresses[2]={"CC:BA:97:47:BE:28","DC:06:75:34:23:D2"};
struct FakePid {
 enum class Control{manual,timer};Control mode=Control::timer;
 unsigned resets=0,tunings=0;uint32_t sampleUs=0;
 float integral=300,p=0,i=0,d=0,low=0,high=0;
 void SetMode(Control value){mode=value;}
 void SetTunings(float nextP,float nextI,float nextD){++tunings;p=nextP;i=nextI;d=nextD;}
 void SetSampleTimeUs(uint32_t value){sampleUs=value;}
 void SetOutputLimits(float minimum,float maximum){low=minimum;high=maximum;}
 void Reset(){++resets;integral=0;}
} elevatorPid;
using QuickPID=FakePid;
struct FakeServo {
 bool connected=false;unsigned attaches=0,writes=0,detaches=0;
 bool attached(){return connected;}
 void detach(){connected=false;++detaches;}
 void attach(int,int,int){++attaches;connected=true;}
 void writeMicroseconds(int){++writes;}
} elevator;
unsigned stops=0,resumes=0,peerSends=0,configWrites=0,configReads=0,wifiReads=0,otherWrites=0;
bool saveSucceeds=true,readSucceeds=true;
int mismatchBoard=-1,mismatchField=-1;
bool invalidReadback=false;
std::vector<std::string> operations;
dataStruct values(int offset){return {1.25f+offset*0.25f,0.25f,0.125f,1600+offset*25,0,
 1100+offset*25,1900-offset*25,30+offset,0,3000+offset*100,23000+offset*100};}
dataStruct defaults(){return {1,0,0,1500,1,1000,2000,40,0,2000,24000};}
allBoards boardsParams={values(0),values(1),values(2)},savedConfigs{};
dataStruct controlParams=values(2);
struct FakeWiFi {String station="DC:06:75:34:23:D0";String macAddress(){return station;}} WiFi;
struct FakeServer {
 std::map<std::string,String> args;int code=0;unsigned responses=0;
 bool hasArg(const char *name){return args.count(name);}
 String arg(const char *name){auto it=args.find(name);return it==args.end()?String():it->second;}
 void send(int value,const char*,const String&){code=value;++responses;}
 void send(int value){code=value;++responses;}
} benchServer;
std::map<std::string,String> flags;
void stopBenchSamples(){++stops;benchAutomatic=false;benchSingle=false;
 sensorHealthy=false;positionValid=false;rawValues.clear();}
bool resumeController(){++resumes;benchActive=false;elevator.attach(1,1000,2000);return true;}
void sendPeer(unsigned){++peerSends;}
void redirect(const char*){benchServer.send(303);}
void *rb=reinterpret_cast<void*>(1),*receivedConfigs=reinterpret_cast<void*>(2);
unsigned pendingRmt=2,returnedRmt=0,pendingConfigs=2,queueResets=0;
int stalePulse=1800;
void *xRingbufferReceive(void*,size_t *bytes,int){if(!pendingRmt)return nullptr;--pendingRmt;
 *bytes=sizeof(stalePulse);return &stalePulse;}
void vRingbufferReturnItem(void*,void*){++returnedRmt;}
constexpr int pdTRUE=1;
int xQueueReceive(void*,dataStruct *data,int){if(!pendingConfigs)return 0;--pendingConfigs;*data=values(2);return pdTRUE;}
int xQueueReset(void*){++queueResets;pendingConfigs=0;return pdTRUE;}
String readFileJson(fs::FS &fs,const char *path,const char *property){
 assert(&fs==&SPIFFS && std::string(path)=="/wifi.json");++wifiReads;
 operations.push_back(std::string("read-flag:")+property);
 auto found=flags.find(property);return found==flags.end()?String():found->second;
}
bool writeStructJson(fs::FS &fs,const char *path,allBoards &next){
 assert(&fs==&SPIFFS && std::string(path)=="/configs.json");++configWrites;
 operations.push_back("write-configs");
 if(!saveSucceeds)return false;
 savedConfigs=next;SPIFFS.files[path]="only-configs-replaced";return true;
}
void alter(dataStruct &data,int field){
 switch(field){
  case 0:data.p=1.25f;break;case 1:data.i=0.25f;break;case 2:data.d=0.125f;break;
  case 3:data.setpoint=1501;break;case 4:data.enable=0;break;
  case 5:data.servoMin=1001;break;case 6:data.servoMax=1999;break;
  case 7:data.factor=41;break;case 8:data.calibration=1;break;
  case 9:data.minMeasured=2001;break;case 10:data.maxMeasured=24001;break;
  default:assert(false);
 }
}
bool readStructJson(fs::FS &fs,const char *path,allBoards &result){
 assert(&fs==&SPIFFS && std::string(path)=="/configs.json");++configReads;
 operations.push_back("read-configs");if(!readSucceeds)return false;
 result=savedConfigs;
 if(mismatchBoard>=0){dataStruct *part=mismatchBoard==0?&result.master:mismatchBoard==1?&result.slave1:&result.slave2;alter(*part,mismatchField);}
 if(invalidReadback)result.slave2.p=std::numeric_limits<float>::quiet_NaN();
 return true;
}
bool writeFileJson(fs::FS&,const char*,const char*,const char*){++otherWrites;return false;}
bool writeArrayJson(fs::FS&,const char*,const char*,String*,int){++otherWrites;return false;}
void same(const dataStruct &a,const dataStruct &b){assert(std::memcmp(&a,&b,sizeof(a))==0);}
void same(const allBoards &a,const allBoards &b){same(a.master,b.master);same(a.slave1,b.slave1);same(a.slave2,b.slave2);}
void resetCase(){
 benchActive=true;otaInProgress=false;benchFilesystem=true;settingsReady=false;
 first=false;slave=false;restartPending=false;WiFi.station="DC:06:75:34:23:D0";
 sensorHealthy=true;positionValid=true;receiverValid=true;servoTimeOut=true;
 simulateSensorTimeout=false;benchAutomatic=true;benchSingle=true;
 control=180;pwmRead=1900;median=12000;position=1450;servoPos=1850;receiverLastValid=1234;
 input=1450;output=120;setpoint=1700;rawValues.assign(20,12000);
 controllerError="Invalid settings";boardSelector="slave-2";
 elevatorPid=FakePid{};elevator=FakeServo{};
 stops=0;resumes=0;peerSends=0;configWrites=0;configReads=0;wifiReads=0;otherWrites=0;
 saveSucceeds=true;readSucceeds=true;mismatchBoard=-1;mismatchField=-1;invalidReadback=false;
 boardsParams={values(0),values(1),values(2)};controlParams=values(2);savedConfigs={};
 rb=reinterpret_cast<void*>(1);receivedConfigs=reinterpret_cast<void*>(2);
 pendingRmt=2;returnedRmt=0;pendingConfigs=2;queueResets=0;
 flags={{"first","False"},{"slave","False"}};
 SPIFFS=fs::FS{};
 SPIFFS.files={{"/configs.json","old-invalid-configs"},{"/wifi.json","unchanged-private-wifi-data"},
  {"/addresses.json","unchanged-peer-addresses"},{"/recording.csv","unchanged-recording"},
  {"/index.html","unchanged-legacy-page"}};
 operations.clear();benchServer=FakeServer{};
 benchServer.args={{"expected-mac","DC:06:75:34:23:D0"},{"confirm","reset-controller-settings"}};
}
void unaffectedFiles(){
 assert(otherWrites==0 && SPIFFS.writeOpens==0 && SPIFFS.renameCalls==0);
 assert(SPIFFS.files.size()==5);
 assert(SPIFFS.files.at("/wifi.json")=="unchanged-private-wifi-data");
 assert(SPIFFS.files.at("/addresses.json")=="unchanged-peer-addresses");
 assert(SPIFFS.files.at("/recording.csv")=="unchanged-recording");
 assert(SPIFFS.files.at("/index.html")=="unchanged-legacy-page");
 assert(macAddresses[0]=="CC:BA:97:47:BE:28" && macAddresses[1]=="DC:06:75:34:23:D2");
 assert(resumes==0 && peerSends==0 && !restartPending);
 assert(elevator.attaches==0 && elevator.writes==0 && !elevator.attached());
 assert(lockDepth==0 && benchServer.responses==1);
}
'''

cases = r'''
void rejectedWithoutWrites(){
 const auto oldBoards=boardsParams;const auto oldControl=controlParams;const bool oldReady=settingsReady;
 restoreControllerDefaults();
 assert(benchServer.code>=400 && benchServer.code<600);
 assert(configWrites==0 && configReads==0 && SPIFFS.files.at("/configs.json")=="old-invalid-configs");
 same(boardsParams,oldBoards);same(controlParams,oldControl);assert(settingsReady==oldReady);
 unaffectedFiles();
}
void rejectedAfterWrite(){
 const auto oldBoards=boardsParams;const auto oldControl=controlParams;const bool oldReady=settingsReady;
 restoreControllerDefaults();
 assert(benchServer.code>=400 && benchServer.code<600 && configWrites==1);
 same(boardsParams,oldBoards);same(controlParams,oldControl);assert(settingsReady==oldReady);
 assert(benchActive);unaffectedFiles();
}
void success(){
 const bool oldFirst=first,oldSlave=slave;const auto oldFlags=flags;
 restoreControllerDefaults();
 assert(benchServer.code>=200 && benchServer.code<300 && configWrites==1 && configReads==1);
 same(boardsParams.master,defaults());same(boardsParams.slave1,defaults());same(boardsParams.slave2,defaults());
 same(savedConfigs,boardsParams);same(controlParams,defaults());
 assert(validControlParams(controlParams) && settingsReady && benchActive);
 assert(first==oldFirst && slave==oldSlave && flags==oldFlags);
 assert(elevatorPid.mode==QuickPID::Control::manual && output==0);
 assert(elevatorPid.resets==1 && elevatorPid.integral==0 && elevatorPid.sampleUs==10000);
 assert(elevatorPid.tunings==1 && elevatorPid.p==1 && elevatorPid.i==0 && elevatorPid.d==0);
 assert(elevatorPid.low==-500 && elevatorPid.high==500 && input==1500 && setpoint==1500);
 assert(!sensorHealthy && !positionValid && rawValues.empty() && median==0 && position==0);
 assert(!receiverValid && control==0 && pwmRead==0 && !servoTimeOut);
 assert(!benchAutomatic && !benchSingle);
 assert(controllerError.isEmpty());
 const auto write=std::find(operations.begin(),operations.end(),"write-configs");
 assert(write!=operations.end());
 assert(std::find(operations.begin(),write,"read-flag:first")!=write);
 assert(std::find(operations.begin(),write,"read-flag:slave")!=write);
 assert(std::find(write,operations.end(),"read-configs")!=operations.end());
 unaffectedFiles();
}
int main(){
 static_assert(sizeof(dataStruct)==44,"Controller settings layout changed");
 for(int gate=0;gate<3;++gate){resetCase();if(gate==0)benchActive=false;
  if(gate==1)otaInProgress=true;if(gate==2)benchFilesystem=false;rejectedWithoutWrites();}
 std::cout << "PASS: active-control, OTA and missing-filesystem gates reject without writes\n";
 for(const char *bad:{"", "DC:06:75:34:23:D1", "CC:BA:97:47:BE:28", "DC:06:75:34:23",
  "DC:06:75:34:23:D0:00", "DC:06:75:34:23:DG", "00:00:00:00:00:00", "FF:FF:FF:FF:FF:FF"}){
  resetCase();benchServer.args["expected-mac"]=bad;rejectedWithoutWrites();}
 resetCase();benchServer.args.erase("expected-mac");rejectedWithoutWrites();
 for(const char *bad:{"", "yes", "reset-controller-settings ", "reset-controller-setting"}){
  resetCase();benchServer.args["confirm"]=bad;rejectedWithoutWrites();}
 resetCase();benchServer.args.erase("confirm");rejectedWithoutWrites();
 std::cout << "PASS: wrong/missing/malformed target identity and inexact confirmation reject without writes\n";
 for(const char *property:{"first","slave"}){
  for(const char *bad:{"", "yes", "FALSE", "false ", "2", "null"}){
   resetCase();flags[property]=bad;rejectedWithoutWrites();}
  resetCase();flags.erase(property);rejectedWithoutWrites();
 }
 std::cout << "PASS: both saved role/setup flags must validate before controller settings can be overwritten\n";
 for(int ready=0;ready<2;++ready){
  resetCase();settingsReady=ready;saveSucceeds=false;rejectedAfterWrite();assert(configReads==0);
  assert(SPIFFS.files.at("/configs.json")=="old-invalid-configs");
  resetCase();settingsReady=ready;readSucceeds=false;elevator.connected=true;
  rejectedAfterWrite();assert(configReads==1);
 }
 for(int board=0;board<3;++board)for(int field=0;field<11;++field){
  resetCase();mismatchBoard=board;mismatchField=field;rejectedAfterWrite();assert(configReads==1);
 }
 resetCase();invalidReadback=true;rejectedAfterWrite();
 std::cout << "PASS: write/read failures and each of 33 wrong readback fields retain previous in-memory settings\n";
 for(int role=0;role<2;++role)for(int setup=0;setup<2;++setup){
  resetCase();first=setup;slave=role;flags["first"]=setup?"True":"False";
  flags["slave"]=role?"True":"False";success();
 }
 for(const char *falseFlag:{"false","False","0"})for(const char *trueFlag:{"true","True","1"}){
  resetCase();flags["first"]=falseFlag;flags["slave"]=trueFlag;slave=true;success();
  resetCase();flags["first"]=trueFlag;flags["slave"]=falseFlag;first=true;success();
 }
 resetCase();benchServer.args["expected-mac"]="dc-06-75-34-23-d0";success();
 resetCase();elevator.connected=true;success();
 resetCase();settingsReady=true;success();
 resetCase();boardsParams={};controlParams={};success();
 std::cout << "PASS: all 33 factory values restore; master/slave/setup flags and unrelated saved files remain unchanged\n";
 std::cout << "PASS: normalized identity accepted; sensing/PID/receiver state cleared with servo detached and no resume/restart\n";
}
'''


handler = function(SOURCE, "restoreControllerDefaults")
routes = function(SOURCE, "configureRoutes")
assert re.search(r'benchServer\.on\("/controller/defaults",\s*HTTP_POST,\s*restoreControllerDefaults\)', routes)
assert not re.search(r'benchServer\.on\("/controller/defaults",\s*HTTP_GET', routes)
assert "restoreControllerDefaults(" not in function(SOURCE, "setup")
assert len(re.findall(r"\brestoreControllerDefaults\s*\(", SOURCE)) == 1
assert "writeStructJson(" not in function(SOURCE, "setup")
assert "writeFileJson(" not in function(SOURCE, "setup")

with tempfile.TemporaryDirectory(prefix="hydrofoil-r51-defaults-") as directory:
    temp = Path(directory)
    (temp / "Arduino.h").write_text(arduino_stub)
    (temp / "SPIFFS.h").write_text(fs_stub)
    (temp / "test.cpp").write_text(stubs + handler + cases)
    command = ["clang++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O1", "-g",
               "-fsanitize=undefined", "-fno-sanitize-recover=all",
               "-I", str(temp), "-I", str(PROJECT / "src"), "-I", str(PROJECT / "include"),
               str(temp / "test.cpp"), "-o", str(temp / "test")]
    subprocess.run(command, check=True, timeout=60)
    subprocess.run([str(temp / "test")], check=True, timeout=10)
print("PASS: defaults recovery is an explicit POST route; startup performs no defaults recovery or settings writes")
