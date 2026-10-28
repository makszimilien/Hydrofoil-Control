"""Focused R5 controller host regressions using actual firmware functions.

These checks do not establish servo direction, loaded mechanism safety, measured
500 Hz cadence, RF performance, or the electrical state of the physical board.
"""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--only", required=True, choices=("quickpid", "filesystem", "validation", "receiver", "modes", "task"))
parser.add_argument("--project", choices=("controller-r5.5",), default="controller-r5.5")
OPTIONS = parser.parse_args()
SELECTED = OPTIONS.only
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


def run_cpp(name, code, includes=(), extra_sources=(), files=None):
    if SELECTED and SELECTED != name:
        return
    with tempfile.TemporaryDirectory(prefix="hydrofoil-r5-" + name + "-") as temp:
        temp = Path(temp)
        (temp / "test.cpp").write_text(code)
        for filename, content in (files or {}).items():
            (temp / filename).write_text(content)
        command = ["clang++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                   "-O1", "-g", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                   "-DARDUINO=10800", "-DARDUINOJSON_ENABLE_ARDUINO_STREAM=0",
                   "-DARDUINOJSON_ENABLE_ARDUINO_PRINT=0", "-DARDUINOJSON_ENABLE_PROGMEM=0",
                   "-I", str(temp), "-I", str(PROJECT / "include")]
        for directory in includes:
            command += ["-I", str(directory)]
        command += [str(temp / "test.cpp"), *map(str, extra_sources), "-o", str(temp / "test")]
        subprocess.run(command, check=True, timeout=60)
        subprocess.run([str(temp / "test")], check=True, timeout=10)


sensor_stubs = r'''
#include "sensor_wait.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
constexpr int OUTPUT=1, INPUT=0, LOW=0, HIGH=1, capacitancePin=6;
struct Timer {} fakeTimer;
Timer *timer = &fakeTimer;
bool interruptsEnabled=true, stuckLow=false, simulateSensorTimeout=false;
bool benchActive=false,otaInProgress=false,settingsReady=true,first=false;
uint64_t clockTicks=0, riseAt=12000;
unsigned reads=0;
void pinMode(int,int) {}
void digitalWrite(int,int) {}
void delayMicroseconds(int) {}
void noInterrupts() {interruptsEnabled=false;}
void interrupts() {interruptsEnabled=true;}
void timerRestart(Timer*) {clockTicks=0; reads=0;}
uint64_t timerRead(Timer*) {return clockTicks;}
int digitalRead(int) {
  if (++reads > 100000) throw std::runtime_error("unbounded sensor wait");
  clockTicks += 40;
  return (!stuckLow && clockTicks >= riseAt) ? HIGH : LOW;
}
struct {void println(const char*){assert(interruptsEnabled);}} Serial;
std::vector<int> rawValues;
bool sensorHealthy=false, positionValid=false;
uint32_t sensorTimeouts=0, sensorGoodSamples=0, sensorLastTicks=0;
int median=0, position=0, control=0, servoPos=0;
float input=0, setpoint=0, output=0;
struct {int calibration=0, minMeasured=2000, maxMeasured=24000;
        int setpoint=1500, servoMin=1000, servoMax=2000;} controlParams;
struct FakePid {
  enum class Control {manual, timer};
  Control mode=Control::timer;
  int computes=0;
  void SetMode(Control m){mode=m;}
  bool Compute(){++computes; return mode!=Control::manual;}
} elevatorPid;
using QuickPID = FakePid;
struct {int writes=0, last=0; bool attached(){return true;} void writeMicroseconds(int value){++writes;last=value;}} elevator;
long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
'''

sensor_cases = r'''
int main() {
  (void)Serial;
  // An interrupted median window must never feed stale samples into control.
  rawValues={12000}; sensorHealthy=true; positionValid=true; stuckLow=true;
  startMeasurement();
  assert(interruptsEnabled && !sensorHealthy && !positionValid && rawValues.empty());
  assert(sensorTimeouts==1 && sensorLastTicks==kSensorTimeoutTicks);
  calculatePid();
  assert(elevatorPid.computes==0 && elevator.writes==0);
  assert(elevatorPid.mode==FakePid::Control::manual);
  stuckLow=false;
  for(int i=0;i<19;++i) { startMeasurement(); calculatePid(); }
  assert(sensorHealthy && !positionValid && elevator.writes==0);
  startMeasurement(); calculatePid();
  assert(positionValid && median==12000 && sensorGoodSamples==20);
  assert(elevatorPid.computes==1 && elevator.writes==1);
  assert(elevatorPid.mode==FakePid::Control::timer);
  std::cout << "PASS: bounded timeout restores interrupts, holds command, and requires 20 fresh samples to recover\n";

  // Upper-middle median20 must reject a minority of arbitrarily low/high dips.
  rawValues.assign(20,12000);
  for(int i=0;i<9;++i) rawValues[i]=1;
  calculatePosition(); assert(median==12000 && positionValid);
  for(int i=0;i<9;++i) rawValues[i]=79999;
  calculatePosition(); assert(median==12000 && positionValid);
  rawValues.clear();
  for(int i=0;i<20;++i) rawValues.push_back(2000+i*1000);
  calculatePosition(); assert(median==12000);
  riseAt=18000; startMeasurement();
  assert(rawValues.size()==20 && rawValues.front()==3000 && rawValues.back()==18000);
  std::cout << "PASS: original upper-middle median20 and rolling capacity preserved\n";

  rawValues.assign(20,12000);
  controlParams.minMeasured=12000; controlParams.maxMeasured=12000;
  const int previousWrites=elevator.writes;
  calculatePid(); assert(!positionValid && elevator.writes==previousWrites);
  controlParams.minMeasured=24000; controlParams.maxMeasured=2000;
  calculatePid(); assert(!positionValid && elevator.writes==previousWrites);
  controlParams.calibration=1;
  controlParams.minMeasured=25000; controlParams.maxMeasured=0;
  calculatePosition();
  assert(controlParams.minMeasured==12000 && controlParams.maxMeasured==12000 && !positionValid);
  rawValues.assign(20,16000); calculatePosition();
  assert(positionValid && position==2000);
  controlParams.calibration=0;
  std::cout << "PASS: empty/reversed calibration holds output; two distinct calibration values establish position\n";

  // Use the actual command mapping; reversed endpoint order is intentional.
  controlParams.minMeasured=2000; controlParams.maxMeasured=24000;
  rawValues.assign(20,12000); controlParams.setpoint=1500;
  for(int reverse=0;reverse<2;++reverse) {
    controlParams.servoMin=reverse?1900:1100;
    controlParams.servoMax=reverse?1100:1900;
    output=-500; control=-1000; calculatePid();
    assert(elevator.last==controlParams.servoMin && setpoint==1000);
    output=0; control=0; calculatePid();
    assert(elevator.last==1500 && setpoint==1500);
    output=500; control=1000; calculatePid();
    assert(elevator.last==controlParams.servoMax && setpoint==2000);
  }
  std::cout << "PASS: forward/reversed servo endpoints and midpoint; receiver-offset setpoint stays within 1000..2000\n";

  timer=nullptr; positionValid=true;
  startMeasurement();
  assert(!sensorHealthy && !positionValid && rawValues.empty() && interruptsEnabled);
  const int writes=elevator.writes; calculatePid();
  assert(elevator.writes==writes);
  std::cout << "PASS: unavailable timer invalidates sensing and cannot command the servo\n";
}
'''
sensor_code = sensor_stubs + "\n".join(function(SOURCE, name) for name in
    ("getMedian", "startMeasurement", "calculatePosition", "writeServoCommand", "calculatePid")) + sensor_cases
run_cpp("sensor", sensor_code)

# Exercise the pinned real QuickPID library, not a copy of the PID arithmetic.
quickpid_dir = PROJECT / ".pio/libdeps/esp32-c3-devkitm-1/QuickPID/src"
setup = function(SOURCE, "setup")
if SELECTED in (None, "quickpid"):
    assert "SetSampleTimeUs(10000)" in function(SOURCE, "resumeController")
pid_code = r'''
#include "Arduino.h"
#include "QuickPID.h"
#include <cassert>
#include <cmath>
#include <iostream>
uint32_t now=0;
uint32_t micros(){return now;}
void close(float actual,float expected){assert(std::fabs(actual-expected)<0.0001f);}
int main(){
  float input=0,output=0,setpoint=10;
  QuickPID pid(&input,&output,&setpoint,0,1,0,
      QuickPID::pMode::pOnError,QuickPID::dMode::dOnError,
      QuickPID::iAwMode::iAwClamp,QuickPID::Action::direct);
  pid.SetOutputLimits(-500,500);
  pid.SetSampleTimeUs(10000);
  pid.SetMode(QuickPID::Control::timer);
  now+=10000; assert(pid.Compute()); close(output,0.1f);
  now+=10000; assert(pid.Compute()); close(output,0.2f);
  pid.SetMode(QuickPID::Control::manual);
  now+=10000; assert(!pid.Compute()); close(output,0.2f);
  pid.SetMode(QuickPID::Control::timer);
  now+=10000; assert(pid.Compute()); close(output,0.3f);
  float inD=0,outD=0,setD=0;
  QuickPID derivative(&inD,&outD,&setD,0,0,1,
      QuickPID::pMode::pOnError,QuickPID::dMode::dOnError,
      QuickPID::iAwMode::iAwClamp,QuickPID::Action::direct);
  derivative.SetOutputLimits(-500,500);
  derivative.SetSampleTimeUs(10000);
  derivative.SetMode(QuickPID::Control::timer);
  derivative.Compute();
  setD=1; now+=10000; derivative.Compute(); close(outD,100);
  now+=10000; derivative.Compute(); close(outD,0);
  setD=100; now+=10000; derivative.Compute(); close(outD,500);
  std::cout << "PASS: real QuickPID uses 10ms I/D scaling, output clamps, and manual hold/recovery\n";
}
'''
arduino_stub = """#pragma once
#include <cstdint>
uint32_t micros();
template<class T> T constrain(T value,T low,T high) { return value<low?low:(value>high?high:value); }
"""
run_cpp("quickpid", pid_code, includes=(quickpid_dir,),
        extra_sources=(quickpid_dir / "QuickPID.cpp",), files={"Arduino.h": arduino_stub})

# Full persistence implementation is linked against the pinned real ArduinoJson
# library and an in-memory filesystem that can fail writes and renames.
filesystem_arduino_stub = r'''
#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
using std::size_t;
class String {
 public:
  std::string value;
  String() = default;
  String(const char* text): value(text?text:"") {}
  String(const std::string& text): value(text) {}
  const char* c_str() const {return value.c_str();}
  size_t length() const {return value.length();}
  bool isEmpty() const {return value.empty();}
  bool reserve(size_t size) {value.reserve(size); return true;}
  bool concat(const char* text) {value+=text; return true;}
  bool concat(const char* text,size_t size) {value.append(text,size); return true;}
  String& operator+=(const char* text) {value+=text; return *this;}
  String& operator+=(const String& text) {value+=text.value; return *this;}
  char operator[](size_t index) const {return value[index];}
  friend String operator+(const String& a,const char* b){return a.value+b;}
  friend String operator+(const String& a,const String& b){return a.value+b.value;}
  friend bool operator==(const String& a,const String& b){return a.value==b.value;}
  friend bool operator!=(const String& a,const String& b){return !(a==b);}
};
struct FakeSerial {
 template<class T> void println(const T&) {}
 template<class T> void print(const T&) {}
 template<class... A> void printf(const char*,A...) {}
};
extern FakeSerial Serial;
'''
filesystem_fs_stub = r'''
#pragma once
#include "Arduino.h"
#include <map>
#define FILE_WRITE "w"
#define FILE_READ "r"
namespace fs {class FS;}
class File {
 public:
  fs::FS* owner=nullptr;
  std::string* data=nullptr;
  size_t cursor=0;
  bool writable=false, opened=false;
  File()=default;
  File(fs::FS* fs,std::string* content,bool writing): owner(fs),data(content),writable(writing),opened(true) {}
  explicit operator bool() const {return opened;}
  bool isDirectory() const {return false;}
  size_t size() const {return opened?data->size():0;}
  size_t readBytes(char* destination,size_t size) {
    if(!opened)return 0;
    const size_t count=std::min(size,data->size()-cursor);
    memcpy(destination,data->data()+cursor,count); cursor+=count; return count;
  }
  size_t write(const uint8_t*,size_t);
  size_t write(uint8_t byte){return write(&byte,1);}
  void flush() {}
  void close(){opened=false;}
};
namespace fs {
class FS {
 public:
  std::map<std::string,std::string> files;
  bool failWrite=false,failRename=false,failOpen=false;
  unsigned failRenameAt=0;
  unsigned writeOpens=0,renameCalls=0;
  bool begin(bool formatOnFail){lastFormat=formatOnFail;return mountSucceeds;}
  bool lastFormat=true,mountSucceeds=true;
  File open(const char* path,const char* mode="r"){
    if(failOpen)return {};
    const bool writing=strcmp(mode,FILE_WRITE)==0;
    if(writing){++writeOpens;files[path].clear();}
    const auto it=files.find(path);
    return it==files.end()?File():File(this,&it->second,writing);
  }
  bool exists(const char* path) const{return files.count(path)!=0;}
  bool remove(const char* path){return files.erase(path)!=0;}
  bool rename(const char* from,const char* to){
    ++renameCalls;
    if(failRename || (failRenameAt && renameCalls==failRenameAt)){failRename=false;return false;}
    const auto it=files.find(from);if(it==files.end())return false;
    files[to]=it->second;files.erase(it);return true;
  }
};
}
inline size_t File::write(const uint8_t* bytes,size_t size){
  if(!opened||!writable)return 0;
  const size_t count=owner->failWrite?size/2:size;
  data->append(reinterpret_cast<const char*>(bytes),count);return count;
}
extern fs::FS SPIFFS;
'''
filesystem_code = r'''
#include "filehandling.h"
#include <ArduinoJson.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
FakeSerial Serial;
fs::FS SPIFFS;
dataStruct values(int variant){
  dataStruct data={};
  data.p=1.0f+variant*0.25f;data.i=variant*0.125f;data.d=variant*0.0625f;
  data.setpoint=1400+variant*100;data.enable=variant%2;
  data.servoMin=1100+variant*50;data.servoMax=1900-variant*50;
  data.factor=30+variant*10;data.calibration=0;
  data.minMeasured=2000+variant*100;data.maxMeasured=24000+variant*100;
  return data;
}
void same(const allBoards& a,const allBoards& b){assert(memcmp(&a,&b,sizeof(a))==0);}
std::string serialize(const JsonDocument& doc){std::string out;serializeJson(doc,out);return out;}
int main(){
  assert(initFS() && !SPIFFS.lastFormat);
  SPIFFS.mountSucceeds=false; assert(!initFS() && !SPIFFS.lastFormat);
  SPIFFS.mountSucceeds=true;
  allBoards source={values(0),values(1),values(2)},loaded={};
  assert(writeStructJson(SPIFFS,"/configs.json",source));
  const std::string original=SPIFFS.files["/configs.json"];
  assert(readStructJson(SPIFFS,"/configs.json",loaded));same(source,loaded);
  DynamicJsonDocument doc(8192);
  assert(!deserializeJson(doc,original));
  assert(doc["master"].size()==11 && doc["slave1"].size()==11 && doc["slave2"].size()==11);
  std::cout << "PASS: no format-on-mount; all 33 distinct settings round-trip through actual parser/writer\n";

  // Missing objects/fields, wrong types, malformed/truncated JSON are atomic rejects.
  for(int bad=0;bad<5;++bad){
    doc.clear();assert(!deserializeJson(doc,original));
    if(bad==0)doc["slave2"].remove("maxMeasured");
    if(bad==1)doc.remove("slave1");
    if(bad==2)doc["master"]["p"]="1.0";
    if(bad==3)doc["master"]["servoMin"]=1100.5;
    SPIFFS.files["/configs.json"]=bad==4?original.substr(0,original.size()/2):serialize(doc);
    loaded=source;
    assert(!readStructJson(SPIFFS,"/configs.json",loaded));same(source,loaded);
  }
  SPIFFS.files["/configs.json"]=original;
  auto changed=source;changed.master.setpoint=1600;
  SPIFFS.failWrite=true;
  assert(!writeStructJson(SPIFFS,"/configs.json",changed));
  assert(SPIFFS.files["/configs.json"]==original);
  SPIFFS.failWrite=false;SPIFFS.failRename=true;
  assert(!writeStructJson(SPIFFS,"/configs.json",changed));
  assert(SPIFFS.files["/configs.json"]==original);
  SPIFFS.failRenameAt=SPIFFS.renameCalls+2;
  assert(!writeStructJson(SPIFFS,"/configs.json",changed));
  assert(SPIFFS.files["/configs.json"]==original);
  SPIFFS.failRenameAt=0;
  assert(readStructJson(SPIFFS,"/configs.json",loaded));same(source,loaded);
  SPIFFS.files["/configs.json.bak"]=original;
  SPIFFS.files.erase("/configs.json");loaded={};
  assert(readStructJson(SPIFFS,"/configs.json",loaded));same(source,loaded);
  SPIFFS.files["/configs.json"]="malformed-primary";
  assert(!readStructJson(SPIFFS,"/configs.json",loaded));same(source,loaded);
  SPIFFS.files["/configs.json"]=original;
  std::cout << "PASS: malformed/missing/type-invalid settings and failed writes/renames preserve previous values\n";
  std::cout << "PASS: backup can recover missing primary; malformed primary never silently falls back\n";

  String addresses[2]={"CC:BA:97:47:BE:28",""};
  assert(writeArrayJson(SPIFFS,"/addresses.json","addresses",addresses,2));
  String readback[2]={"unchanged0","unchanged1"};
  assert(readArrayJson(SPIFFS,"/addresses.json","addresses",readback,2));
  assert(readback[0]==addresses[0] && readback[1]==addresses[1]);
  for(const char* bad:{"{\"addresses\":[\"CC:BA:97:47:BE:28\",\"\",\"\"]}",
                       "{\"addresses\":[\"not-a-mac\",\"\"]}",
                       "{\"addresses\":[\"CC:BA:97:47:BE:28\",\"cc-ba-97-47-be-28\"]}",
                       "{\"addresses\":[123,\"\"]}"}){
    SPIFFS.files["/addresses.json"]=bad;
    readback[0]="unchanged0";readback[1]="unchanged1";
    assert(!readArrayJson(SPIFFS,"/addresses.json","addresses",readback,2));
    assert(readback[0]=="unchanged0" && readback[1]=="unchanged1");
  }
  std::cout << "PASS: peer-address read is bounded and rejects malformed values without partially changing targets\n";

  SPIFFS.files["/wifi.json"]="{\"first\":\"False\",\"slave\":\"False\",\"enable\":\"True\",\"upload\":\"False\",\"ssid\":\"test\",\"password\":\"test-value\"}";
  assert(writeFileJson(SPIFFS,"/wifi.json","slave","True"));
  assert(readFileJson(SPIFFS,"/wifi.json","slave")=="True");
  assert(readFileJson(SPIFFS,"/wifi.json","ssid")=="test");
  assert(readFileJson(SPIFFS,"/wifi.json","password")=="test-value");
  const std::string wifi=SPIFFS.files["/wifi.json"];
  SPIFFS.failWrite=true;
  assert(!writeFileJson(SPIFFS,"/wifi.json","slave","False"));
  assert(SPIFFS.files["/wifi.json"]==wifi);
  std::cout << "PASS: Wi-Fi single-property update preserves unrelated fields and rejects failed writes\n";
}
'''
json_dir = PROJECT / ".pio/libdeps/esp32-c3-devkitm-1/ArduinoJson/src"
run_cpp("filesystem", filesystem_code, includes=(PROJECT / "src", json_dir),
        extra_sources=(PROJECT / "src/filehandling.cpp",),
        files={"Arduino.h": filesystem_arduino_stub, "SPIFFS.h": filesystem_fs_stub})

validation_code = r'''
#include "controller_validation.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
dataStruct validParams(){return {1.25f,0.25f,0.125f,1500,1,1000,2000,40,0,2000,24000};}
int main(){
  auto data=validParams();assert(validControlParams(data));
  data.servoMin=2000;data.servoMax=1000;assert(validControlParams(data));
  for(int bad=0;bad<15;++bad){
    data=validParams();
    switch(bad){
      case 0:data.p=std::numeric_limits<float>::quiet_NaN();break;
      case 1:data.i=std::numeric_limits<float>::infinity();break;
      case 2:data.d=-0.01f;break;
      case 3:data.p=4.001f;break;
      case 4:data.i=2.001f;break;
      case 5:data.d=2.001f;break;
      case 6:data.setpoint=999;break;
      case 7:data.setpoint=2001;break;
      case 8:data.enable=2;break;
      case 9:data.calibration=-1;break;
      case 10:data.servoMin=999;break;
      case 11:data.servoMax=2001;break;
      case 12:data.factor=101;break;
      case 13:data.minMeasured=-1;break;
      case 14:data.maxMeasured=80001;break;
    }
    assert(!validControlParams(data));
  }
  data=validParams();data.calibration=1;data.minMeasured=25000;data.maxMeasured=0;
  assert(validControlParams(data));
  // R5.2 stores incomplete ranges so OFF can always exit calibration; resume checks usability.
  data.calibration=0;assert(validControlParams(data));
  data.minMeasured=12000;data.maxMeasured=12000;assert(validControlParams(data));
  std::cout << "PASS: control validation rejects nonfinite/range-invalid values and permits reversed servo endpoints\n";
  uint8_t destination[6]={1,2,3,4,5,6};
  const uint8_t expected[6]={0xCC,0xBA,0x97,0x47,0xBE,0x28};
  assert(parseMacAddress("CC:BA:97:47:BE:28",destination));
  assert(memcmp(destination,expected,6)==0);
  assert(parseMacAddress("cc-ba-97-47-be-28",destination));
  assert(memcmp(destination,expected,6)==0);
  for(const char* bad:{"", "CC:BA:97:47:BE", "CC:BA:97:47:BE:28:00", "CC:BA:97:47:BE:2Z",
                       "CC:BA-97:47:BE:28", "CD:BA:97:47:BE:28", "FF:FF:FF:FF:FF:FF", "00:00:00:00:00:00"}){
    assert(!parseMacAddress(bad,destination));assert(memcmp(destination,expected,6)==0);
  }
  assert(!parseMacAddress(nullptr,destination));
  std::cout << "PASS: MAC parser accepts consistent colon/hyphen unicast addresses; invalid input cannot partly modify destination\n";
  auto incoming=validParams(),out=validParams();out.setpoint=1800;const auto previous=out;
  uint8_t tiny[1]={0};
  assert(!decodeControlPacket(tiny,1,out));assert(memcmp(&out,&previous,sizeof(out))==0);
  assert(!decodeControlPacket(nullptr,sizeof(dataStruct),out));
  assert(!decodeControlPacket(reinterpret_cast<const uint8_t*>(&incoming),-1,out));
  assert(!decodeControlPacket(reinterpret_cast<const uint8_t*>(&incoming),sizeof(incoming)+1,out));
  incoming.p=std::numeric_limits<float>::quiet_NaN();
  assert(!decodeControlPacket(reinterpret_cast<const uint8_t*>(&incoming),sizeof(incoming),out));
  assert(memcmp(&out,&previous,sizeof(out))==0);
  incoming=validParams();
  assert(decodeControlPacket(reinterpret_cast<const uint8_t*>(&incoming),sizeof(incoming),out));
  assert(memcmp(&out,&incoming,sizeof(out))==0);
  std::cout << "PASS: short/null/oversized/nonfinite ESP-NOW frames rejected before mutation or out-of-bounds read; valid 44-byte layout preserved\n";
  static_assert(sizeof(dataStruct)==44,"ESP-NOW compatibility layout changed");
}
'''
run_cpp("validation", validation_code, includes=(PROJECT / "src",),
        files={"Arduino.h": filesystem_arduino_stub, "SPIFFS.h": filesystem_fs_stub})

receiver_stubs = r'''
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>
struct rmt_item32_t {uint32_t duration0,level0,duration1,level1;};
std::vector<std::vector<rmt_item32_t>> blocks;
unsigned blockIndex=0,returned=0;
void* rb=reinterpret_cast<void*>(1);
uint32_t now=0,receiverLastValid=0,receiverClockHz=2000000;
constexpr uint32_t kReceiverTimeoutMs=250;
bool receiverValid=false;
int pwmRead=0,control=0;
struct {int factor=40;} controlParams;
uint32_t millis(){return now;}
void* xRingbufferReceive(void*,size_t* bytes,int){
 if(blockIndex==blocks.size())return nullptr;
 auto& block=blocks[blockIndex++];*bytes=block.size()*sizeof(rmt_item32_t);return block.data();
}
void vRingbufferReturnItem(void*,void*){++returned;}
void pulses(std::initializer_list<rmt_item32_t> values){blocks={values};blockIndex=0;}
'''
receiver_cases = r'''
int main(){
  now=10;pulses({{3000,1,10000,0}});getPulseDuration();
  assert(receiverValid && pwmRead==1500 && control==0 && returned==1);
  now=20;pulses({{1000,0,4000,1}});getPulseDuration();
  assert(receiverValid && pwmRead==2000 && control==200 && returned==2);
  now=30;pulses({{2000,1,10000,0},{3000,1,10000,0},{3800,1,10000,0}});getPulseDuration();
  assert(pwmRead==1900 && control==160 && returned==3);
  now=40;pulses({{400,1,10000,0},{10000,0,8000,1}});getPulseDuration();
  assert(pwmRead==1900 && control==160 && receiverLastValid==30);
  blocks.clear();blockIndex=0;
  now=280;getPulseDuration();assert(receiverValid && control==160);
  now=281;getPulseDuration();assert(!receiverValid && control==0);
  std::cout << "PASS: both RMT pulse polarities and latest valid pulse; invalid/no receiver becomes neutral after 250ms\n";
  receiverClockHz=4000000;now=300;pulses({{6000,1,10000,0}});getPulseDuration();
  assert(receiverValid && pwmRead==1500 && control==0);
  receiverClockHz=2000000;
  now=UINT32_MAX-100;pulses({{2000,1,10000,0}});getPulseDuration();
  assert(receiverValid && control==-200);
  blocks.clear();blockIndex=0;now=100;getPulseDuration();assert(receiverValid);
  now=151;getPulseDuration();assert(!receiverValid && control==0);
  receiverClockHz=0;pulses({{2000,1,10000,0}});getPulseDuration();
  assert(!receiverValid && control==0);
  rb=nullptr;control=123;getPulseDuration();assert(control==0);
  std::cout << "PASS: measured RMT clock conversion, millis rollover, missing ring buffer and zero-clock protection\n";
}
'''
run_cpp("receiver", receiver_stubs + function(SOURCE,"getPulseDuration") + receiver_cases)

modes_stubs = r'''
#include "filehandling.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>
constexpr int servoPin=1;
struct ControlLock {ControlLock(){} ~ControlLock(){}};
void* controlTaskHandle=reinterpret_cast<void*>(1);
void* rb=reinterpret_cast<void*>(1);
unsigned pendingRmt=2,returnedRmt=0;
int stalePulse=0;
void* xRingbufferReceive(void*,size_t* bytes,int){if(!pendingRmt)return nullptr;--pendingRmt;*bytes=sizeof(stalePulse);return &stalePulse;}
void vRingbufferReturnItem(void*,void*){++returnedRmt;}
bool servoCenterTest=false;bool settingsReady=true,first=false,otaInProgress=false,benchActive=true;
bool sensorHealthy=true,positionValid=true,receiverValid=true;
int control=180,servoPos=0;
float input=1400,output=125,setpoint=1600;
int timerValue=0;int* timer=&timerValue;
bool servoTimeOut=true;
String controllerError;
dataStruct controlParams={1,0,0,1500,1,1100,1900,40,0,2000,24000};
unsigned stops=0;
std::vector<int> rawValues(20,10000);
void stopBenchSamples(){++stops;sensorHealthy=false;positionValid=false;rawValues.clear();}
struct FakePid {
 enum class Control{manual,timer};
 Control mode=Control::timer;
 uint32_t sampleUs=0;float min=0,max=0;
 void SetMode(Control next){mode=next;}
 void SetSampleTimeUs(uint32_t value){sampleUs=value;}
 void SetTunings(float,float,float){}
 void SetOutputLimits(float low,float high){min=low;max=high;}
} elevatorPid;
using QuickPID=FakePid;
struct FakeServo {
 bool connected=false,failAttach=false;
 unsigned writes=0,attaches=0,detaches=0;
 int value=0,low=0,high=0;
 void attach(int pin,int minimum,int maximum){assert(pin==servoPin);++attaches;low=minimum;high=maximum;connected=!failAttach;}
 bool attached(){return connected;}
 void detach(){connected=false;++detaches;}
 void writeMicroseconds(int pulse){assert(connected);++writes;value=pulse;}
} elevator;
'''
modes_cases = r'''
int main(){
  assert(resumeController());
  assert(!benchActive && elevator.attached() && !receiverValid && control==0);
  assert(!pendingRmt && returnedRmt==2);
  assert(!sensorHealthy && !positionValid && rawValues.empty());
  assert(elevator.value==1500 && output==0 && input==1500);
  assert(elevator.low==1000 && elevator.high==2000);
  assert(elevatorPid.sampleUs==10000 && elevatorPid.min==-500 && elevatorPid.max==500);
  assert(elevatorPid.mode==QuickPID::Control::timer);
  writeServoCommand(500);assert(elevator.value==1100);
  writeServoCommand(2500);assert(elevator.value==1900);
  controlParams.servoMin=1900;controlParams.servoMax=1100;
  writeServoCommand(500);assert(elevator.value==1100);
  writeServoCommand(2500);assert(elevator.value==1900);
  const unsigned writes=elevator.writes;
  enterBenchMode();
  assert(benchActive && !elevator.attached() && !servoTimeOut);
  assert(elevatorPid.mode==QuickPID::Control::manual);
  writeServoCommand(1500);assert(elevator.writes==writes);
  const unsigned stopped=stops;enterBenchMode();assert(stops==stopped);
  std::cout << "PASS: explicit resume clears stale samples/receiver, centers within limits, and bench entry detaches servo\n";

  for(int failure=0;failure<5;++failure){
    settingsReady=failure!=0;first=failure==1;otaInProgress=failure==2;timer=failure==3?nullptr:&timerValue;
    controlTaskHandle=failure==4?nullptr:reinterpret_cast<void*>(1);
    assert(!resumeController());assert(!elevator.attached() && benchActive);
    writeServoCommand(1500);assert(elevator.writes==writes);
  }
  settingsReady=true;first=false;otaInProgress=false;timer=&timerValue;
  controlTaskHandle=reinterpret_cast<void*>(1);
  controlParams.calibration=1;assert(!resumeController()&&benchActive&&!elevator.attached());
  controlParams.calibration=0;controlParams.maxMeasured=controlParams.minMeasured;
  assert(!resumeController()&&benchActive&&!elevator.attached());
  controlParams.maxMeasured=24000;
  elevator.failAttach=true;assert(!resumeController());
  assert(benchActive && !elevator.attached() && !controllerError.isEmpty());
  elevator.failAttach=false;assert(resumeController());
  const unsigned resumedWrites=elevator.writes;
  otaInProgress=true;writeServoCommand(1600);assert(elevator.writes==resumedWrites);
  otaInProgress=false;first=true;writeServoCommand(1600);assert(elevator.writes==resumedWrites);
  first=false;settingsReady=false;writeServoCommand(1600);assert(elevator.writes==resumedWrites);
  settingsReady=true;elevator.detach();writeServoCommand(1600);assert(elevator.writes==resumedWrites);
  std::cout << "PASS: OTA/setup/missing settings/timer/attach failures cannot enable output; all servo-write gates hold\n";
}
'''
run_cpp("modes", modes_stubs + '\n#include "calibration_session.h"\n' + "\n".join(function(SOURCE,n) for n in
        ("enterBenchMode","writeServoCommand","resumeController")) + modes_cases,
        includes=(PROJECT / "src",),
        files={"Arduino.h": filesystem_arduino_stub, "SPIFFS.h": filesystem_fs_stub})

settings_stubs = r'''
#include "controller_validation.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
fs::FS SPIFFS;
struct ControlLock {ControlLock(){} ~ControlLock(){}};
bool settingsReady=true,slave=false,saveSucceeds=true;
unsigned saves=0,applies=0,sends=0,lastSent=99;
const char* jsonConfigsPath="/configs.json";
dataStruct validParams(){return {1.25f,0,0,1500,1,1000,2000,40,0,2000,24000};}
dataStruct controlParams=validParams();
allBoards boardsParams={validParams(),validParams(),validParams()};
String boardSelector="master";
struct FakeServer {
 std::map<std::string,String> args;int code=0;
 bool hasArg(const char* name){return args.count(name);}
 String arg(const char* name){auto i=args.find(name);return i==args.end()?String():i->second;}
 void send(int value,const char*,const char*){code=value;}
} benchServer;
bool writeStructJson(fs::FS&,const char*,allBoards&){++saves;return saveSucceeds;}
void applyLocalParams(const dataStruct& next){++applies;controlParams=next;}
void sendPeer(unsigned index){++sends;lastSent=index;}
void goodRequest(){
 benchServer.args={{"board-selector","master"},{"slider-p","1.25"},{"slider-i","0"},{"slider-d","0"},
  {"slider-setpoint","1500"},{"slider-factor","40"},{"slider-enable","1"},{"slider-calibration","0"},
  {"slider-servo-min","1000"},{"slider-servo-max","2000"}};
 benchServer.code=0;
}
'''
settings_cases = r'''
int main(){
  for(const char* missing:{"slider-p","slider-i","slider-d","slider-setpoint","slider-factor", "slider-enable", "slider-calibration","slider-servo-min","slider-servo-max","board-selector"}){
    goodRequest();benchServer.args.erase(missing);const auto previous=boardsParams;
    setSliders();assert(benchServer.code==400 && saves==0 && applies==0 && sends==0);
    assert(memcmp(&previous,&boardsParams,sizeof(previous))==0);
  }
  for(const char* bad:{"nan","inf","1.5junk","","5"}){
    goodRequest();benchServer.args["slider-p"]=bad;setSliders();
    assert(benchServer.code==400 && saves==0 && applies==0 && sends==0);
  }
  goodRequest();benchServer.args["slider-setpoint"]="1500.25";setSliders();assert(benchServer.code==400 && saves==0);
  goodRequest();benchServer.args["board-selector"]="unknown";setSliders();assert(benchServer.code==400 && saves==0);
  settingsReady=false;goodRequest();setSliders();assert(benchServer.code==503 && saves==0);settingsReady=true;
  std::cout << "PASS: missing/nonfinite/trailing/fractional/out-of-range slider data cannot save, move servo, or send peers\n";
  goodRequest();saveSucceeds=false;benchServer.args["slider-setpoint"]="1700";const auto previous=boardsParams;
  setSliders();assert(benchServer.code==500 && saves==1 && applies==0 && sends==0);
  assert(memcmp(&previous,&boardsParams,sizeof(previous))==0 && controlParams.setpoint==1500);
  saveSucceeds=true;setSliders();assert(benchServer.code==200 && applies==1 && controlParams.setpoint==1700);
  goodRequest();benchServer.args["board-selector"]="slave-2";benchServer.args["slider-servo-min"]="1900";benchServer.args["slider-servo-max"]="1100";
  setSliders();assert(benchServer.code==200 && applies==1 && sends==1 && lastSent==1);
  assert(boardsParams.slave2.servoMin==1900 && boardsParams.slave2.servoMax==1100);
  std::cout << "PASS: slider changes apply only after successful persistence, including reversed remote servo endpoints\n";

  goodRequest();benchServer.args["slider-calibration"]="1";setSliders();
  assert(benchServer.code==200 && controlParams.calibration==1);
  assert(controlParams.minMeasured==25000 && controlParams.maxMeasured==0);
  // Runtime learning changes controlParams, while saved boardsParams still has sentinel.
  controlParams.minMeasured=4500;controlParams.maxMeasured=12500;
  goodRequest();setSliders();assert(benchServer.code==200);
  assert(!controlParams.calibration && controlParams.minMeasured==4500 && controlParams.maxMeasured==12500);
  assert(boardsParams.master.minMeasured==4500 && boardsParams.master.maxMeasured==12500);
  std::cout << "PASS: ending calibration validates and preserves newly measured live endpoints\n";
  slave=true;goodRequest();benchServer.args["board-selector"]="master";benchServer.args["slider-setpoint"]="1300";
  const auto oldMaster=boardsParams.master;const unsigned oldSends=sends;setSliders();
  assert(benchServer.code==200 && controlParams.setpoint==1300 && boardsParams.slave1.setpoint==1300);
  assert(memcmp(&oldMaster,&boardsParams.master,sizeof(oldMaster))==0 && sends==oldSends);
  std::cout << "PASS: slave-local settings remain stored in slave1 regardless of UI selector\n";
}
'''
run_cpp("settings", settings_stubs + "\n".join(function(SOURCE,n) for n in
        ("parseNumber","parseInteger","setSliders")) + settings_cases,
        includes=(PROJECT / "src",),
        files={"Arduino.h": filesystem_arduino_stub, "SPIFFS.h": filesystem_fs_stub})

packet_stubs = r'''
#include <deque>
std::deque<dataStruct> packetQueue;
void* receivedConfigs=reinterpret_cast<void*>(1);
constexpr int pdTRUE=1;
uint32_t rejectedPackets=0,receivedPackets=0;
String controllerError;
int xQueueSend(void*,const dataStruct* data,int){if(packetQueue.size()>=4)return 0;packetQueue.push_back(*data);return pdTRUE;}
int xQueueReceive(void*,dataStruct* data,int){if(packetQueue.empty())return 0;*data=packetQueue.front();packetQueue.pop_front();return pdTRUE;}
void deliver(dataStruct data){onDataRecv(nullptr,reinterpret_cast<const uint8_t*>(&data),sizeof(data));}
'''
# deliver is declared after the firmware callback so no production code is replaced.
packet_stubs=packet_stubs.replace("void deliver(dataStruct data){onDataRecv(nullptr,reinterpret_cast<const uint8_t*>(&data),sizeof(data));}", "")
packet_cases = r'''
void deliver(dataStruct data){onDataRecv(nullptr,reinterpret_cast<const uint8_t*>(&data),sizeof(data));}
int main(){
  uint8_t shortPacket[1]={0};
  onDataRecv(nullptr,shortPacket,1);assert(rejectedPackets==1 && packetQueue.empty());
  auto next=validParams();next.setpoint=1600;
  deliver(next);assert(packetQueue.size()==1 && saves==0 && applies==0);
  slave=false;serviceReceivedConfigs();assert(saves==0 && applies==0 && packetQueue.empty());
  slave=true;deliver(next);serviceReceivedConfigs();
  assert(saves==1 && applies==1 && controlParams.setpoint==1600 && receivedPackets==1);
  deliver(next);serviceReceivedConfigs();assert(saves==1 && applies==1);
  saveSucceeds=false;next.setpoint=1700;deliver(next);serviceReceivedConfigs();
  assert(saves==2 && applies==1 && controlParams.setpoint==1600 && !controllerError.isEmpty());
  saveSucceeds=true;
  std::cout << "PASS: ESP-NOW callback queues only; main-loop slave apply preserves settings on failed save and ignores unchanged packets\n";

  next=validParams();next.calibration=1;deliver(next);serviceReceivedConfigs();
  assert(controlParams.calibration==1 && controlParams.minMeasured==25000 && controlParams.maxMeasured==0);
  controlParams.minMeasured=4800;controlParams.maxMeasured=13200;
  next.calibration=0;deliver(next);serviceReceivedConfigs();
  assert(controlParams.calibration==0 && controlParams.minMeasured==4800 && controlParams.maxMeasured==13200);
  assert(boardsParams.slave1.minMeasured==4800 && boardsParams.slave1.maxMeasured==13200);
  std::cout << "PASS: slave calibration start/stop keeps the slave's live measured range, not the sender's\n";
  const auto rejected=rejectedPackets;
  for(int i=0;i<5;++i)deliver(next);
  assert(packetQueue.size()==4 && rejectedPackets==rejected+1);
  receivedConfigs=nullptr;deliver(next);assert(rejectedPackets==rejected+2);
  std::cout << "PASS: full/unavailable packet queue rejects safely without direct servo or filesystem calls\n";
}
'''
run_cpp("packets", settings_stubs + packet_stubs + "\n".join(function(SOURCE,n) for n in
        ("onDataRecv","serviceReceivedConfigs")) + packet_cases,
        includes=(PROJECT / "src",),
        files={"Arduino.h": filesystem_arduino_stub, "SPIFFS.h": filesystem_fs_stub})

task_stubs = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
int lockDepth=0;
struct ControlLock {ControlLock(){++lockDepth;} ~ControlLock(){--lockDepth;}};
bool benchActive=false,otaInProgress=false,settingsReady=true,first=false;
bool servoTimeOut=false,positionValid=true;
uint32_t now=0,servoTimeOutStart=0;
constexpr uint32_t servoTimeOutInterval=2000;
constexpr int ledPin=8,LOW=0,HIGH=1;
float input=1500,setpoint=1500;
uint32_t millis(){return now;}
struct Ticker {unsigned calls=0;void update(){assert(lockDepth==1);++calls;}} rmtTicker,measurementTicker,pidTicker;
void digitalWrite(int pin,int){assert(pin==ledPin && lockDepth==1);}
struct StopIteration {};
void vTaskDelay(int ticks){assert(ticks==1 && lockDepth==0);throw StopIteration();}
'''
task_cases = r'''
void iteration(){try{runControllerTask(nullptr);}catch(const StopIteration&){}assert(lockDepth==0);}
int main(){
 iteration();assert(rmtTicker.calls==1 && measurementTicker.calls==1 && pidTicker.calls==1);
 for(int blocked=0;blocked<4;++blocked){
  benchActive=blocked==0;otaInProgress=blocked==1;settingsReady=blocked!=2;first=blocked==3;
  iteration();assert(rmtTicker.calls==1 && measurementTicker.calls==1 && pidTicker.calls==1);
 }
 benchActive=false;otaInProgress=false;settingsReady=true;first=false;
 servoTimeOut=true;servoTimeOutStart=100;now=2099;
 iteration();assert(servoTimeOut && rmtTicker.calls==2 && measurementTicker.calls==2 && pidTicker.calls==1);
 now=2100;iteration();assert(!servoTimeOut && pidTicker.calls==2);
 servoTimeOut=true;servoTimeOutStart=UINT32_MAX-999;now=999;
 iteration();assert(servoTimeOut && pidTicker.calls==2);
 now=1000;iteration();assert(!servoTimeOut && pidTicker.calls==3);
 std::cout << "PASS: independent control task obeys bench/OTA/settings/setup gates and endpoint preview timing across clock rollover\n";
 std::cout << "PASS: control operations share a lock and task yields after releasing it (host execution, not timing proof)\n";
}
'''
run_cpp("task", task_stubs + function(SOURCE,"runControllerTask") + task_cases)

if SELECTED is None:
    bench_source=(PROJECT / "src/bench_mode.cpp").read_text()
    # Complements executable interlock checks with reachability guards at wiring points.
    assert "resumeController(" not in function(bench_source,"serviceBenchSamples")
    assert "enterBenchMode(" in bench_source
    loop=function(SOURCE,"loop")
    assert "if(!otaInProgress)" in loop
    assert "if(benchActive)serviceBenchSamples();" in loop
    assert "ArduinoOTA.handle();" in loop
    assert "setupBenchRoutes();" in setup
    assert 'xTaskCreate(runControllerTask,"foil-control",4096,nullptr,2,' in setup
    assert "rmtTicker.update()" not in loop and "measurementTicker.update()" not in loop
    assert "writeStructJson(" not in setup and "writeFileJson(" not in setup
    print("PASS: recording completion cannot resume controller; OTA and bench/controller loop interlocks; boot does not write settings")

if SELECTED == "modes":
    assert 'xTaskCreate(runControllerTask' in setup
    assert setup.index('xTaskCreate(runControllerTask') < setup.index('if(!resumeController()')
    assert 'resumeController(' not in function(SOURCE,'loop')
    assert 'sendPeer(' not in setup
    print('PASS boot starts through validated resume path after scheduler creation; no loop auto-resume or settings push')
