from pathlib import Path
import ast,re,tempfile,subprocess,argparse
parser=argparse.ArgumentParser();parser.add_argument('--project',choices=('controller-r5.2','controller-r5.2p'),default='controller-r5.2');options=parser.parse_args()
ROOT=Path(__file__).resolve().parents[1];P=ROOT/options.project;SOURCE=(P/'src/main.cpp').read_text()
tree=ast.parse((ROOT/'tests/run_controller_r5_tests.py').read_text())
def fixture(name):
 for x in tree.body:
  if isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in x.targets):return ast.literal_eval(x.value)
 raise KeyError(name)
def function(s,n):
 m=re.search(r'^[\w:*& <>]+\b'+n+r'\([^;]*?\)\s*\{',s,re.M);start=s.index('{',m.start());end=start+1;depth=1
 while depth:depth+=(s[end]=='{')-(s[end]=='}');end+=1
 return s[m.start():end]
def run(name,code):
 with tempfile.TemporaryDirectory() as t:
  p=Path(t);(p/'Arduino.h').write_text(fixture('filesystem_arduino_stub'));(p/'SPIFFS.h').write_text(fixture('filesystem_fs_stub'));(p/'test.cpp').write_text(code)
  subprocess.run(['clang++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I',str(p),'-I',str(P/'include'),'-I',str(P/'src'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
  subprocess.run([str(p/'test')],check=True,timeout=15)
 print('PASS:',name)
stubs=fixture('settings_stubs').replace('void applyLocalParams(const dataStruct& next){++applies;controlParams=next;}','')
stubs+='''
#include "calibration_session.h"
#include <vector>
CalibrationSession calibrationSession;String calibrationNotice;
std::vector<int> rawValues;
bool benchActive=true,servoTimeOut=false;unsigned servoTimeOutStart=0,moves=0;
unsigned millis(){return 123;}
void enterBenchMode(){benchActive=true;}
void writeServoCommand(int){++moves;}
struct FakePid{enum class Control{manual,timer};void SetTunings(float,float,float){}void SetMode(Control){}} elevatorPid;
using QuickPID=FakePid;
'''
code=stubs+'\n'.join(function(SOURCE,n) for n in ['applyLocalParams','parseNumber','parseInteger','setSliders'])+r'''
void toggle(int value){goodRequest();benchServer.args["slider-calibration"]=value?"1":"0";setSliders();}
int main(){
 toggle(1);assert(benchServer.code==200&&calibrationSession.active);
 assert(controlParams.minMeasured==2000&&controlParams.maxMeasured==24000);
 calibrationSession.observe(900);calibrationSession.observe(5000);calibrationSession.observe(1200);
 saveSucceeds=false;toggle(0);assert(benchServer.code==500&&controlParams.calibration==1&&calibrationSession.active);
 assert(boardsParams.master.minMeasured==2000);saveSucceeds=true;toggle(0);
 assert(benchServer.code==200&&!controlParams.calibration&&!calibrationSession.active);
 assert(controlParams.minMeasured==900&&controlParams.maxMeasured==5000&&moves==0&&benchActive);
 toggle(1);toggle(0);assert(benchServer.code==200&&controlParams.minMeasured==900&&controlParams.maxMeasured==5000);
 toggle(1);calibrationSession.observe(1234);calibrationSession.observe(1234);toggle(0);
 assert(benchServer.code==200&&controlParams.minMeasured==900&&controlParams.maxMeasured==5000);
 // Exact B4 legacy state: saved calibration ON with uncollected endpoints.
 controlParams.calibration=1;controlParams.minMeasured=25000;controlParams.maxMeasured=0;
 boardsParams.master=controlParams;calibrationSession.finish();toggle(0);
 assert(benchServer.code==200&&controlParams.calibration==0&&benchActive&&moves==0);
 assert(controlParams.minMeasured==25000&&controlParams.maxMeasured==0);
 toggle(1);calibrationSession.observe(1000);calibrationSession.observe(8000);toggle(0);
 assert(controlParams.minMeasured==1000&&controlParams.maxMeasured==8000&&benchActive&&moves==0);
 // Invalid other input still rejected; failed start preserves saved calibration.
 toggle(1);goodRequest();benchServer.args["slider-p"]="nan";setSliders();assert(benchServer.code==400&&controlParams.calibration==1);
 toggle(0);saveSucceeds=false;toggle(1);assert(benchServer.code==500&&!controlParams.calibration&&!calibrationSession.active);
}
'''
run('actual slider/apply: start, range commit, empty/equal cancellation, failed persistence, legacy B4 OFF, no servo movement',code)
# Actual sensor path feeds filtered measurements, not raw noise or timeout samples.
sensor=fixture('sensor_stubs').replace('uint64_t timerRead(Timer*) {return clockTicks;}','uint64_t timerRead(Timer*) {if(simulateSensorTimeout)clockTicks+=40;return clockTicks;}')+'\n#include "calibration_session.h"\nCalibrationSession calibrationSession;\n'
sensor+='\n'.join(function(SOURCE,n) for n in ['getMedian','startMeasurement','calculatePosition'])+r'''
int main(){(void)Serial;(void)elevator;calibrationSession.begin();for(int i=0;i<19;i++)startMeasurement();assert(calibrationSession.samples==0);
 startMeasurement();assert(calibrationSession.samples==1);for(int i=0;i<20;i++){riseAt=20000;startMeasurement();}
 assert(calibrationSession.usable());const auto samples=calibrationSession.samples;
 stuckLow=true;startMeasurement();assert(calibrationSession.samples==samples&&rawValues.empty());
 stuckLow=false;for(int i=0;i<19;i++)startMeasurement();assert(calibrationSession.samples==samples);
 startMeasurement();assert(calibrationSession.samples==samples+1);
 simulateSensorTimeout=true;startMeasurement();assert(calibrationSession.samples==samples+1);
}
'''
run('actual acquisition: median warmup, valid range, timeout exclusion and fresh warmup',sensor)
bench=(P/'src/bench_mode.cpp').read_text()
code=r'''
#include "bench_recording.h"
#include <cassert>
#include <vector>
#include <cstdint>
namespace br=bench_recording;
br::Recorder benchRecording;bool benchAutomatic=false,benchSingle=false,active=true;
uint32_t benchLastSample=0,sensorGoodSamples=0,sensorTimeouts=0,sensorLastTicks=1000;
int now=0;const char* benchRecordingError="";
uint32_t millis(){return now;}bool calibrationSampling(){return active;}
void startMeasurement(){++sensorGoodSamples;}
void stopBenchSamples(br::FinishReason=br::FinishReason::Stopped){benchAutomatic=false;}
'''+function(bench,'serviceBenchSamples')+r'''
int main(){now=10;serviceBenchSamples();assert(sensorGoodSamples==1);now=19;serviceBenchSamples();assert(sensorGoodSamples==1);
 now=20;serviceBenchSamples();assert(sensorGoodSamples==2);active=false;now=30;serviceBenchSamples();assert(sensorGoodSamples==2);}
'''
run('actual bench service: calibration samples with recorder OFF; stops when calibration ends',code)
# Firmware gates are exercised in independent actual-function mode harness below.
assert 'calibrationSession.begin()' not in function(SOURCE,'setup')
assert 'resumeController(' not in function(SOURCE,'setSliders')
assert 'resumeController(' not in function(SOURCE,'applyLocalParams')
print('PASS: interrupted startup does not invent a new range; calibration changes never resume output')

modes=fixture('modes_stubs')+'\n#include "calibration_session.h"\n'
cases=fixture('modes_cases').replace('  assert(resumeController());','  controlParams.calibration=1;assert(!resumeController()&&!elevator.attached());controlParams.calibration=0;\n  controlParams.minMeasured=25000;controlParams.maxMeasured=0;assert(!resumeController()&&!elevator.attached());\n  controlParams.minMeasured=2000;controlParams.maxMeasured=24000;assert(resumeController());',1)
run('actual resume: invalid/active calibration remains detached; existing servo/mode guards',modes+'\n'.join(function(SOURCE,n) for n in ['enterBenchMode','writeServoCommand','resumeController'])+cases)
packets=fixture('packet_stubs').replace('void deliver(dataStruct data){onDataRecv(nullptr,reinterpret_cast<const uint8_t*>(&data),sizeof(data));}','')
code=stubs+packets+'\n'.join(function(SOURCE,n) for n in ['applyLocalParams','onDataRecv','serviceReceivedConfigs'])+r'''
void deliver(dataStruct p){onDataRecv(nullptr,reinterpret_cast<const uint8_t*>(&p),sizeof(p));serviceReceivedConfigs();}
int main(){slave=true;auto p=validParams();p.calibration=1;deliver(p);assert(calibrationSession.active);
 calibrationSession.observe(1000);calibrationSession.observe(9000);p.calibration=0;p.minMeasured=2;p.maxMeasured=3;
 saveSucceeds=false;deliver(p);assert(controlParams.calibration&&calibrationSession.active);
 saveSucceeds=true;deliver(p);assert(!controlParams.calibration&&controlParams.minMeasured==1000&&controlParams.maxMeasured==9000);
 assert(boardsParams.slave1.minMeasured==1000&&moves==0&&benchActive);
 p.calibration=1;deliver(p);p.calibration=0;deliver(p);assert(controlParams.minMeasured==1000&&controlParams.maxMeasured==9000);
}
'''
run('actual peer handler: local calibration retained, failed save retry, empty cancellation, no output resume',code)

if options.project=='controller-r5.2p':
 assert 'resumeController(' not in function(SOURCE,'setup')
 assert 'benchActive=true;' in function(SOURCE,'setup')
 print('PASS: R5.2P startup leaves control paused and never calls resumeController')
