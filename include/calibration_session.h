#pragma once
// Candidate endpoints stay separate from the last saved calibration until OFF.
struct CalibrationSession {
 bool active=false;
 unsigned samples=0;
 int minimum=80000, maximum=0;
 void begin(){active=true;samples=0;minimum=80000;maximum=0;}
 void observe(int value){if(!active||value<0||value>80000)return;
  if(value<minimum)minimum=value;if(value>maximum)maximum=value;++samples;}
 bool usable() const{return samples>=2&&maximum>minimum;}
 void finish(){active=false;}
};
inline bool usableCalibrationRange(int minimum,int maximum){
 return minimum>=0&&maximum<=80000&&maximum>minimum;
}
