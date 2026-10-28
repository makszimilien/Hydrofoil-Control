#pragma once
#include <cmath>
#include <cstring>
#include <cstdint>
#include "filehandling.h"
inline bool validControlParams(const dataStruct &p) {
 return std::isfinite(p.p)&&std::isfinite(p.i)&&std::isfinite(p.d)&&
 p.p>=0&&p.p<=4&&p.i>=0&&p.i<=2&&p.d>=0&&p.d<=2&&
 p.setpoint>=1000&&p.setpoint<=2000&&p.factor>=0&&p.factor<=100&&
 (p.enable==0||p.enable==1)&&(p.calibration==0||p.calibration==1)&&
 p.servoMin>=1000&&p.servoMin<=2000&&p.servoMax>=1000&&p.servoMax<=2000&&
 p.minMeasured>=0&&p.minMeasured<=80000&&p.maxMeasured>=0&&p.maxMeasured<=80000;
}
inline bool parseMacAddress(const char *s,uint8_t out[6]) {
 if(!s||std::strlen(s)!=17)return false;
 auto hex=[](char c)->int {if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;};
 const char separator=s[2];if(separator!=':'&&separator!='-')return false;
 uint8_t result[6];bool any=false;
 for(int i=0;i<6;i++){int a=hex(s[i*3]),z=hex(s[i*3+1]);if(a<0||z<0||(i<5&&s[i*3+2]!=separator))return false;result[i]=(a<<4)|z;any|=result[i]!=0;}
 if(!any||(result[0]&1))return false;
 std::memcpy(out,result,6);return true;
}
inline bool decodeControlPacket(const uint8_t *bytes,int length,dataStruct &out) {
 if(!bytes||length!=sizeof(dataStruct))return false;
 dataStruct candidate;std::memcpy(&candidate,bytes,sizeof(candidate));
 // A slave owns its measured calibration endpoints. Validate incoming controls
 // independently, then validate the local calibrated range before applying.
 dataStruct checked=candidate;
 if(candidate.calibration!=0&&candidate.calibration!=1)return false;
 checked.calibration=1;
 if(!validControlParams(checked))return false;
 out=candidate;return true;
}
