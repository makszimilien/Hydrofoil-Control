#pragma once
#include <cstdint>
#include <cstring>
// Actions: 1=center, 2=pause, 3=resume, 1000..2000=direction-aware reference pulse in us.
// Separate packet namespace: old firmware cannot mistake a test for settings.
struct ServoTestPacket { uint32_t magic=0x48465434,version=2,kind=1,session=0,sequence=0,action=0,result=0,actualPulse=0; };
inline bool sameServoTest(const ServoTestPacket&a,const ServoTestPacket&b){return a.session==b.session&&a.sequence==b.sequence&&a.action==b.action;}
inline bool servoTestAckMatches(const ServoTestPacket&a,const ServoTestPacket&b){return a.kind==2&&sameServoTest(a,b);}
inline bool decodeServoTestPacket(const uint8_t*data,int length,ServoTestPacket&out){
 if(!data||length!=sizeof(ServoTestPacket))return false;
 ServoTestPacket p;std::memcpy(&p,data,sizeof(p));
 if(p.magic!=0x48465434||p.version!=2||(p.kind!=1&&p.kind!=2)||!((p.action>=1&&p.action<=3)||(p.action>=1000&&p.action<=2000))||p.result>1||(p.kind==1&&p.result))return false;
 if(p.kind==1&&p.actualPulse)return false;
 if(p.kind==2){
  bool held=!p.result&&(p.action==1||p.action>=1000);
  if(held){if(p.actualPulse<1000||p.actualPulse>2000)return false;
   if(p.action==1&&p.actualPulse!=1500)return false;
   if(p.action>=1000&&p.actualPulse!=p.action&&p.actualPulse!=3000-p.action)return false;
  }else if(p.actualPulse)return false;
 }
 out=p;return true;
}
