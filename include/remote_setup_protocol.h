#pragma once
#include "controller_validation.h"
// Independent request/reply protocol. Fits the ESP-NOW v1 250-byte payload.
struct SetupPacket {
 uint32_t magic=0x48465335,version=1,kind=1,session=0,sequence=0,action=0,result=0;
 dataStruct settings{};
 uint32_t flags=0,mode=0,raw=0,good=0,timeouts=0,samples=0;
 int32_t median=0,position=0,pulse=1500,receiver=0,minimum=0,maximum=0;
 char notice[64]={},error[48]={};
};
static_assert(sizeof(SetupPacket)<=250,"Setup packet exceeds ESP-NOW payload");
enum SetupFlags:uint32_t { SetupReady=1,SetupHealthy=2,SetupPosition=4,SetupAttached=8,
 SetupCalActive=16,SetupCalEnabled=32,SetupCalValid=64,SetupSampling=128,SetupRangeReady=256,SetupReceiver=512 };
inline bool setupMatches(const SetupPacket&a,const SetupPacket&b){return a.kind==2&&a.session==b.session&&a.sequence==b.sequence&&a.action==b.action;}
inline bool decodeSetupPacket(const uint8_t*bytes,int n,SetupPacket&out){
 if(!bytes||n!=sizeof(SetupPacket))return false;SetupPacket p;std::memcpy(&p,bytes,n);
 if(p.magic!=0x48465335||p.version!=1||(p.kind!=1&&p.kind!=2)||p.action>2||p.result>5||p.mode>2||p.flags>1023||p.notice[63]||p.error[47])return false;
 if(p.kind==2&&(p.flags&SetupReady)&&!validControlParams(p.settings))return false;
 out=p;return true;
}
