#pragma once
#include "controller_validation.h"
constexpr uint32_t kPeerMagic=0x48463331;
struct PeerPacket { uint32_t magic=kPeerMagic,version=1,kind=1,session=0,sequence=0,result=0;dataStruct params{}; };
static_assert(sizeof(PeerPacket)<=250,"ESP-NOW packet too large");
inline bool sameRequest(const PeerPacket&a,const PeerPacket&b){return a.session==b.session&&a.sequence==b.sequence&&!std::memcmp(&a.params,&b.params,sizeof(dataStruct));}
inline bool ackMatches(const PeerPacket&a,const PeerPacket&b){return a.kind==(b.kind==5?6:b.kind==3?4:2)&&sameRequest(a,b);}
inline bool decodePeerPacket(const uint8_t*bytes,int length,PeerPacket&out){
 if(!bytes||length!=sizeof(PeerPacket))return false;
 PeerPacket p;std::memcpy(&p,bytes,sizeof(p));
 if(p.magic!=kPeerMagic||p.version!=1||(p.kind<1||p.kind>6)||p.result>2||!validControlParams(p.params))return false;
 if(p.params.calibration||p.params.minMeasured||p.params.maxMeasured)return false;
 out=p;return true;
}
