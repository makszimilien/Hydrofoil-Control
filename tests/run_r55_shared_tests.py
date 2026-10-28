from pathlib import Path
# Reuse the tested host fixtures, then exercise the new actual firmware functions.
exec((Path(__file__).with_name('run_r55_tests.py')).read_text())
shared=base+'\n'.join(fn(n) for n in ['transmitPeer','sendPeer','serviceReceivedConfigs','parseNumber','parseInteger','copySharedTravel','copySharedTuning','setSharedTuning'])+r'''
int main(){
 slave=false;controlParams=boardsParams.master;controlParams.servoMin=1800;controlParams.servoMax=1200;boardsParams.master=controlParams;
 boardsParams.slave1.servoMin=1900;boardsParams.slave1.servoMax=1100; // intentionally stale direction in master copy
 goodRequest();benchServer.args["include-travel"]="1";benchServer.args["slider-servo-min"]="1250";benchServer.args["slider-servo-max"]="1750";
 saveSucceeds=false;auto before=controlParams;setSharedTuning();assert(benchServer.code==500&&wire.empty()&&controlParams.servoMin==before.servoMin);
 saveSucceeds=true;setSharedTuning();assert(benchServer.code==200&&wire.size()==1&&wire.back().kind==5);
 assert(controlParams.servoMin==1750&&controlParams.servoMax==1250);
 auto request=wire.back();auto sends=wire.size();setSharedTuning();assert(benchServer.code==409&&wire.size()==sends);
 slave=true;controlParams.servoMin=1201;controlParams.servoMax=1796;controlParams.minMeasured=1871;controlParams.maxMeasured=13559;
 ReceivedPeer rx{};parseMacAddress("CC:BA:97:47:BE:28",rx.sender);rx.packet=request;queue.push_back(rx);serviceReceivedConfigs();
 assert(controlParams.servoMin==1250&&controlParams.servoMax==1750);assert(controlParams.minMeasured==1871&&controlParams.maxMeasured==13559);
 assert(wire.back().kind==6&&ackMatches(wire.back(),request));auto ack=wire.back();ack.kind=2;assert(!ackMatches(ack,request));
 auto saved=saves;queue.push_back(rx);serviceReceivedConfigs();assert(saves==saved);
 // Tuning-only requests leave travel exactly alone; reversed receivers stay reversed on shared travel.
 rx.packet.sequence++;rx.packet.kind=3;rx.packet.params.servoMin=1000;rx.packet.params.servoMax=2000;queue.push_back(rx);serviceReceivedConfigs();assert(controlParams.servoMin==1250&&controlParams.servoMax==1750&&wire.back().kind==4);
 controlParams.servoMin=1800;controlParams.servoMax=1200;rx.packet.sequence++;rx.packet.kind=5;queue.push_back(rx);serviceReceivedConfigs();assert(controlParams.servoMin==2000&&controlParams.servoMax==1000);
 // Invalid shared travel and local calibration cannot commit or transmit.
 slave=false;transfers[0].pending=false;goodRequest();benchServer.args["include-travel"]="1";benchServer.args["slider-servo-min"]="1500";benchServer.args["slider-servo-max"]="1500";
 saved=saves;setSharedTuning();assert(benchServer.code==400&&saves==saved);controlParams.calibration=1;setSharedTuning();assert(benchServer.code==409&&saves==saved);
 // Shared forms and direct radio settings cannot reactivate the legacy bypass.
 controlParams.calibration=0;goodRequest();benchServer.args["include-travel"]="0";benchServer.args["slider-enable"]="0";setSharedTuning();assert(benchServer.code==200&&controlParams.enable==1&&wire.back().params.enable==1);
 slave=true;rx.packet.sequence+=20;rx.packet.params.enable=0;queue.push_back(rx);serviceReceivedConfigs();assert(wire.back().result==0&&controlParams.enable==1&&boardsParams.slave1.enable==1);
}
'''
run('shared travel preserves actual recipient direction and calibration, tuning-only isolation, failed storage, pending transfer, ACK kind and invalid range',shared)

identity=base[:base.index('int esp_now_send')]+r"""
struct IdentityPacket {uint32_t magic=0x48464935,kind=1,nonce=0;char name[33]={};};
struct ReceivedIdentity {uint8_t sender[6];IdentityPacket packet;};
String peerNames[2],benchHostname="HF-Custom-Z7";uint32_t identityNonce[2]={},identitySentAt[2]={};unsigned identityAttempts[2]={};bool identityPending[2]={};
int receivedIdentities=1;std::deque<ReceivedIdentity> iq;std::vector<IdentityPacket> iw;
int xQueueReceive(int,ReceivedIdentity*p,int){if(iq.empty())return 0;*p=iq.front();iq.pop_front();return 1;}
int esp_now_send(const uint8_t*,const uint8_t*p,size_t n){assert(n==sizeof(IdentityPacket));IdentityPacket copy;std::memcpy(&copy,p,n);iw.push_back(copy);return 0;}
"""+'\n'.join(fn(n) for n in ['transmitIdentity','requestIdentities','serviceIdentities'])+r"""
int main(){
 requestIdentities();assert(iw.size()==1&&identityPending[0]);requestIdentities();assert(iw.size()==1);
 ReceivedIdentity rx{};parseMacAddress("DC:06:75:32:AB:B4",rx.sender);rx.packet=iw[0];rx.packet.kind=2;snprintf(rx.packet.name,sizeof(rx.packet.name),"HF-Changed-Q9");
 rx.packet.nonce++;iq.push_back(rx);serviceIdentities();assert(identityPending[0]&&peerNames[0].length()==0);
 rx.packet.nonce--;rx.sender[5]++;iq.push_back(rx);serviceIdentities();assert(identityPending[0]);
 rx.sender[5]--;iq.push_back(rx);serviceIdentities();assert(!identityPending[0]&&peerNames[0]=="HF-Changed-Q9");
 now=30000;requestIdentities();assert(identityPending[0]);now=31000;serviceIdentities();now=32000;serviceIdentities();now=33000;serviceIdentities();assert(!identityPending[0]&&peerNames[0].length()==0&&iw.size()==4);
 slave=true;rx.packet.kind=1;rx.packet.nonce=987;iq.push_back(rx);serviceIdentities();assert(iw.back().kind==2&&iw.back().nonce==987&&std::string(iw.back().name)=="HF-Custom-Z7");assert(saves==0&&applies==0);
}
"""
run('dynamic peer Wi-Fi names, nonce/sender validation, bounded retry and no settings writes',identity)
