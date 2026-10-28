#pragma once
// Runs in the main loop; radio callbacks only validate and queue packets.
void captureSetupState(SetupPacket &p) {
 ControlLock lock;
 p.settings=slave?boardsParams.slave1:boardsParams.master;
 p.flags=(settingsReady?SetupReady:0)|(sensorHealthy?SetupHealthy:0)|(positionValid?SetupPosition:0)|
  (elevator.attached()?SetupAttached:0)|(calibrationSession.active?SetupCalActive:0)|
  (controlParams.calibration?SetupCalEnabled:0)|(usableCalibrationRange(controlParams.minMeasured,controlParams.maxMeasured)?SetupCalValid:0)|
  ((!benchActive||calibrationSampling())?SetupSampling:0)|(calibrationSession.usable()?SetupRangeReady:0)|(receiverValid?SetupReceiver:0);
 p.mode=servoCenterTest?2:benchActive?1:0;p.raw=sensorLastTicks;p.median=median;p.position=position;p.pulse=servoPos;p.receiver=pwmRead;
 p.good=sensorGoodSamples;p.timeouts=sensorTimeouts;p.samples=calibrationSession.samples;
 p.minimum=calibrationSession.active?calibrationSession.minimum:controlParams.minMeasured;
 p.maximum=calibrationSession.active?calibrationSession.maximum:controlParams.maxMeasured;
 snprintf(p.notice,sizeof(p.notice),"%s",calibrationNotice.c_str());snprintf(p.error,sizeof(p.error),"%s",controllerError.c_str());
}
const char* calibrationResult(uint32_t result,uint32_t action){
 if(result==1)return "Refused: board is not ready";
 if(result==2)return "Save failed: previous calibration state retained";
 if(result==3)return "Refused: older calibration command";
 if(result==4)return "Finished: no usable new range; previous calibration retained";
 if(result==5)return "Finished: no usable range; calibrate again before Run";
 return action==1?"Confirmed: calibration started; output paused":"Confirmed: calibration finished and saved";
}
void transmitSetup(unsigned i){
 auto &t=setupTransfers[i];uint8_t mac[6];if(!parseMacAddress(macAddresses[i].c_str(),mac))return;
 addPeer(macAddresses[i]);esp_now_send(mac,reinterpret_cast<uint8_t*>(&t.request),sizeof(t.request));t.sentAt=millis();++t.attempts;
}
bool beginSetupRequest(unsigned i,uint32_t action){
 auto &t=setupTransfers[i];uint8_t mac[6];if(slave||!espNowReady||!receivedSetup||!parseMacAddress(macAddresses[i].c_str(),mac))return false;
 // A calibration action can supersede an in-flight read, never another action.
 if(t.pending&&(action==0||t.request.action!=0))return false;
 t.request=SetupPacket{};t.request.session=setupSession;t.request.sequence=++setupSequence;t.request.action=action;
 t.pending=true;t.attempts=0;if(action){t.available=false;t.calibrationStatus="Waiting for board confirmation…";}
 transmitSetup(i);return true;
}
void answerSetupRequest(ReceivedSetup &rx){
 auto &p=rx.packet;uint32_t result=0;
 if(p.action){
  const bool sameSender=haveLastCalibration&&!std::memcmp(lastCalibrationSender,rx.sender,6);
  const bool sameSession=sameSender&&lastCalibrationRequest.session==p.session;
  const bool duplicate=sameSession&&lastCalibrationRequest.sequence==p.sequence&&lastCalibrationRequest.action==p.action;
  const bool stale=sameSession&&int32_t(p.sequence-lastCalibrationRequest.sequence)<=0;
  if(duplicate)result=lastCalibrationResult;
  else if(stale)result=3;
  else {result=changeCalibration(p.action==1);lastCalibrationRequest=p;std::memcpy(lastCalibrationSender,rx.sender,6);lastCalibrationResult=result;haveLastCalibration=true;}
 }
 SetupPacket reply=p;reply.kind=2;reply.result=result;captureSetupState(reply);
 char addr[18];snprintf(addr,sizeof(addr),"%02X:%02X:%02X:%02X:%02X:%02X",rx.sender[0],rx.sender[1],rx.sender[2],rx.sender[3],rx.sender[4],rx.sender[5]);
 addPeer(String(addr));esp_now_send(rx.sender,reinterpret_cast<uint8_t*>(&reply),sizeof(reply));
}
void serviceRemoteSetup(){
 ReceivedSetup rx;
 while(receivedSetup&&xQueueReceive(receivedSetup,&rx,0)==pdTRUE){
  if(slave){if(rx.packet.kind==1)answerSetupRequest(rx);continue;}
  for(unsigned i=0;i<2;++i){auto &t=setupTransfers[i];uint8_t mac[6];
   if(t.pending&&parseMacAddress(macAddresses[i].c_str(),mac)&&!std::memcmp(mac,rx.sender,6)&&setupMatches(rx.packet,t.request)){
    t.snapshot=rx.packet;t.receivedAt=millis();t.available=true;t.pending=false;
    if(t.request.action)t.calibrationStatus=calibrationResult(rx.packet.result,t.request.action);
   }
  }
 }
 for(unsigned i=0;i<2;++i){auto &t=setupTransfers[i];if(t.pending&&uint32_t(millis()-t.sentAt)>=750){
  if(t.attempts>=3){t.pending=false;t.available=false;if(t.request.action)t.calibrationStatus="Unconfirmed: no reply. Check live state before retrying.";}
  else transmitSetup(i);
 }}
}
void setupStateJson(JsonObject o,const SetupPacket&p){
 o["mode"]=p.mode==2?"center-test":p.mode==1?"bench":"controller";
 o["settings_valid"]=bool(p.flags&SetupReady);o["sensor_healthy"]=bool(p.flags&SetupHealthy);o["position_valid"]=bool(p.flags&SetupPosition);
 o["sampling"]=bool(p.flags&SetupSampling);o["servo_attached"]=bool(p.flags&SetupAttached);o["receiver_valid"]=bool(p.flags&SetupReceiver);
 o["raw_ticks"]=p.raw;o["median_ticks"]=p.median;o["position"]=p.position;o["servo_us"]=p.pulse;o["receiver_us"]=p.receiver;
 o["good_samples"]=p.good;o["timeouts"]=p.timeouts;o["calibration_active"]=bool(p.flags&SetupCalActive);o["calibration_enabled"]=bool(p.flags&SetupCalEnabled);
 o["calibration_valid"]=bool(p.flags&SetupCalValid);o["range_ready"]=bool(p.flags&SetupRangeReady);o["calibration_samples"]=p.samples;
 o["minimum"]=p.minimum;o["maximum"]=p.maximum;o["notice"]=p.notice;o["error"]=p.error;
 auto v=o.createNestedObject("settings");auto &c=p.settings;
 v["slider-p"]=c.p;v["slider-i"]=c.i;v["slider-d"]=c.d;v["slider-setpoint"]=c.setpoint;v["slider-factor"]=c.factor;v["slider-enable"]=c.enable;
 v["slider-calibration"]=c.calibration;v["slider-servo-min"]=c.servoMin;v["slider-servo-max"]=c.servoMax;v["min_measured"]=c.minMeasured;v["max_measured"]=c.maxMeasured;
}
void sendBoardStates(){
 DynamicJsonDocument d(6144);SetupPacket local;captureSetupState(local);auto o=d.createNestedObject("master");
 o["fresh"]=true;o["available"]=true;o["age_ms"]=0;o["calibration_pending"]=false;setupStateJson(o,local);
 if(!slave)for(unsigned i=0;i<2;++i){uint8_t mac[6];if(!parseMacAddress(macAddresses[i].c_str(),mac))continue;
  auto &t=setupTransfers[i];if(!transfers[i].pending&&!t.pending&&(!t.attempts||uint32_t(millis()-t.sentAt)>=1000))beginSetupRequest(i,0);
  auto peer=d.createNestedObject(i?"slave-2":"slave-1");peer["available"]=t.available;peer["age_ms"]=t.available?uint32_t(millis()-t.receivedAt):0;
  peer["fresh"]=t.available&&uint32_t(millis()-t.receivedAt)<3500;peer["pending"]=t.pending;
  peer["calibration_pending"]=t.pending&&t.request.action!=0;peer["calibration_status"]=t.calibrationStatus;
  if(t.available)setupStateJson(peer,t.snapshot);
 }
 sendJson(d);
}
void requestRemoteCalibration(){
 String board=benchServer.arg("board"),value=benchServer.arg("enabled");int i=board=="slave-1"?0:board=="slave-2"?1:-1;
 if(i<0||slave||(value!="0"&&value!="1")){benchServer.send(400,"text/plain","Choose a paired board and calibration on/off");return;}
 if(otaInProgress||transfers[i].pending||testTransfers[i].pending){benchServer.send(409,"text/plain","Wait for the other board command to finish");return;}
 if(!beginSetupRequest(i,value=="1"?1:2)){benchServer.send(409,"text/plain","Board unavailable or calibration command still pending");return;}
 benchServer.send(202,"text/plain","Sent; waiting for the board to save and confirm");
}
