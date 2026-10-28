"use strict";
const $=id=>document.getElementById(id);
const fields=['p','i','d','setpoint','factor','enable','calibration','servo-min','servo-max'].map(x=>'slider-'+x);
let sharedWaiting=false,sharedDirty=false,peerAddresses={},peerNames={},connectedLabel='This board';
let boardStates={},loadGeneration=0,calBusy=false,calPrompt='',lastTestState={},overviewStamp='',overviewError='';
let targetButtons={},targetButtonStamp='',groupTravelEdited=false;
const group=()=>activeBoard==='both';
let ready=false,busy=false,activeBoard='master',isSlave=false,dirty=false,awaiting=false;
const status=t=>{$('save-status').textContent=t;};
const freshState=d=>!!(d&&d.fresh&&d.settings_valid);
function feedback(t){$('test-action-status').textContent=t;$('servo-action-status').textContent=t;}
function labelFor(key){return key==='master'?connectedLabel:peerLabel(key);}
function boardKeys(){return ['master',...(!isSlave?Object.keys(peerAddresses):[])];}
function peerLabel(key){const name=peerNames[key]||'';return name?name.slice(-2).toUpperCase()+' · Slave '+key.slice(-1):'Slave '+key.slice(-1)+' · name unavailable';}
function labelPeers(){for(const select of [$('pulse-target')])for(const o of Array.from(select.options)){if(o.value.startsWith('slave-'))o.textContent=peerLabel(o.value);}}

function identity(d){const suffix=(d.ssid||'—').slice(-2).toUpperCase();connectedLabel=suffix+' · '+(d.slave?'Slave':'Master');$('board-id').textContent=suffix;$('board-role').textContent=d.slave?'Slave':'Master';$('board-network').textContent=d.ssid||'';$('recorder-scope').textContent='These tools use '+connectedLabel+' only.';$('connection-state').textContent='Connected';$('control-scope').textContent=d.slave?'This board':'All paired boards';$('pulse-target').options[0].textContent=connectedLabel;}
const remote=()=>!isSlave&&activeBoard.startsWith('slave-');
function groupLabel(){return boardKeys().length>2?'All boards':'Both';}
function renderBoardButtons(){
 const keys=[...boardKeys(),...(!isSlave&&Object.keys(peerAddresses).length?['both']:[])];const stamp=keys.join(',');
 if(stamp!==targetButtonStamp){targetButtonStamp=stamp;targetButtons={};$('board-tabs').textContent='';for(const key of keys){const button=document.createElement('button');button.value=key;button.onclick=()=>selectBoard(key);targetButtons[key]=button;$('board-tabs').appendChild(button);}}
 for(const [key,button]of Object.entries(targetButtons)){button.textContent=key==='both'?groupLabel():labelFor(key).split(' · ')[0];button.setAttribute('aria-label',key==='both'?'Set up or tune '+groupLabel().toLowerCase():'Set up or tune '+labelFor(key));button.setAttribute('aria-pressed',activeBoard===key?'true':'false');button.disabled=busy||awaiting||calBusy;}
}
async function selectBoard(next){if(busy||awaiting||calBusy||next===activeBoard)return;if(dirty&&!confirm('Discard unapplied settings?'))return;awaiting=false;sharedWaiting=false;await loadBoard(next);}
function controls(){const editingBlocked=!ready||busy||awaiting||!!boardStates[activeBoard]?.calibration_enabled;for(const id of fields)$(id).disabled=editingBlocked;for(const id of ['reverse','apply','apply-shared'])$(id).disabled=editingBlocked;$('apply-shared').disabled=$('apply-shared').disabled||Object.values(boardStates).some(d=>d.calibration_enabled||d.calibration_pending);$('board-selector').disabled=busy||awaiting||calBusy;$('shared-travel').disabled=editingBlocked;$('reload-settings').disabled=busy||awaiting||calBusy;for(const button of Object.values(targetButtons))button.disabled=busy||awaiting||calBusy;}
async function request(url,body){const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),2500);const opt={cache:'no-store',signal:abort.signal};if(body){opt.method='POST';opt.headers={'Content-Type':'application/x-www-form-urlencoded'};opt.body=new URLSearchParams(body);}try{const r=await fetch(url,opt);if(!r.ok)throw Error(await r.text());return r;}finally{clearTimeout(timeout);}}

function render(){renderBoardButtons();for(const id of fields){if(id==='slider-calibration'||id==='slider-enable')continue;const el=$(id+'-value');if(el)el.textContent=$(id).value;}
 $('apply').textContent='Apply to '+selectedLabel()+' only';$('apply').hidden=group();$('reverse').hidden=group();$('endpoint-a-label').textContent=group()?'Lower pulse (µs)':'Endpoint A (µs)';$('endpoint-b-label').textContent=group()?'Upper pulse (µs)':'Endpoint B (µs)';
 $('selected-controls').hidden=group();$('run-selected').textContent='Run '+selectedLabel().split(' · ')[0];$('pause-selected').textContent='Pause '+selectedLabel().split(' · ')[0];
 $('draft-state').textContent=boardStates[activeBoard]?.calibration_enabled?'Finish calibration before changing tuning.':dirty?'Unsaved changes':'';
 $('reload-settings').textContent=dirty?'Discard edits & reload saved settings':'Refresh saved settings';
 const a=Number($('slider-servo-min').value),b=Number($('slider-servo-max').value);$('direction-state').textContent=group()?'Select an individual board to reverse its direction.':'Direction: '+(a>b?'reversed':'normal');
 $('travel-title').textContent=group()?'Travel limits · '+groupLabel().toLowerCase():'Travel & direction';$('travel-hint').textContent=(group()?'':'Reverse swaps A and B. Save direction on this board only. ')+(group()?'Travel changes pause the boards. Resume them in Control.':'Travel changes pause this board; press Run afterward.');$('sensor-details-card').hidden=group();
 $('calibration-title').textContent='Calibrate '+selectedLabel();
 $('calibration-card').hidden=group();$('group-calibration-hint').hidden=!group();$('group-calibration-hint').textContent='Calibration is individual. Choose '+boardKeys().map(k=>labelFor(k).split(' · ')[0]).join(' or ')+' above to calibrate that sensor.';
 $('apply-shared').hidden=!group()||isSlave||!Object.keys(peerAddresses).length;$('shared-hint').hidden=$('apply-shared').hidden;$('shared-travel-label').hidden=$('apply-shared').hidden;controls();}

function draft(){sharedWaiting=false;dirty=true;status('');render();}
function selectedLabel(){return group()?groupLabel():remote()?peerLabel(activeBoard):connectedLabel;}
function fillSettings(state){for(const id of fields){$(id).value=String(id==='slider-enable'?1:state.settings[id]);$(id).placeholder='';}}
function showBoardState(){
 if(group()){showGroupState();return;}
 const d=boardStates[activeBoard];const fresh=freshState(d);
 ready=fresh&&!d.calibration_pending;
 $('target-note').textContent=fresh?('Saved settings read from '+selectedLabel()+(remote()?' · '+(d.age_ms/1000).toFixed(1)+' s ago':'')):('Waiting for '+selectedLabel()+' — saved settings unavailable');
 $('selected-status').className='readout';$('selected-status').setAttribute('data-state',boardStatus(d).tone);$('selected-status').textContent=describeBoard(d);
 $('next-step').textContent=nextStep(d);
 $('board-readings').textContent='';$('sensor-details').textContent='';
 if(fresh){
  const readings=[['Sensor level',d.sampling&&d.position_valid?String(Math.round(d.position)):'—'],['Servo output',d.servo_attached?d.servo_us+' µs':'Off'],['Sensor',!d.sampling?'Paused':d.sensor_healthy?'Reading':'Timeout']];
  for(const [name,value]of readings){const p=document.createElement('p'),label=document.createElement('small'),number=document.createElement('strong');label.textContent=name;number.textContent=value;p.appendChild(label);p.appendChild(number);$('board-readings').appendChild(p);}
  $('sensor-details').textContent=(d.sampling?'Latest samples':'Last samples — sensing paused')+'\nRaw: '+d.raw_ticks+' ticks · Median: '+d.median_ticks+' ticks\nGood: '+d.good_samples+' · Timeouts: '+d.timeouts+'\nSaved calibration: '+d.settings.min_measured+'–'+d.settings.max_measured+' ticks\nReceiver: '+(d.receiver_valid?d.receiver_us+' µs':'No fresh signal');
  $('calibration-status').textContent=d.calibration_enabled?(d.calibration_active?'Collecting '+d.calibration_samples+' readings · '+d.minimum+'–'+d.maximum+' ticks · '+(d.range_ready?'Range ready':'Move through depth range'):'Calibration interrupted — Finish, then Start again'):(d.notice|| (d.calibration_valid?'Saved calibration ready':'Calibration needed'));
  if(!dirty&&!busy&&!awaiting)fillSettings(d);
 }else {$('calibration-status').textContent='Waiting for fresh board status';}
 if(d&&d.calibration_status)$('calibration-action-status').textContent=d.calibration_status;
 const prompt=fresh?activeBoard+':'+d.calibration_enabled+':'+d.calibration_valid:'';if(prompt!==calPrompt&&fresh){if(d.calibration_enabled||!d.calibration_valid)$('calibration-card').open=true;calPrompt=prompt;}
 $('cal-start').disabled=!fresh||calBusy||busy||awaiting||!!d.calibration_pending||!!d.calibration_enabled;
 $('cal-stop').disabled=!fresh||calBusy||busy||awaiting||!!d.calibration_pending||!d.calibration_enabled;
 $('run-selected').disabled=!fresh||calBusy||!!d.calibration_pending||!!d.calibration_enabled||!d.calibration_valid;
 $('pause-selected').disabled=busy;
 // Switching targets and requesting readback must still work when a slave is offline.
 render();$('board-selector').disabled=busy||awaiting||calBusy;
}
function groupSettings(){
 const states=boardKeys().map(k=>boardStates[k]);if(!states.every(freshState))return null;
 const result={};for(const id of fields){let values=states.map(d=>d.settings[id]);if(id==='slider-calibration')values=states.map(()=>0);if(id==='slider-enable')values=states.map(()=>1);if(id==='slider-servo-min')values=states.map(d=>Math.min(d.settings['slider-servo-min'],d.settings['slider-servo-max']));if(id==='slider-servo-max')values=states.map(d=>Math.max(d.settings['slider-servo-min'],d.settings['slider-servo-max']));result[id]=values.every(v=>v===values[0])?values[0]:'';}return result;
}
function showGroupState(){
 const keys=boardKeys(),states=keys.map(k=>boardStates[k]),saved=groupSettings();ready=!!saved&&!states.some(d=>d.calibration_enabled||d.calibration_pending);
 $('target-note').textContent=saved?'':'Waiting for fresh saved settings from every board';
 showStatusRows($('selected-status'),keys);
 $('next-step').textContent=!saved?'Keep the master’s Wi-Fi connection. Both boards must reply before shared settings can be applied.':!ready?'Finish each board’s calibration before shared tuning.':Object.values(saved).some(v=>v==='')?'Fields marked “Different” need a shared value before Apply.':'';
 $('board-readings').textContent='';$('sensor-details').textContent='';
 for(const key of keys){const d=boardStates[key],row=document.createElement('p');row.textContent=labelFor(key)+': '+(freshState(d)?'level '+(d.sampling&&d.position_valid?Math.round(d.position):'—')+' · servo '+(d.servo_attached?d.servo_us+' µs':'off'):'Waiting for readings');$('board-readings').appendChild(row);}
 if(saved&&!dirty&&!busy&&!awaiting){for(const id of fields){$(id).value=String(saved[id]);$(id).placeholder=saved[id]===''?'Different':'';}}
 $('cal-start').disabled=$('cal-stop').disabled=true;
 $('run-selected').disabled=!ready||states.some(d=>!d.calibration_valid)||busy||awaiting;$('pause-selected').disabled=busy;
 render();
}
function boardStatus(d){
 if(!d||!d.fresh)return {tone:'fault',text:'OFFLINE / STALE — current state unconfirmed'};
 if(!d.settings_valid)return {tone:'fault',text:'Setup needed · settings unavailable'};
 if(d.error)return {tone:'fault',text:'FAULT · '+d.error};
 if(d.calibration_pending)return {tone:'paused',text:'Calibration command pending'};
 if(d.calibration_enabled)return {tone:'paused',text:d.calibration_active?'Calibrating · servo output off':'Calibration interrupted · Finish, then Start'};
 if(d.mode==='center-test')return {tone:'normal',text:'TEST · holding '+d.servo_us+' µs'};
 if(d.mode==='bench')return {tone:'paused',text:'PAUSED — servo output off'};
 if(!d.servo_attached)return {tone:'fault',text:'Output off · check setup'};
 if(!d.settings['slider-enable'])return {tone:'paused',text:'Legacy control setting · apply tuning to use the sensor'};
 if(!d.sensor_healthy)return {tone:'fault',text:'SENSOR TIMEOUT · last servo command held'};
 if(!d.position_valid)return {tone:'fault',text:'Sensor not ready · last servo command held'};
 return {tone:'normal',text:'RUNNING · water-height control'};
}
function describeBoard(d){return boardStatus(d).text;}
function showStatusRows(container,keys,includePulse=false){
 container.className='readout status-list';container.setAttribute('data-state','normal');container.textContent='';
 for(const key of keys){const d=boardStates[key],state=boardStatus(d),row=document.createElement('div');row.className='board-status-row';row.setAttribute('data-state',state.tone);row.textContent=labelFor(key)+': '+state.text+(includePulse&&freshState(d)&&d.servo_attached&&d.mode!=='center-test'?' · '+d.servo_us+' µs':'');container.appendChild(row);}
}
function nextStep(d){
 if(d&&d.fresh&&!d.settings_valid)return 'This board’s saved settings need recovery. Open its Device setup using its own Wi-Fi.';
 if(!freshState(d))return remote()?'Stay on the master’s Wi-Fi. Check this board’s power and range; both boards need R5.5.':'Waiting for the connected board. Check the Wi-Fi connection.';
 if(d.calibration_pending)return 'Wait for this board to confirm the calibration command.';
 if(d.calibration_enabled)return d.calibration_active?'Move this sensor fully out of the water, down to its intended depth, and back up. Repeat, then Finish & save.':'Press Finish, then Start a new calibration.';
 if(!d.calibration_valid)return 'First, calibrate this sensor. The calibration controls are open below.';
 if(d.mode==='center-test')return 'Test position is held. Press Run to return this board to its saved control settings.';
 if(d.mode==='bench')return 'Press Run when you want this board to respond. Saved calibration is retained.';
 if(d.settings['slider-enable']&&(!d.sensor_healthy||!d.position_valid))return 'The sensor is not supplying a usable level. Check the connection and readings before tuning.';
 if(!d.settings['slider-enable'])return 'Apply tuning to use water-height sensing on this board.';
 return '';
}
function showOverview(){
 const keys=boardKeys(),all=keys.map(k=>boardStates[k]),n=all.filter(d=>d&&d.fresh).length;
 const scope=keys.length>2?'all boards':keys.length===2?'both':'this board';$('resume-test').textContent='Run '+scope;$('stop-test').textContent='Pause '+scope;$('exit-test').textContent='End test & run '+scope;$('pause-test').textContent='End test & pause '+scope;$('center-test').textContent=keys.length===1?'Center THIS servo · 1500 µs':'Center '+scope+' · 1500 µs';$('apply-shared').textContent='Apply to '+(keys.length>2?'all boards':'both boards');$('control-scope').textContent=keys.length===1?'This board':'All paired boards';
 const summary=overviewError||n+' of '+keys.length+' boards connected';if($('mode-status').textContent!==summary)$('mode-status').textContent=summary;const stamp=keys.map(k=>[labelFor(k),describeBoard(boardStates[k]),boardStates[k]?.calibration_enabled,boardStates[k]?.calibration_valid].join('|')).join(';');
 if(stamp!==overviewStamp){overviewStamp=stamp;$('overview-boards').textContent='';
 for(const key of keys){const d=boardStates[key],card=document.createElement('div'),name=document.createElement('strong'),state=document.createElement('p'),cal=document.createElement('p');card.className='board-summary';card.setAttribute('data-state',boardStatus(d).tone);name.textContent=labelFor(key);state.className='status-label';state.textContent=describeBoard(d);cal.textContent=freshState(d)?(d.calibration_enabled?'Calibration in progress':d.calibration_valid?'Calibration saved':'Calibration needed'):'No fresh readings';card.appendChild(name);card.appendChild(state);card.appendChild(cal);$('overview-boards').appendChild(card);}}
 const waiting=keys.filter(k=>!freshState(boardStates[k]));const blocked=keys.filter(k=>freshState(boardStates[k])&&(boardStates[k].calibration_enabled||!boardStates[k].calibration_valid||boardStates[k].calibration_pending));
 const anyTestPending=Object.values(lastTestState).some(t=>t&&t.pending);
 $('center-test').disabled=!!(waiting.length||anyTestPending||all.some(d=>d&&d.calibration_enabled));
 $('resume-test').disabled=!!(waiting.length||blocked.length||anyTestPending||awaiting);$('exit-test').disabled=$('resume-test').disabled;
 $('run-hint').textContent=blocked.length?'Calibrate '+blocked.map(labelFor).join(', ')+' before Run.':'Run uses saved settings. Pause turns servo output off.';
 $('connection-help').textContent=waiting.length?'Waiting for '+waiting.map(labelFor).join(', ')+'. Keep this Wi-Fi connection; check board power and range.':isSlave?'Connect to the master’s Wi-Fi to manage both boards.':'Stay on this Wi-Fi for either board’s calibration, tuning and live readings.';
 showStatusRows($('servo-live-status'),keys,true);
 const target=$('pulse-target').value==='local'?'master':$('pulse-target').value;const td=boardStates[target];
 $('pulse-send').disabled=!freshState(td)||!!td?.calibration_enabled||!!lastTestState[target]?.pending;$('pulse-send').textContent='Hold value on '+labelFor(target).split(' · ')[0];
 if(freshState(td))$('pulse-feedback').textContent='Current '+labelFor(target)+': '+(td.servo_attached?td.servo_us+' µs actual output':'output off');
 else $('pulse-feedback').textContent='Waiting for fresh output status from '+labelFor(target);
}
async function refreshBoardStates(){boardStates=await(await request('/board-state')).json();overviewError='';showBoardState();showOverview();}
async function loadBoard(board){const generation=++loadGeneration;activeBoard=board;$('board-selector').value=board;ready=false;dirty=false;groupTravelEdited=false;sharedWaiting=false;sharedDirty=false;calPrompt='';$('selected-action-status').textContent='';controls();$('calibration-action-status').textContent='';
 try{const states=await(await request('/board-state')).json();if(generation!==loadGeneration)return;boardStates=states;showBoardState();status(ready?'':'Waiting for board readback…');}catch(e){status(e.message);$('board-selector').disabled=false;}}
$('reload-settings').onclick=async()=>{if(dirty&&!confirm('Discard unapplied settings and read the board?'))return;await loadBoard(activeBoard);};
$('board-selector').addEventListener('change',()=>selectBoard($('board-selector').value));
$('inputs').addEventListener('input',e=>{if(fields.includes(e.target.id)){if(group()&&['slider-servo-min','slider-servo-max'].includes(e.target.id))groupTravelEdited=true;draft();}});
$('shared-travel').onchange=()=>{if(group())draft();};
$('reverse').onclick=()=>{if(group())return;const a=$('slider-servo-min').value;$('slider-servo-min').value=$('slider-servo-max').value;$('slider-servo-max').value=a;draft();};
function validateTuning(travel){
 $('slider-enable').value='1'; // Compatibility field; normal operation always uses the sensor.
 const checks=[['p','P',0,4,false],['i','I',0,2,false],['d','D',0,2,false],['setpoint','Set Point',1000,2000,true],['factor','Receiver influence',0,100,true]];
 if(travel)checks.push(['servo-min',group()?'Lower pulse':'Endpoint A',1000,2000,true],['servo-max',group()?'Upper pulse':'Endpoint B',1000,2000,true]);
 for(const [key,label,min,max,whole] of checks){const raw=String($('slider-'+key).value),n=Number(raw);if(!raw.trim()||!Number.isFinite(n)||n<min||n>max||(whole&&!Number.isInteger(n))){if(key.startsWith('servo-'))$('travel-details').open=true;const message=label+': enter '+(whole?'a whole number':'a number')+' from '+min+' to '+max;status(message);$('draft-state').textContent=message;return false;}}
 return true;
}
$('apply').onclick=async()=>{if(group()||!ready||busy||awaiting||!validateTuning(true))return;busy=true;controls();const body={'board-selector':activeBoard};for(const id of fields)body[id]=$(id).value;sharedWaiting=false;
try{await request('/set-sliders',body);dirty=false;awaiting=remote();status(remote()?'Saved on master; waiting for slave confirmation':'Applied on this board');}catch(e){status('Not confirmed: '+e.message);}finally{busy=false;render();}};
$('apply-shared').onclick=async()=>{
 if(!group()||!ready||busy||awaiting||!validateTuning($('shared-travel').checked))return;busy=true;controls();
 const body={'include-travel':$('shared-travel').checked?'1':'0'};for(const key of ['p','i','d','setpoint','factor','enable'])body['slider-'+key]=$('slider-'+key).value;if($('shared-travel').checked)for(const key of ['servo-min','servo-max'])body['slider-'+key]=$('slider-'+key).value;
 try{await request('/shared-tuning',body);sharedWaiting=true;sharedDirty=!$('shared-travel').checked&&groupTravelEdited;awaiting=true;status('Master saved. Waiting for paired boards…');}
 catch(e){status('Shared tuning not confirmed: '+e.message);}finally{busy=false;controls();}
};
async function calibrate(enabled){
 if(group()||!ready||calBusy||busy||awaiting)return;const board=activeBoard;calBusy=true;$('cal-start').disabled=$('cal-stop').disabled=true;
 try{await request(remote()?'/remote-calibration':'/calibration',remote()?{enabled,board}:{enabled});$('calibration-action-status').textContent=remote()?'Sent — waiting for this board to confirm':'Calibration command saved';await refreshBoardStates();}
 catch(e){$('calibration-action-status').textContent='Not confirmed: '+e.message;}
 finally{calBusy=false;showBoardState();}
}
$('cal-start').onclick=()=>calibrate('1');$('cal-stop').onclick=()=>calibrate('0');
async function selectedCommand(action){if(group())return;try{await request('/servo-test',{action,scope:remote()?activeBoard:'local'});$('selected-action-status').textContent='Requested for '+selectedLabel()+' — current state appears above';}catch(e){$('selected-action-status').textContent=e.message;}}
$('run-selected').onclick=()=>selectedCommand('resume');$('pause-selected').onclick=()=>selectedCommand('pause');
async function refreshPeers(){const d=await(await request('/get-addresses')).json();$('slave-device-list').textContent=Object.values(d).filter(Boolean).join(' · ')||'No slaves configured';
for(const [i,v]of Object.values(d).entries()){const key='slave-'+(i+1);if(v)peerAddresses[key]=v;if(v&&!isSlave&&!Array.from($('pulse-target').options).some(o=>o.value===key)){const o=document.createElement('option');o.value=key;o.textContent=peerLabel(key);$('pulse-target').appendChild(o);}}}
$('submit-button').onclick=async()=>{try{await request('/add-mac',{mac:$('mac-input').value});await refreshPeers();status('Peer saved. Select it, review settings, then Apply.');}catch(e){status(e.message);}};
$('remove-slaves-button').onclick=async()=>{if(!confirm('Remove all paired slaves?'))return;try{await request('/remove-slaves',{'remove-slaves':'true'});location.reload();}catch(e){status(e.message);}};
$('wifi-off-button').onclick=async()=>{try{await request('/wifi-off',{'wifi-off':'true'});status('Restart board to restore hotspot');}catch(e){status(e.message);}};
async function poll(){try{const d=await(await request('/device-status')).json();identity(d);
const names=await(await request('/board-identities')).json();for(const [key,entry]of Object.entries(names))peerNames[key]=entry.name||'';labelPeers();
const test=await(await request('/servo-test-status')).json();lastTestState=test;
const lines=[connectedLabel+': '+test.local];let pending=false;
if(!isSlave)for(const key of ['slave-1','slave-2']){const t=test[key];if(t.address){const state=boardStates[key];lines.push(peerLabel(key)+': '+(state&&state.fresh?(state.mode==='bench'?'PAUSED':state.mode==='center-test'?'TEST · '+state.servo_us+' µs':'ACTIVE'):'State unavailable')+'\nLast command: '+t.status);pending=pending||t.pending;}}
$('test-status').textContent=lines.join('\n');
for(const id of ['center-test','resume-test'])$(id).disabled=pending;$('stop-test').disabled=false;
await refreshBoardStates();
if(remote()||sharedWaiting){const peers=await(await request('/peer-status')).json();
 if(sharedWaiting){const entries=Object.keys(peerAddresses).map(k=>[k,peers[k]]);awaiting=entries.some(([,t])=>t&&t.pending);
 $('transfer-status').textContent=entries.map(([k,t])=>peerLabel(k)+': '+(t?t.status:'Unavailable')).join('\n');
 if(!awaiting){const confirmed=entries.every(([,t])=>t&&t.status.startsWith('Confirmed'));if(confirmed)dirty=sharedDirty;status(confirmed?'Shared tuning confirmed on all boards.'+(sharedDirty?' Travel edits are still unsaved. Check Match travel limits too, then Apply again.':''):'Master saved; one or more slaves not confirmed. See results below.');}
 }else{const t=peers[activeBoard];$('transfer-status').textContent=t.status;awaiting=t.pending;if(!dirty&&!busy&&!t.pending&&t.status!=='Not sent')status(t.status);}
 controls();}else $('transfer-status').textContent='';
showBoardState();showOverview();

}catch(e){boardStates={};overviewError='Connection lost — board state unconfirmed';showBoardState();showOverview();$('connection-state').textContent='Disconnected';}setTimeout(poll,1000);}
async function init(){controls();try{const d=await(await request('/device-status')).json();isSlave=d.slave;identity(d);$('pairing-tools').hidden=isSlave;if(isSlave){$('exit-test').textContent='End test & run this board';$('pause-test').textContent='End test & pause this board';$('center-test').textContent='Center THIS servo — 1500 µs';$('stop-test').textContent='Pause this board';$('resume-test').textContent='Run this board';}if(isSlave){$('selector-card').hidden=true;$('submit-button').disabled=true;$('remove-slaves-button').disabled=true;}await refreshPeers();await loadBoard('master');}catch(e){status(e.message);}poll();}init();

async function servoTest(action){
for(const id of ['center-test','stop-test','resume-test'])$(id).disabled=true;
try{await request('/servo-test',{action,scope:isSlave?'local':'all'});feedback('Requested — check each board’s output and confirmation below.');}
catch(e){feedback(e.message);}
finally{for(const id of ['center-test','stop-test','resume-test'])$(id).disabled=false;showOverview();}
}
$('center-test').onclick=()=>servoTest('center');$('stop-test').onclick=()=>servoTest('pause');$('resume-test').onclick=()=>servoTest('resume');$('exit-test').onclick=()=>servoTest('resume');$('pause-test').onclick=()=>servoTest('pause');

function adjustPulse(delta){const n=Number($('pulse-value').value);$('pulse-value').value=String(Math.min(2000,Math.max(1000,(Number.isFinite(n)?n:1500)+delta)));}
$('pulse-target').addEventListener('change',showOverview);
$('pulse-down').onclick=()=>adjustPulse(-10);$('pulse-up').onclick=()=>adjustPulse(10);
$('pulse-send').onclick=async()=>{
 const pulse=Number($('pulse-value').value);
 if(!Number.isInteger(pulse)||pulse<1000||pulse>2000){feedback('Enter a whole number from 1000 to 2000 µs');return;}
 $('pulse-send').disabled=true;
 try{await request('/servo-test',{action:'pulse',pulse:String(pulse),scope:isSlave?'local':$('pulse-target').value});feedback('Requested '+pulse+' µs — saved direction applied. Check actual output above.');}
 catch(e){$('test-action-status').textContent=e.message;}
 finally{$('pulse-send').disabled=false;}
};
