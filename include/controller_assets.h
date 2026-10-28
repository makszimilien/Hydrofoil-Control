#pragma once
#include <Arduino.h>
static const char kControllerIndex[] PROGMEM = R"R5ASSET(<!doctype html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Hydrofoil R5.5</title><link rel="stylesheet" href="/style.css?v=r55"></head><body>
<header><div class="brand">HYDROFOIL <span>R5.5</span></div><div class="identity"><strong id="board-id">—</strong><div><b id="board-role">Connecting</b><small id="board-network">Checking board…</small></div><span id="connection-state" class="chip">Connecting</span></div><nav><a href="#control">Control</a><a href="#tune">Tune</a><a href="#test">Test</a></nav></header>
<main><section class="card" id="control"><div class="section-title"><h1>Control</h1><span id="control-scope" class="chip">Connected board</span></div><p id="mode-status" role="status">Connecting…</p>
<div class="two"><button id="resume-test" class="primary">Run both</button><button id="stop-test">Pause both</button></div>
<p class="hint" id="run-hint">Run uses saved settings. Pause turns servo output off.</p><div id="overview-boards" class="overview"></div><p id="connection-help" class="hint"></p><p id="test-action-status" class="feedback" role="status"></p>
</section>
<section class="card" id="tune"><h2>Set up & tune</h2><div id="selector-card"><div id="board-tabs" class="board-tabs" role="group" aria-label="Boards to set up or tune"></div><input type="hidden" id="board-selector" value="master"></div><p id="target-note" class="hint" role="status"></p>
<div class="readout" id="selected-status" role="status">Reading board…</div>
<div class="metrics" id="board-readings"></div><p id="next-step" class="next-step"></p>
<details id="sensor-details-card"><summary>More sensor readings</summary><div id="sensor-details" class="readout"></div></details>
<div id="selected-controls"><div class="two"><button id="run-selected">Run selected board</button><button id="pause-selected">Pause selected board</button></div><p id="selected-action-status" class="feedback" role="status"></p></div>
<p id="group-calibration-hint" class="hint" hidden></p><details id="calibration-card"><summary id="calibration-title">Calibrate selected board</summary><p class="hint">Start with this sensor out of the water. Slowly lower it to the deepest intended depth, then lift it out. Repeat a few times and Finish & save. Only this board is paused.</p><div class="two"><button id="cal-start">Start calibration</button><button id="cal-stop">Finish & save</button></div><p id="calibration-status" role="status"></p><p id="calibration-action-status" role="status"></p></details>

<h3 class="tuning-heading">Tuning settings</h3><p id="draft-state" class="feedback" role="status"></p><div id="inputs"><input type="hidden" id="slider-enable" value="1"><input type="hidden" id="slider-calibration" value="0">
<div class="field-grid"><label>P · strength<input id="slider-p" type="number" min="0" max="4" step="0.1" value="0" inputmode="decimal"><span id="slider-p-value" hidden></span></label><label>I · correction<input id="slider-i" type="number" min="0" max="2" step="0.01" value="0" inputmode="decimal"><span id="slider-i-value" hidden></span></label><label>D · damping<input id="slider-d" type="number" min="0" max="2" step="0.01" value="0" inputmode="decimal"><span id="slider-d-value" hidden></span></label><label>Set Point<input id="slider-setpoint" type="number" min="1000" max="2000" step="1" value="1500" inputmode="decimal"><small>1000–2000 · 1500 is mid-range.</small><span id="slider-setpoint-value" hidden></span></label><label>Receiver influence (%)<input id="slider-factor" type="number" min="0" max="100" step="1" value="0" inputmode="decimal"><span id="slider-factor-value" hidden></span></label></div><details><summary>What these values do</summary><p>P sets correction strength; I corrects persistent error; D adds damping. Set Point uses the sensor’s saved calibration: 1000 is its minimum reading and 2000 its maximum. Receiver influence lets the transmitter shift the set point; 0% leaves the target unchanged.</p></details>
<details id="travel-details"><summary id="travel-title">Travel & direction</summary><div class="field-grid"><label><span id="endpoint-a-label">Endpoint A (µs)</span><input type="number" inputmode="numeric" id="slider-servo-min" min="1000" max="2000" step="1" value="1000" aria-describedby="servo-min-hint"><small id="servo-min-hint">Enter a value between 1000 and 2000 µs.</small><span id="slider-servo-min-value" hidden></span></label><label><span id="endpoint-b-label">Endpoint B (µs)</span><input type="number" inputmode="numeric" id="slider-servo-max" min="1000" max="2000" step="1" value="2000" aria-describedby="servo-max-hint"><small id="servo-max-hint">Enter a value between 1000 and 2000 µs.</small><span id="slider-servo-max-value" hidden></span></label></div><p id="direction-state" class="hint"></p><button id="reverse">Reverse direction</button><p class="hint" id="travel-hint">Travel changes pause the affected boards; press Run afterward.</p></details></div>
<div class="save-area"><button id="apply">Apply to selected board</button><label id="shared-travel-label" class="check"><input id="shared-travel" type="checkbox" checked>Match travel limits too</label><button id="apply-shared" class="primary">Apply to both boards</button><p id="shared-hint" class="hint">Each board keeps its own direction and sensor calibration.</p><p id="save-status" role="status" class="feedback"></p><button id="reload-settings" class="quiet">Refresh saved settings</button><div id="transfer-status" role="status" class="readout"></div></div></section>
<section class="card" id="test"><h2>Test</h2><p class="hint">Center first, then move one foil at a time. Watch its angle as you test.</p><button id="center-test" class="primary">Center both · 1500 µs</button><p class="hint">Always exactly 1500 µs on each board, independent of direction, endpoints or trim.</p><div id="servo-live-status" class="readout" role="status"></div><details id="servo-test-card"><summary>Move one servo</summary><label for="pulse-target">Servo to move</label><select id="pulse-target"><option value="local">Connected board</option></select><label for="pulse-value">Test value (µs)</label><div class="pulse-row"><button id="pulse-down" aria-label="Decrease reference by 10 microseconds">−10</button><input id="pulse-value" type="number" min="1000" max="2000" step="1" value="1500"><button id="pulse-up" aria-label="Increase reference by 10 microseconds">+10</button></div><button id="pulse-send">Hold this pulse</button><p class="hint">1000–2000. Saved direction is applied; actual output appears above. This test bypasses travel limits. Start near 1500 and check linkage clearance.</p><p id="pulse-feedback" class="feedback" role="status"></p></details><p id="servo-action-status" class="feedback" role="status"></p><div class="two"><button id="exit-test">End test & run both</button><button id="pause-test">End test & pause both</button></div><details><summary>Last commands & test details</summary><div id="test-status" class="readout" role="status"></div><p>Only the selected servo moves. Test holds until Pause, Run or restart. Restart returns to automatic control with valid calibration. Reversed boards mirror the reference around 1500. Live output above is the commanded pulse, not a measurement of foil angle. Last-command receipts confirm delivery separately.</p></details><details><summary>Recording & diagnostics</summary><p id="recorder-scope" class="hint">These tools use the Wi-Fi-connected board only.</p><p><a href="/bench">30-second recorder</a> · <a href="/diagnostics">Detailed diagnostics</a></p></details></section>

<section class="card"><details><summary>Connection & setup</summary><p><a href="/setup">Board role and restart</a></p><div id="pairing-tools"><label for="mac-input">Add a slave · station MAC address</label><input id="mac-input" type="text" placeholder="FF:FF:FF:FF:FF:FF"><p class="hint">First-time pairing only: copy the slave’s station MAC from its Device setup page, then return to the master’s Wi-Fi. Pairing is saved after power-off.</p><button id="submit-button">Add board</button><p id="slave-device-list"></p><button id="remove-slaves-button">Remove paired boards</button></div><button id="wifi-off-button">Hide Wi-Fi until restart</button><p class="hint">Restart restores Wi-Fi and starts normal control. Saved settings are retained.</p></details></section><footer>Hydrofoil R5.5</footer></main><script src="/script.js?v=r55"></script></body></html>
)R5ASSET";
static const char kControllerScript[] PROGMEM = R"R5ASSET("use strict";
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
)R5ASSET";
static const char kControllerStyle[] PROGMEM = R"R5ASSET(*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:170px}body{margin:0;background:#eef2f5;color:#192d3c;font:17px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}header{position:sticky;top:0;z-index:4;background:#122c3c;color:#fff;padding:12px max(16px,calc((100% - 680px)/2));box-shadow:0 2px 10px #0002}.brand{font-size:11px;font-weight:750;letter-spacing:2px;color:#a9c4d4;white-space:nowrap}.brand span{float:right;letter-spacing:0}.identity{display:flex;gap:12px;align-items:center;margin:5px 0 10px}.identity>strong{font-size:38px;line-height:1.2}.identity b{display:block;font-size:18px}.identity small{display:block;color:#bbd0dc;font-size:11px;overflow-wrap:anywhere}.chip{border-radius:20px;background:#e5edf3;color:#27465c;padding:5px 10px;font-size:12px;font-weight:650}.identity .chip{margin-left:auto;background:#29495b;color:#e3f6ff}nav{display:flex;gap:8px}nav a{flex:1;text-align:center;padding:8px;color:#fff;text-decoration:none;border-radius:6px;background:#ffffff12;font-weight:650}main,.content{max-width:712px;margin:auto;padding:16px}.card{background:#fff;border:1px solid #d5dfe6;border-radius:14px;padding:18px;margin:0 0 14px;box-shadow:0 2px 6px #142c3c05}h1,h2{font-size:23px;margin:0 0 14px;letter-spacing:-.4px}h3{font-size:16px;margin:0}p{margin:10px 0}label{display:block;font-weight:650;font-size:15px;margin:10px 0 4px}.field-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px 14px}input,select,button{font:inherit;min-height:48px;border-radius:8px;border:1px solid #a8bac6;padding:10px;width:100%;max-width:100%;color:inherit}input,select{background:white}input[type=number]{font-size:21px;font-variant-numeric:tabular-nums}button{font-weight:650;background:#edf3f6;cursor:pointer;margin:5px 0}button.primary,button[aria-pressed=true]{background:#096c9c;border-color:#096c9c;color:#fff}button:disabled,input:disabled,select:disabled{opacity:.5;cursor:default}.two{display:grid;grid-template-columns:1fr 1fr;gap:10px}.section-title{display:flex;align-items:center;justify-content:space-between;gap:10px}.section-title h1{margin:0}details{border-top:1px solid #dde5ea;margin-top:14px;padding-top:10px}details:first-child{margin-top:0;border-top:0;padding-top:0}summary{font-weight:650;cursor:pointer;min-height:44px;padding:8px 0}details p,.hint,small{font-size:13px;color:#546b7a}.hint{margin:8px 0 12px}small{display:block;font-weight:400;margin-top:5px}.readout{font-size:14px;background:#f0f5f8;border-radius:8px;padding:10px;white-space:pre-line;overflow-wrap:anywhere}.readout:empty,.feedback:empty{display:none}.feedback{font-size:14px;font-weight:600}.save-area{border-top:1px solid #dde5ea;margin-top:16px;padding-top:12px}.pulse-row{display:grid;grid-template-columns:64px 1fr 64px;gap:8px;align-items:center}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:8px;font-size:13px}.metrics p{margin:0;padding:8px;background:#f1f5f8;border-radius:6px}a{color:#086b9c}footer{text-align:center;font-size:12px;color:#617887;padding:10px 0 20px}[hidden]{display:none!important}:focus-visible{outline:3px solid #efac38;outline-offset:2px}@media(max-width:380px){main{padding:10px}.card{padding:14px}.field-grid{gap:6px 10px}.identity>strong{font-size:32px}.chip{font-size:11px;padding:4px 7px}}
.check{display:flex;align-items:center;gap:10px;font-weight:500}.check input{width:24px;min-height:24px;height:24px;accent-color:#096c9c;flex-shrink:0}

.overview{display:grid;gap:8px}.board-summary{padding:12px;border:1px solid #d8e3eb;border-radius:10px;background:#f6f9fb}.board-summary p{margin:4px 0;font-size:14px}.board-summary strong{font-size:19px}.next-step{border-left:3px solid #1680aa;padding:8px 12px;background:#eef7fb;font-size:14px;border-radius:0 6px 6px 0}.next-step:empty{display:none}.tuning-heading{margin-top:24px}.quiet{background:transparent;border-color:transparent;font-size:14px;text-decoration:underline}.metrics{margin-top:8px}.metrics strong{display:block;font-size:20px;line-height:1.4;font-variant-numeric:tabular-nums}.metrics small{margin:0}.identity .chip{white-space:nowrap}button[aria-pressed=true]{box-shadow:inset 0 -3px 0 #004f78}

.board-tabs{display:flex;gap:8px}.board-tabs button{flex:1;min-width:0;font-size:20px;padding:10px 6px}.board-tabs button[aria-pressed=true]{background:#096c9c;color:white;border-color:#096c9c}

#target-note:empty{display:none}
.status-list{padding:0;background:transparent;display:grid;gap:8px}.board-status-row{padding:10px;border-radius:8px;background:#f0f5f8}.board-summary[data-state=paused],.readout[data-state=paused],.board-status-row[data-state=paused]{background:#fff5df;border-left:4px solid #b76a00;color:#784400}.board-summary[data-state=fault],.readout[data-state=fault],.board-status-row[data-state=fault]{background:#fff0ee;border-left:4px solid #bd3027;color:#a0251c}.board-summary[data-state=paused] .status-label,.board-summary[data-state=fault] .status-label,.readout[data-state=paused],.readout[data-state=fault],.board-status-row[data-state=paused],.board-status-row[data-state=fault]{font-weight:700}
)R5ASSET";
static const char kControllerSetup[] PROGMEM = R"R5ASSET(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Hydrofoil device setup</title><link rel="stylesheet" href="/style.css"></head><body><div class="content"><h1>Device setup</h1><p><strong id="setup-name" style="font-size:32px">—</strong> · <b id="setup-role">Connecting</b></p><div class="card"><p>Station MAC for pairing: <strong id="mac">…</strong></p><form action="/setup" method="POST"><label>Role <select name="role" id="role" disabled><option value="master">Master</option><option value="slave">Slave</option></select></label><p>Use Master for the board whose Wi-Fi you’ll normally join; use Slave for the other board.</p><p>Saving restarts this board. Its HF-Bench hotspot remains available in either role.</p><button id="save-role" disabled>Save role and restart</button><p id="setup-error" role="status"></p></form><p>After initial pairing, use the master’s screen for both boards. Firmware updates and recovery still use each board’s own Wi-Fi.</p><a href="/">Return to controller</a></div></div><script>const role=document.getElementById('role'),save=document.getElementById('save-role'),error=document.getElementById('setup-error');async function loadSetup(){try{const [m,d]=await Promise.all([fetch('/get-mac'),fetch('/device-status')]);if(!m.ok||!d.ok)throw Error('Could not load device settings');document.getElementById('mac').textContent=await m.text();const state=await d.json();document.getElementById('setup-name').textContent=(state.ssid||'—').slice(-2);document.getElementById('setup-role').textContent=state.slave?'Slave':'Master';role.value=state.slave?'slave':'master';role.disabled=false;save.disabled=false;}catch(e){error.textContent='Settings unavailable. Reload before saving.';}}loadSetup();</script></body></html>)R5ASSET";
