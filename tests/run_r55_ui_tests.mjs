// Execute the shipped browser scripts with a minimal DOM and simulated board.
// These tests do not establish radio behavior, browser rendering, or servo safety.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';

const project='.';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..',project);
const read=p=>fs.readFileSync(path.join(root,p),'utf8');
const controller=read('data/script.js'), html=read('data/index.html'), setup=read('data/setup.html');
const bench=read('include/bench_page.h');
const embedded=read('include/controller_assets.h');
for(const [name,source] of [['Index',html],['Script',controller],['Setup',setup],['Style',read('data/style.css')]]){
  const prefix='static const char kController'+name+'[] PROGMEM = R"R5ASSET(';
  assert(embedded.includes(prefix));
  assert.equal(embedded.split(prefix)[1].split(')R5ASSET";')[0],source,'Embedded '+name+' must match verified source');
}
console.log('PASS: embedded application UI exactly matches source HTML, JavaScript, and style');
const inline=s=>[...s.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g)].map(m=>m[1]).join('\n');
const fields=['p','i','d','setpoint','factor','enable','calibration','servo-min','servo-max'].map(v=>'slider-'+v);
for(const [name,code] of [['controller',controller],['setup',inline(setup)],['bench',inline(bench)]])new vm.Script(code,{filename:name});
console.log('PASS: controller, setup, and recorder JavaScript syntax');

class Element {
  constructor(id=''){this.id=id;this.value='';this.textContent='';this.disabled=false;this.hidden=false;this.style={};this.options=[];this.children=[];this.events={};this.clientWidth=0;this.clientHeight=0;this.classList={toggle(){}};}
  get textContent(){return this._text+this.children.map(el=>el.textContent).join('');}
  set textContent(value){this._text=String(value);this.children=[];this.options=[];}
  setAttribute(name,value){this[name]=value;}
  addEventListener(name,fn){(this.events[name]??=[]).push(fn);}
  appendChild(el){this.children.push(el);this.options.push(el);}
  dispatch(name,event={}){for(const fn of this.events[name]??[])fn({target:this,preventDefault(){},...event});}
}
function harness(code,markup,handler){
  const elements=new Map();
  for(const match of markup.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)){
    const el=new Element(match[1]);el.disabled=/\bdisabled\b/.test(match[0]);elements.set(el.id,el);
  }
  for(const id of ['board-selector','pulse-target'])if(elements.has(id)){const o=new Element();o.value=id==='board-selector'?'master':'local';elements.get(id).options.push(o);}
  const calls=[],timers=[],modeButtons=[new Element(),new Element(),new Element()];
  const document={getElementById(id){assert(elements.has(id),'Missing DOM id '+id);return elements.get(id);},createElement(){return new Element();},querySelectorAll(selector){assert.equal(selector,'#mode-form button');return modeButtons;}};
  const context=vm.createContext({document,window:{addEventListener(){},devicePixelRatio:1},URLSearchParams,AbortController,console,confirm:()=>true,location:{reload(){}},setTimeout(fn,ms){timers.push({fn,ms});return timers.length;},clearTimeout(){},fetch:async(url,options={})=>{const call={url,options,method:options.method??'GET'};calls.push(call);return handler(call);}});
  vm.runInContext(code,context);
  return {elements,calls,timers,context,el:id=>document.getElementById(id)};
}
const response=(value,status=200)=>({ok:status>=200&&status<300,status,json:async()=>value,text:async()=>typeof value==='string'?value:JSON.stringify(value)});
const flush=async()=>{for(let i=0;i<12;i++)await new Promise(resolve=>setImmediate(resolve));};
const settings=offset=>Object.fromEntries(fields.map((name,i)=>[name,offset+i]));
const posts=h=>h.calls.filter(c=>c.method==='POST');


let failReadback=false,remoteOverrides={};
let fresh=true,remoteCalibrationPending=false,calibrationMessage='',remoteP=2.7;
let calibrationEnabled=false, slave=false, pending=false, delivery='Not sent', reject=false;
const values=Object.fromEntries(fields.map(n=>[n,0]));Object.assign(values,{'slider-servo-min':1100,'slider-servo-max':1900,'slider-enable':1,'slider-setpoint':1500});
const handler=({url,method,options})=>{
 if(url==='/board-state'){if(failReadback)return response('No connection',503);const state={fresh,available:fresh,settings_valid:true,calibration_valid:true,position_valid:true,age_ms:100,mode:'bench',sampling:true,sensor_healthy:true,servo_attached:false,calibration_enabled:calibrationEnabled,calibration_active:false,calibration_pending:remoteCalibrationPending,calibration_status:calibrationMessage,settings:values};return response({master:state,'slave-1':{...state,settings:{...values,'slider-p':remoteP,...remoteOverrides}}});}
 if(url==='/remote-calibration'){remoteCalibrationPending=true;calibrationMessage='Waiting for board confirmation';return response('Sent',202);}
 if(url==='/device-status')return response({slave,controller_mode:'bench',ssid:slave?'HF-Alternate-Z7':'HF-Custom-Q9',ap_mac:'00:00:00:00:00:29'});
 if(url==='/board-identities')return response({'slave-1':{name:'HF-Alternate-Z7'}});
 if(url==='/shared-tuning'){pending=true;delivery='Waiting for board confirmation';return response('OK');}
 if(url==='/servo-test-status')return response({local:'Holding exactly 1500 us','slave-1':{address:'DC:06:75:32:AB:B4',pending:false,status:'Confirmed: holding 1500 us'},'slave-2':{address:''}});
 if(url==='/servo-test')return response('OK');
 if(url==='/diagnostics')return response({calibration_active:false,calibration_enabled:calibrationEnabled,calibration_notice:'Calibration OFF'});
 if(url==='/get-addresses')return response({broadcastAddress0:'DC:06:75:32:AB:B4',broadcastAddress1:''});
 if(url.startsWith('/get-settings'))return response(values);
 if(url==='/get-values')return response({'Actual-measured-value':1234});
 if(url==='/peer-status')return response({'slave-1':{pending,status:delivery}});
 if(url==='/set-sliders'){if(reject)return response('Cannot save',500);pending=true;delivery='Waiting for board confirmation';return response('OK');}
 if(url==='/calibration')return response('OK');
 throw Error('Unexpected '+method+' '+url);
};
const h=harness(controller,html,handler);await flush();assert.equal(posts(h).length,0);
// Actual DOM range values are strings.
for(const id of fields)h.el(id).value=String(h.el(id).value);
h.el('slider-p').value='2';h.el('inputs').dispatch('input',{target:h.el('slider-p')});
assert.equal(posts(h).length,0,'Editing must not send');
h.el('reverse').onclick();assert.equal(h.el('slider-servo-min').value,'1900');assert.equal(h.el('slider-servo-max').value,'1100');assert.equal(posts(h).length,0);
h.el('board-selector').value='slave-1';h.el('board-selector').dispatch('change');await flush();assert.equal(posts(h).length,0,'Selecting does not send');
assert(!h.el('calibration-card').hidden);assert.equal(h.el('slider-p').value,String(remoteP));assert.equal(posts(h).length,0);
await h.el('apply').onclick();assert.equal(posts(h).length,1);assert(h.el('apply').disabled);assert(h.el('save-status').textContent.includes('waiting'));
assert.equal(new URLSearchParams(posts(h)[0].options.body).get('slider-enable'),'1');
pending=false;delivery='Unconfirmed: no reply';await vm.runInContext('poll()',h.context);assert.equal(h.el('transfer-status').textContent,delivery);assert(!h.el('apply').disabled);
await h.el('apply').onclick();pending=false;delivery='Confirmed: board saved settings';await vm.runInContext('poll()',h.context);assert.equal(h.el('transfer-status').textContent,delivery);
reject=true;await h.el('apply').onclick();assert(h.el('save-status').textContent.startsWith('Not confirmed'));reject=false;
slave=true;const local=harness(controller,html,handler);await flush();assert(local.el('selector-card').hidden);assert(!local.el('calibration-card').hidden);
await local.el('cal-start').onclick();assert.equal(posts(local).length,1);assert.equal(posts(local)[0].url,'/calibration');assert.equal(new URLSearchParams(posts(local)[0].options.body).get('enabled'),'1');
assert(!html.includes('width: 25px'));assert(html.includes('Calibrate selected board'));
console.log('PASS: actual UI draft/apply, reverse, sensor-only tuning, pending/confirmed/unconfirmed/error states, selected-board calibration');

calibrationEnabled=true;await vm.runInContext('poll()',local.context);assert(local.el('cal-start').disabled);assert(!local.el('cal-stop').disabled,'Interrupted calibration can always be turned off');assert(local.el('apply').disabled,'Finish calibration before applying tuning');
console.log('PASS interrupted calibration exposes Finish despite inactive session');

await h.el('center-test').onclick();let c=posts(h).at(-1);assert.equal(c.url,'/servo-test');assert.equal(c.options.body.get('action'),'center');assert.equal(c.options.body.get('scope'),'all');
await local.el('center-test').onclick();c=posts(local).at(-1);assert.equal(c.options.body.get('scope'),'local');
assert(local.el('center-test').textContent.includes('THIS'));assert(h.el('test-status').textContent.includes('Confirmed'));
console.log('PASS test buttons independent from settings target; local slave scope and individual confirmation');

h.el('pulse-target').value='slave-1';h.el('pulse-value').value='1600';let count=posts(h).length;
h.el('pulse-up').onclick();assert.equal(h.el('pulse-value').value,'1610');assert.equal(posts(h).length,count);
await h.el('pulse-send').onclick();c=posts(h).at(-1);assert.equal(c.options.body.get('pulse'),'1610');assert.equal(c.options.body.get('scope'),'slave-1');
h.el('pulse-value').value='2001';count=posts(h).length;await h.el('pulse-send').onclick();assert.equal(posts(h).length,count);
local.el('pulse-value').value='1400';await local.el('pulse-send').onclick();assert.equal(posts(local).at(-1).options.body.get('scope'),'local');
await h.el('center-test').onclick();assert.equal(posts(h).at(-1).options.body.get('action'),'center');assert.equal(posts(h).at(-1).options.body.get('pulse'),null);
console.log('PASS manual pulse drafts, explicit one-board send, validation and independent center command');

for(const id of ['slider-servo-min','slider-servo-max'])assert(html.includes('id="'+id+'"'));
calibrationEnabled=false;await vm.runInContext('poll()',local.context);
for(const bad of ['', '999','2001','1500.5']){local.el('slider-servo-min').value=bad;count=posts(local).length;await local.el('apply').onclick();assert.equal(posts(local).length,count);}
local.el('slider-servo-min').value='1809';local.el('slider-servo-max').value='1207';count=posts(local).length;
local.el('inputs').dispatch('input',{target:local.el('slider-servo-min')});assert.equal(posts(local).length,count);
await local.el('apply').onclick();c=posts(local).at(-1);assert.equal(c.url,'/set-sliders');assert.equal(c.options.body.get('slider-servo-min'),'1809');assert.equal(c.options.body.get('slider-servo-max'),'1207');
console.log('PASS typed endpoints: no send while editing, exact reversed values on Apply, invalid entries rejected');

assert.equal(h.el('board-id').textContent,'Q9');assert.equal(local.el('board-id').textContent,'Z7');assert(h.el('pulse-target').options.some(o=>o.textContent==='Z7 · Slave 1'));
// Use the actual visible target buttons, including the new shared target.
slave=false;pending=false;calibrationEnabled=false;
const g=harness(controller,html,handler);await flush();
const choose=async(h,key)=>{await vm.runInContext(`targetButtons[${JSON.stringify(key)}].onclick()`,h.context);await flush();};
assert(html.includes('type="hidden" id="board-selector"'));
assert.equal(vm.runInContext("targetButtons.master.textContent",g.context),'Q9');
assert.equal(vm.runInContext("targetButtons['slave-1'].textContent",g.context),'Z7');
assert.equal(vm.runInContext("targetButtons.both.textContent",g.context),'Both');
await choose(g,'both');assert.equal(posts(g).length,0,'Selecting Both only reads');
assert(g.el('apply').hidden&&!g.el('apply-shared').hidden&&g.el('reverse').hidden);
assert(g.el('calibration-card').hidden&&!g.el('group-calibration-hint').hidden);
assert.equal(g.el('slider-p').value,'');assert.equal(g.el('slider-p').placeholder,'Different');
assert.equal(g.el('slider-i').value,'0');
g.el('shared-travel').checked=true;count=posts(g).length;
await g.el('apply-shared').onclick();assert.equal(posts(g).length,count,'Mixed P needs an explicit value');
await g.el('cal-start').onclick();await g.el('apply').onclick();assert.equal(posts(g).length,count,'Both cannot calibrate or send a per-board settings command');
g.el('slider-p').value='2';g.el('inputs').dispatch('input',{target:g.el('slider-p')});
await vm.runInContext('poll()',g.context);assert.equal(g.el('slider-p').value,'2','Group draft survives polls');
await g.el('apply-shared').onclick();c=posts(g).at(-1);assert.equal(c.url,'/shared-tuning');assert.equal(c.options.body.get('include-travel'),'1');assert(!c.options.body.has('slider-calibration'));assert(!c.options.body.has('board-selector'));
assert(g.el('apply-shared').disabled);assert(vm.runInContext('targetButtons.master.disabled',g.context));
pending=false;delivery='Confirmed: board saved settings';await vm.runInContext('poll()',g.context);assert(g.el('save-status').textContent.includes('confirmed on all boards'));assert.equal(g.el('draft-state').textContent,'');
await g.el('resume-test').onclick();assert.equal(posts(g).at(-1).options.body.get('scope'),'all');
await g.el('stop-test').onclick();assert.equal(posts(g).at(-1).options.body.get('action'),'pause');assert.equal(posts(g).at(-1).options.body.get('scope'),'all');
// Reversed order alone is not a disagreement in travel limits.
remoteOverrides={'slider-servo-min':1900,'slider-servo-max':1100};await g.el('reload-settings').onclick();assert.equal(g.el('slider-servo-min').value,'1100');assert.equal(g.el('slider-servo-max').value,'1900');
remoteOverrides={'slider-servo-min':1201,'slider-servo-max':1796,'slider-enable':0};await g.el('reload-settings').onclick();assert.equal(g.el('slider-servo-min').value,'');assert.equal(g.el('slider-enable').value,'1');
g.el('slider-p').value='2';g.el('inputs').dispatch('input',{target:g.el('slider-p')});count=posts(g).length;
await g.el('apply-shared').onclick();assert.equal(posts(g).length,count,'Mixed travel needs explicit bounds');assert(g.el('travel-details').open);
g.el('slider-servo-min').value='1150';g.el('inputs').dispatch('input',{target:g.el('slider-servo-min')});
g.el('shared-travel').checked=false;g.el('shared-travel').onchange();await g.el('apply-shared').onclick();c=posts(g).at(-1);assert(!c.options.body.has('slider-servo-min'),'Travel opt-out omits all travel values');assert.equal(c.options.body.get('slider-enable'),'1');
pending=false;delivery='Confirmed: board saved settings';await vm.runInContext('poll()',g.context);assert(g.el('save-status').textContent.includes('Travel edits are still unsaved'));assert(g.el('draft-state').textContent.includes('Unsaved'));
vm.runInContext('confirm=()=>false',g.context);await choose(g,'master');assert.equal(vm.runInContext('activeBoard',g.context),'both','Cancel discard keeps Both and its draft');
vm.runInContext('confirm=()=>true',g.context);await choose(g,'slave-1');assert(!g.el('apply').hidden&&g.el('apply-shared').hidden&&!g.el('reverse').hidden);assert(!g.el('calibration-card').hidden);assert.equal(g.el('slider-servo-min').value,'1201');
await choose(g,'both');fresh=false;await vm.runInContext('poll()',g.context);assert(g.el('apply-shared').disabled);assert(!vm.runInContext('targetButtons.master.disabled',g.context),'Offline target can be changed');
count=posts(g).length;await g.el('apply-shared').onclick();assert.equal(posts(g).length,count);fresh=true;remoteOverrides={};await choose(g,'master');
console.log('PASS dynamic name buttons, Both mixed values, explicit common settings, direction/calibration isolation, shared travel opt-out, confirmations, drafts, all-board Run/Pause and offline recovery');

// The master uses actual slave readback, keeps drafts intact, and blocks stale writes.
slave=false;pending=false;delivery='Confirmed: board saved settings';await vm.runInContext('poll()',h.context);
await vm.runInContext("loadBoard('slave-1')",h.context);assert.equal(h.el('slider-p').value,'2.7');assert(h.el('target-note').textContent.includes('Saved settings read from Z7'));
h.el('slider-p').value='3.2';h.el('inputs').dispatch('input',{target:h.el('slider-p')});remoteP=1.4;await vm.runInContext('poll()',h.context);assert.equal(h.el('slider-p').value,'3.2','Polling must preserve unsaved draft');
fresh=false;await vm.runInContext('poll()',h.context);assert(h.el('apply').disabled&&h.el('cal-start').disabled);assert(!h.el('board-selector').disabled);assert(h.el('selected-status').textContent.includes('STALE'));count=posts(h).length;await h.el('apply').onclick();assert.equal(posts(h).length,count);
fresh=true;await h.el('reload-settings').onclick();assert.equal(h.el('slider-p').value,'1.4');
await h.el('cal-start').onclick();c=posts(h).at(-1);assert.equal(c.url,'/remote-calibration');assert.equal(c.options.body.get('board'),'slave-1');assert.equal(c.options.body.get('enabled'),'1');assert(h.el('cal-start').disabled);assert(h.el('calibration-action-status').textContent.includes('Waiting'));
remoteCalibrationPending=false;calibrationEnabled=true;calibrationMessage='Confirmed: calibration started; output paused';await vm.runInContext('poll()',h.context);assert(!h.el('cal-stop').disabled);assert(h.el('calibration-action-status').textContent.includes('Confirmed'));
await h.el('cal-stop').onclick();assert.equal(posts(h).at(-1).options.body.get('enabled'),'0');
console.log('PASS actual slave readback, draft preservation, stale-data write blocking, recovery and confirmed remote calibration targeting');

// Contextual next steps, truthful board-wide state and feedback beside the controls.
remoteCalibrationPending=false;calibrationEnabled=false;fresh=true;pending=false;await vm.runInContext("loadBoard('slave-1')",h.context);await vm.runInContext('poll()',h.context);
assert(h.el('next-step').textContent.includes('Press Run'));assert(h.el('servo-live-status').textContent.includes('Z7'));
await h.el('pause-selected').onclick();assert(h.el('selected-action-status').textContent.includes('Z7'));assert.equal(posts(h).at(-1).options.body.get('scope'),'slave-1');
await h.el('exit-test').onclick();assert.equal(posts(h).at(-1).options.body.get('action'),'resume');assert.equal(posts(h).at(-1).options.body.get('scope'),'all');
await h.el('reload-settings').onclick();h.el('slider-setpoint').value='';count=posts(h).length;await h.el('apply').onclick();assert.equal(posts(h).length,count);assert(h.el('draft-state').textContent.includes('Set Point'));
console.log('PASS contextual next steps, selected-board confirmations, test exit scope and input validation');

// Capture every intermediate summary write, not just the final DOM after polling.
let summaryText=h.el('mode-status').textContent;const summaryWrites=[];
Object.defineProperty(h.el('mode-status'),'textContent',{get(){return summaryText;},set(v){summaryText=v;summaryWrites.push(v);},configurable:true});
await vm.runInContext('poll()',h.context);assert.equal(summaryText,'2 of 2 boards connected');assert.equal(summaryWrites.length,0,'Unchanged polling should not rewrite the summary or flash local mode text');
failReadback=true;await vm.runInContext('poll()',h.context);assert.deepEqual(summaryWrites,['Connection lost — board state unconfirmed']);
summaryWrites.length=0;await vm.runInContext('poll()',h.context);assert.equal(summaryWrites.length,0,'Persistent failure must not alternate with the board count');
failReadback=false;await vm.runInContext('poll()',h.context);assert.deepEqual(summaryWrites,['2 of 2 boards connected']);
console.log('PASS no intermediate Control-summary flashing, unchanged polls do not rewrite, connection loss and recovery update once');

// Status colors follow each board independently and never turn paused sensor data into a fault.
vm.runInContext(`
 boardStates.master={...boardStates.master,mode:'controller',servo_attached:true,servo_us:1500,sensor_healthy:true,position_valid:true,calibration_enabled:false,calibration_pending:false,error:''};
 boardStates['slave-1']={...boardStates.master,mode:'bench',sampling:false,servo_attached:false,sensor_healthy:false,position_valid:false};
 activeBoard='both';showBoardState();showOverview();
`,h.context);
const checkColors=(expected)=>{
 for(const id of ['overview-boards','selected-status','servo-live-status']){
  assert.deepEqual(h.el(id).children.map(el=>el['data-state']),expected,id+' must color each board separately');
 }
};
checkColors(['normal','paused']);assert(h.el('overview-boards').children[1].textContent.includes('PAUSED — servo output off'));
vm.runInContext("boardStates['slave-1']={...boardStates['slave-1'],mode:'controller',sampling:true,servo_attached:true};showBoardState();showOverview()",h.context);
checkColors(['normal','fault']);assert(h.el('overview-boards').children[1].textContent.includes('SENSOR TIMEOUT'));
vm.runInContext("boardStates['slave-1'].sensor_healthy=true;boardStates['slave-1'].position_valid=true;showBoardState();showOverview()",h.context);
checkColors(['normal','normal']);
vm.runInContext("boardStates['slave-1'].fresh=false;showBoardState();showOverview()",h.context);
checkColors(['normal','fault']);assert(h.el('selected-status').textContent.includes('OFFLINE / STALE'));
vm.runInContext("Object.assign(boardStates['slave-1'],{fresh:true,mode:'bench',servo_attached:false,calibration_enabled:true,calibration_active:true,sensor_healthy:false,position_valid:false});showBoardState();showOverview()",h.context);
checkColors(['normal','paused']);assert(h.el('selected-status').textContent.includes('Calibrating'));
vm.runInContext("Object.assign(boardStates['slave-1'],{mode:'center-test',servo_attached:true,calibration_enabled:false,servo_us:1600});showBoardState();showOverview()",h.context);
checkColors(['normal','normal']);assert(h.el('selected-status').textContent.includes('TEST · holding 1600'));
vm.runInContext("Object.assign(boardStates['slave-1'],{mode:'bench',error:'Servo output could not start'});showBoardState();showOverview()",h.context);
checkColors(['normal','fault']);assert(h.el('overview-boards').children[1].textContent.includes('Servo output could not start'));
vm.runInContext("activeBoard='slave-1';showBoardState()",h.context);assert.equal(h.el('selected-status')['data-state'],'fault');assert.equal(h.el('selected-status').className,'readout');
vm.runInContext("boardStates['slave-1'].error='';showBoardState()",h.context);assert.equal(h.el('selected-status')['data-state'],'paused');
console.log('PASS independent amber pause / red fault states across Control, Tune and Test; recovery, offline, calibration, held tests and explicit errors remain distinct');
