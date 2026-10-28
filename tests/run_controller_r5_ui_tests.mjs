// Execute the shipped browser scripts with a minimal DOM and simulated board.
// These tests do not establish radio behavior, browser rendering, or servo safety.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';

const project=process.argv[2]??'.';

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
  addEventListener(name,fn){(this.events[name]??=[]).push(fn);}
  appendChild(el){this.children.push(el);this.options.push(el);}
  dispatch(name,event={}){for(const fn of this.events[name]??[])fn({target:this,preventDefault(){},...event});}
}
function harness(code,markup,handler){
  const elements=new Map();
  for(const match of markup.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)){
    const el=new Element(match[1]);el.disabled=/\bdisabled\b/.test(match[0]);elements.set(el.id,el);
  }
  const calls=[],timers=[],modeButtons=[new Element(),new Element(),new Element()];
  const document={getElementById(id){assert(elements.has(id),'Missing DOM id '+id);return elements.get(id);},createElement(){return new Element();},querySelectorAll(selector){assert.equal(selector,'#mode-form button');return modeButtons;}};
  const context=vm.createContext({document,window:{addEventListener(){},devicePixelRatio:1},URLSearchParams,console,setTimeout(fn,ms){timers.push({fn,ms});return timers.length;},clearTimeout(){},fetch:async(url,options={})=>{const call={url,options,method:options.method??'GET'};calls.push(call);return handler(call);}});
  vm.runInContext(code,context);
  return {elements,calls,timers,context,el:id=>document.getElementById(id)};
}
const response=(value,status=200)=>({ok:status>=200&&status<300,status,json:async()=>value,text:async()=>typeof value==='string'?value:JSON.stringify(value)});
const flush=async()=>{for(let i=0;i<12;i++)await new Promise(resolve=>setImmediate(resolve));};
const settings=offset=>Object.fromEntries(fields.map((name,i)=>[name,offset+i]));
const posts=h=>h.calls.filter(c=>c.method==='POST');

let releaseSlave=null,saveFails=false,loadFails=false;
const h=harness(controller,html,({url,method})=>{
  if(url==='/diagnostics')return response({calibration_active:true,calibration_samples:40,calibration_range_ready:true});
  if(url==='/device-status')return response({slave:false,controller_mode:'bench'});
  if(url==='/get-addresses')return response({broadcastAddress0:'AA:BB:CC:DD:EE:FF'});
  if(url==='/get-values')return response({'PID-input':1500});
  if(url.startsWith('/get-settings')){
    if(loadFails)return response('Board unavailable',503);
    if(url.includes('slave-1')&&releaseSlave===null)return new Promise(resolve=>releaseSlave=()=>resolve(response(settings(20))));
    return response(settings(url.includes('slave-1')?20:10));
  }
  if(url==='/set-sliders'&&method==='POST')return saveFails?response('Could not save settings',500):response('OK');
  throw Error('Unexpected request '+url);
});
assert(h.el('slider-p').disabled,'Sliders must be disabled during initial load');
await flush();
assert.equal(posts(h).length,0,'Opening the controller must never save');
assert.equal(String(h.el('slider-p').value),'10');
assert.equal(h.el('slider-p').disabled,false);
console.log('PASS: opening the controller loads stored values without POST');

h.el('board-selector').value='slave-1';h.el('board-selector').dispatch('change');
assert(h.el('slider-p').disabled&&h.el('board-selector').disabled);
h.el('inputs').dispatch('change',{target:h.el('slider-p')});await flush();
assert.equal(posts(h).length,0,'Stale slider change during board load must not save');
releaseSlave();await flush();
assert.equal(String(h.el('slider-p').value),'20');
h.el('slider-p').value='1.2';h.el('inputs').dispatch('input',{target:h.el('slider-p')});
assert.equal(h.el('slider-p-value').textContent,'1.2');
h.el('inputs').dispatch('change',{target:h.el('slider-p')});await flush();
assert.equal(posts(h).length,1);
const submitted=Object.fromEntries(posts(h)[0].options.body);
assert.equal(submitted['board-selector'],'slave-1');
assert.equal(submitted['slider-p'],'1.2');
for(const name of fields.slice(1))assert.equal(submitted[name],String(settings(20)[name]));
console.log('PASS: selecting a board cannot autosave stale values; save sends that board and all its displayed fields');

saveFails=true;h.el('slider-p').value='1.3';h.el('inputs').dispatch('change',{target:h.el('slider-p')});await flush();
assert.match(h.el('save-status').textContent,/Not saved.*Could not save settings/,'Rejected save must remain visibly identified after restoring stored values');
assert.equal(String(h.el('slider-p').value),'20');
loadFails=true;h.el('board-selector').value='slave-2';h.el('board-selector').dispatch('change');await flush();
assert.match(h.el('save-status').textContent,/Board unavailable/);assert(h.el('slider-p').disabled);
h.el('inputs').dispatch('change');await flush();assert.equal(posts(h).length,2);
console.log('PASS: failed save remains visible, restores stored values; failed board load disables saving');

let releaseSetup;
const setupOk=harness(inline(setup),setup,({url})=>url==='/get-mac'?response('AA:BB:CC:DD:EE:FF'):new Promise(resolve=>releaseSetup=()=>resolve(response({slave:true}))));
assert(setupOk.el('role').disabled&&setupOk.el('save-role').disabled);
releaseSetup();await flush();
assert.equal(setupOk.el('role').value,'slave');assert.equal(setupOk.el('role').disabled,false);assert.equal(setupOk.el('save-role').disabled,false);assert.equal(posts(setupOk).length,0);
const setupError=harness(inline(setup),setup,({url})=>url==='/get-mac'?response('AA:BB:CC:DD:EE:FF'):response('Unavailable',503));
await flush();assert(setupError.el('role').disabled&&setupError.el('save-role').disabled);assert.match(setupError.el('setup-error').textContent,/Settings unavailable/);assert.equal(posts(setupError).length,0);
console.log('PASS: setup cannot save an unloaded/default role; actual slave role loads without POST; load failure remains visible');

const stat={count:0,min:null,max:null,span:null,mean:null,stddev:null};
const diagnostics=(state,session=7)=>({build:'R5-test',controller_mode:'bench',recording:{state,session_id:session,elapsed_ms:state==='finished'?30000:1000,duration_ms:30000,samples:state==='finished'?2:0,good:state==='finished'?2:0,timeouts:0,raw:stat,filtered:stat,reason:'completed'}});
const recording=harness(inline(bench),bench,({url})=>{assert.equal(url,'/diagnostics');return response(diagnostics('recording'));});
await flush();assert.equal(posts(recording).length,0);assert(recording.el('record').disabled);assert.match(recording.el('record-state').textContent,/Recording/);
const reopened=harness(inline(bench),bench,({url})=>{
  if(url==='/diagnostics')return response(diagnostics('finished'));
  if(url==='/recording.csv?session_id=7')return response('elapsed_ms,raw_ticks,valid,median20_ticks\n10,12000,1,\n20,13000,1,\n');
  if(url==='/record')return response('Could not start recording',503);
  throw Error('Unexpected request '+url);
});
await flush();assert.equal(posts(reopened).length,0);assert.equal(reopened.el('download').hidden,false);assert.equal(reopened.el('download').href,'/recording.csv?session_id=7');assert.match(reopened.el('chart-note').textContent,/All 2 acquired samples/);
reopened.el('record').dispatch('click');await flush();assert.equal(reopened.el('action-error').hidden,false);assert.match(reopened.el('action-error').textContent,/Could not start recording/);
assert.equal(posts(reopened).length,1);
console.log('PASS: recorder reconnect is read-only, recovers the completed session/CSV, and displays command failures');

// Native forms remain deliberate user actions; normal navigation does not POST.
assert.match(html,/action="\/controller\/pause" method="POST"/);
assert.match(bench,/action="\/controller\/resume" method="post"/);
assert.match(bench,/Finishing a recording leaves control paused/);
console.log('PASS: deliberate controller/bench mode forms and recorder pause notice retained');
