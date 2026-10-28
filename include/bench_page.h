#pragma once
#include <Arduino.h>

const char kBenchPage[] PROGMEM = R"HFHTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hydrofoil bench recorder</title>
<style>
:root{color-scheme:light;--ink:#182b3b;--muted:#526575;--line:#cbd6de;--blue:#1767b3;--amber:#a65e00;--red:#a92323}
*{box-sizing:border-box}body{margin:0;background:#eff3f6;color:var(--ink);font:16px/1.5 system-ui,-apple-system,sans-serif}main{max-width:900px;margin:auto;padding:20px 16px 40px}h1{font-size:1.65rem;margin:0 0 4px}h2{font-size:1.15rem;margin:0 0 14px}p{margin:8px 0}a{color:#07569b;text-underline-offset:3px}button,a.download{font:inherit;font-weight:650;min-height:48px;padding:11px 16px;border:1px solid #8095a5;border-radius:9px;background:white;color:var(--ink);cursor:pointer;text-align:center}button:disabled{opacity:.48;cursor:default}button:focus-visible,a:focus-visible,summary:focus-visible{outline:3px solid #e29a29;outline-offset:3px}.primary{background:var(--blue);border-color:var(--blue);color:white;flex:2}.stop{flex:1}.controls{display:flex;gap:10px;flex-wrap:wrap}.card{background:#fff;border:1px solid #dbe3e9;border-radius:13px;padding:18px;margin-top:16px}.subtle{color:var(--muted);font-size:.9rem}.notice{padding:10px 12px;border-left:4px solid #d59425;background:#fff8e9}.connection{font-size:.9rem;min-height:1.5em;margin-top:10px}.error{color:var(--red);font-weight:600}.status{font-weight:650;margin:13px 0 5px}.progress{height:8px;background:#e4ebf1;border-radius:6px;overflow:hidden}.progress span{display:block;background:var(--blue);height:100%;width:0;transition:width .2s}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:17px 0}.metric span{display:block;color:var(--muted);font-size:.8rem}.metric strong{font-size:1.25rem;font-variant-numeric:tabular-nums}table{border-collapse:collapse;width:100%;font-size:.92rem;font-variant-numeric:tabular-nums}th,td{padding:8px 5px;text-align:right;border-bottom:1px solid #e4eaf0}th:first-child,td:first-child{text-align:left}thead th{font-weight:650}tbody th{font-weight:400}.raw{color:var(--blue)}.filtered{color:var(--amber)}.legend{display:flex;flex-wrap:wrap;gap:8px 20px;font-size:.88rem;margin:8px 0}.swatch{display:inline-block;width:18px;height:3px;vertical-align:middle;margin-right:6px}.raw .swatch{background:var(--blue)}.filtered .swatch{background:var(--amber)}.timeout .swatch{background:var(--red);height:9px;width:2px;margin-left:8px;margin-right:14px}.chart{width:100%;height:280px;display:block}a.download{display:inline-block;text-decoration:none;margin-top:10px}a[hidden],[hidden]{display:none!important}details>summary{cursor:pointer;font-weight:600}details[open]>summary{margin-bottom:12px}.mode-buttons button{font-size:.9rem;flex:1 1 180px}.links{display:flex;gap:8px 18px;flex-wrap:wrap;font-size:.9rem}pre{overflow:auto;max-height:420px;font-size:12px;line-height:1.4;background:#f2f5f7;padding:12px;border-radius:6px}.live{font-variant-numeric:tabular-nums}.live strong{font-size:1.2rem}.nojs{color:var(--red)}@media(max-width:520px){main{padding:16px 12px 30px}.card{padding:15px}.metrics{grid-template-columns:repeat(2,1fr)}.controls{flex-direction:column}.chart{height:245px}h1{font-size:1.45rem}}
</style>
</head>
<body><main>
<h1><strong id="recorder-board">—</strong> · Recorder</h1><p><a href="/">Controller</a></p><form action="/controller/resume" method="post"><button type="submit">Resume normal controller</button></form><p id="control-mode">Checking controller mode…</p>
<p class="subtle">Starting any bench action pauses control and disables servo output. Finishing a recording leaves control paused. Keep the servo unplugged during bench work.</p>
<div id="connection" class="connection" role="status">Connecting to the board...</div>
<noscript><p class="nojs">This page needs JavaScript to show recordings. The basic sensing controls and Diagnostics link below still work.</p></noscript>
<section class="card" aria-labelledby="record-title">
<h2 id="record-title">Capture the full pattern</h2>
<p>Hold the sensor depth and wire arrangement steady, then record. The board saves every sample for up to 30 seconds.</p>
<p class="notice subtle">A new recording replaces the previous one. Download any result you want to keep first. Recordings are held in memory and lost when power is removed.</p>
<div class="controls" style="margin-top:14px">
<button id="record" class="primary" type="button" disabled>Record 30 seconds</button>
<form id="stop-form" action="/mode" method="post" class="stop"><button id="stop" style="width:100%" name="mode" value="off">Stop recording / sensing</button></form>
</div>
<p id="action-error" class="error" role="alert" hidden></p>
<p id="record-error" class="error" role="alert" hidden></p>
<p id="record-state" class="status" aria-live="polite">No recording loaded</p>
<div class="progress" aria-hidden="true"><span id="progress"></span></div>
<p class="subtle">Recording continues on the board if this page closes or your phone disconnects. Return to this page to see the result.</p>
<div class="metrics">
<div class="metric"><span>Elapsed</span><strong id="elapsed">—</strong></div>
<div class="metric"><span>Samples recorded</span><strong id="samples">—</strong></div>
<div class="metric"><span>Successful</span><strong id="good">—</strong></div>
<div class="metric"><span>Timeouts</span><strong id="timeouts">—</strong></div>
</div>
<table aria-label="Recording statistics in timer ticks">
<thead><tr><th scope="col">Ticks</th><th scope="col" class="raw">Raw</th><th scope="col" class="filtered">Median of 20</th></tr></thead>
<tbody>
<tr><th scope="row">Values included</th><td id="raw-count">—</td><td id="filtered-count">—</td></tr>
<tr><th scope="row">Lowest</th><td id="raw-min">—</td><td id="filtered-min">—</td></tr>
<tr><th scope="row">Highest</th><td id="raw-max">—</td><td id="filtered-max">—</td></tr>
<tr><th scope="row">Range</th><td id="raw-span">—</td><td id="filtered-span">—</td></tr>
<tr><th scope="row">Mean</th><td id="raw-mean">—</td><td id="filtered-mean">—</td></tr>
<tr><th scope="row">Standard deviation</th><td id="raw-stddev">—</td><td id="filtered-stddev">—</td></tr>
</tbody></table>
<p class="subtle">Statistics exclude timeouts. The median trace starts only after its 20-sample window is ready.</p>
</section>
<section class="card" aria-labelledby="pattern-title">
<h2 id="pattern-title">Recorded pattern</h2>
<div class="legend"><span class="raw"><i class="swatch"></i>Raw samples</span><span class="filtered"><i class="swatch"></i>Median of 20</span><span class="timeout"><i class="swatch"></i>Timeout</span></div>
<canvas id="chart" class="chart" role="img" aria-label="Recorded raw and median sensor readings versus elapsed time"></canvas>
<p id="chart-note" class="subtle">The complete chart appears when a recording ends.</p>
<a id="download" class="download" href="/recording.csv" download="hydrofoil-recording.csv" hidden>Download every sample (CSV)</a>
<p class="subtle">Target rate: 100 samples per second; the chart and CSV use actual elapsed times. A 20-sample median spans about 200 ms here. This does not verify the normal 500 Hz controller, where that window is about 40 ms.</p>
</section>
<details class="card">
<summary>Other sensing checks</summary>
<p class="live">Latest raw reading: <strong id="live-ticks">—</strong> ticks</p>
<p id="live-state" class="subtle">Waiting for board status.</p>
<form id="mode-form" action="/mode" method="post" class="controls mode-buttons">
<button name="mode" value="single">One real sample</button>
<button name="mode" value="auto">Automatic real samples (100 Hz)</button>
<button name="mode" value="timeout">Simulate repeated timeouts (100 Hz)</button>
</form>
<p class="subtle">These checks do not create a recording. The timeout test ignores HIGH in software; do not short any wires. Stop a recording before starting another sensing check.</p>
</details>
<details class="card">
<summary>Board details</summary>
<nav class="links"><a href="/diagnostics">Diagnostics</a><a href="/settings/configs.json">Controller settings</a><a href="/settings/addresses.json">Saved board addresses</a></nav>
<p class="subtle">ArduinoOTA is available on port 3232 at this board's numeric IP. Upload application firmware only, never a filesystem image. Existing settings are preserved.</p>
<pre id="diagnostics">Waiting for board status.</pre>
</details>
</main>
<script>
'use strict';
const byId=id=>document.getElementById(id);
let latest=null, chartRows=[], chartKey=null, loadedKey=null, failedKey=null;
let pollTimer=null, polling=false, refreshRequested=false, acting=false, actionVersion=0;
let connected=false;
const n=(value,digits=0)=>value===null||value===undefined||!Number.isFinite(Number(value))?'—':Number(value).toLocaleString(undefined,{maximumFractionDigits:digits});
function message(id,text,error=false){const el=byId(id);el.textContent=text;el.classList.toggle('error',error);}
function setControls(){const active=latest&&latest.recording&&latest.recording.state==='recording';byId('record').disabled=!connected||acting||active;byId('record').textContent=active?'Recording...':'Record 30 seconds';byId('stop').disabled=acting;document.querySelectorAll('#mode-form button').forEach(b=>{b.disabled=acting||active;});}
function render(d){
  if(!d.recording||!['idle','recording','finished'].includes(d.recording.state))throw new Error('This firmware did not provide recorder status.');
  const r=d.recording, key=String(r.session_id);latest=d;
  byId('record-error').hidden=!r.error;
  byId('record-error').textContent=r.error?'Recording error: '+r.error:'';
  byId('diagnostics').textContent=JSON.stringify(d,null,2);
  byId('live-ticks').textContent=n(d.last_raw_ticks);
  byId('live-state').textContent=(d.automatic_sensing?(d.simulated_timeout?'Simulated timeouts running.':'Automatic real samples running.'):'Automatic sensing off.')+' Successful samples since boot: '+n(d.sensor_good_samples)+'. Timeouts since boot: '+n(d.sensor_timeouts)+'.';
  const reason={completed:'Completed',stopped:'Stopped early',replaced:'Replaced',ota:'Stopped for firmware update',full:'Sample capacity reached',none:'Finished'};
  let state=r.state==='idle'?'Ready to record':r.state==='recording'?'Recording · '+(r.elapsed_ms/1000).toFixed(1)+' / '+(r.duration_ms/1000).toFixed(0)+' seconds':(reason[r.reason]||'Finished')+' · recording '+key;
  byId('record-state').textContent=state;
  byId('progress').style.width=(r.state==='idle'?0:Math.min(100,100*r.elapsed_ms/r.duration_ms))+'%';
  byId('elapsed').textContent=r.state==='idle'?'—':(r.elapsed_ms/1000).toFixed(2)+' s';
  for(const field of ['samples','good','timeouts'])byId(field).textContent=r.state==='idle'?'—':n(r[field]);
  for(const kind of ['raw','filtered'])for(const field of ['count','min','max','span','mean','stddev'])byId(kind+'-'+field).textContent=n(r[kind]&&r[kind][field],field==='mean'||field==='stddev'?1:0);
  const downloadable=r.state==='finished';
  byId('download').hidden=!downloadable;
  byId('download').href='/recording.csv?session_id='+encodeURIComponent(key);
  byId('download').download='hydrofoil-recording-'+key+'.csv';
  if(r.state!=='finished'){loadedKey=null;failedKey=null;}
  if(r.state!=='finished'||chartKey!==key){chartRows=[];chartKey=key;drawChart();}
  if(r.state==='idle')message('chart-note','The complete chart appears when a recording ends.');
  if(r.state==='recording')message('chart-note','Recording on the board. The complete trace will load after it stops.');
  setControls();
}
function parseCsv(body,expected){
  const lines=body.trim().split(/\r?\n/);
  if(lines.shift()!=='elapsed_ms,raw_ticks,valid,median20_ticks')throw new Error('Unexpected CSV format.');
  const rows=lines.filter(line=>line.length).map(line=>{
    const f=line.split(',');
    if(f.length!==4||!/^\d+$/.test(f[0])||!/^\d+$/.test(f[1])||!['0','1'].includes(f[2])||(f[3]!==''&&!/^\d+(?:\.\d+)?$/.test(f[3])))throw new Error('An invalid sample was found in the CSV.');
    return {t:Number(f[0]),raw:Number(f[1]),valid:f[2]==='1',median:f[3]===''?null:Number(f[3])};
  });
  if(rows.length!==expected)throw new Error('Sample count changed while loading. Refresh to retry.');
  for(let i=1;i<rows.length;i++)if(rows[i].t<rows[i-1].t)throw new Error('Sample times are out of order.');
  return rows;
}
async function loadChart(r){
  const key=String(r.session_id);
  if(r.state!=='finished'||loadedKey===key||failedKey===key)return;
  const version=actionVersion;message('chart-note','Loading the complete recorded trace...');
  try{
    const response=await fetch('/recording.csv?session_id='+encodeURIComponent(key),{cache:'no-store'});
    if(!response.ok)throw new Error('The board returned '+response.status+'.');
    const rows=parseCsv(await response.text(),r.samples);
    if(version!==actionVersion||!latest||latest.recording.state!=='finished'||String(latest.recording.session_id)!==key)return;
    chartRows=rows;chartKey=key;loadedKey=key;drawChart();
    message('chart-note',rows.length?'All '+n(rows.length)+' acquired samples are shown. Red marks and line gaps indicate timeouts.':'This recording ended before any samples were captured.');
  }catch(error){
    if(version!==actionVersion)return;
    failedKey=key;message('chart-note','Could not load the trace: '+error.message+' Download the CSV or refresh this page to retry.',true);
  }
}
async function poll(){
  if(polling){refreshRequested=true;return;}
  clearTimeout(pollTimer);polling=true;
  try{
    const response=await fetch('/diagnostics',{cache:'no-store'});
    if(!response.ok)throw new Error('Board response '+response.status);
    const data=await response.json();render(data);connected=true;byId("control-mode").textContent=data.controller_mode==="center-test"?"Alignment test — fixed servo pulse active; a bench action stops the test":data.controller_mode==="bench"?"Bench mode — servo output off":"Normal controller active — a bench action will pause it";
    byId('recorder-board').textContent=(data.ssid||'—').slice(-2)+' · '+(data.slave?'Slave':'Master');message('connection','Connected · '+data.build);setControls();
    await loadChart(data.recording);
  }catch(error){connected=false;message('connection','Connection unavailable. Displayed values may be old; an active recording continues on the board. '+error.message,true);setControls();}
  finally{polling=false;pollTimer=setTimeout(poll,refreshRequested?50:1000);refreshRequested=false;}
}
async function command(path,body){
  if(acting)return;acting=true;actionVersion++;setControls();byId('action-error').hidden=true;
  try{
    const options={method:'POST',cache:'no-store'};
    if(body){options.headers={'Content-Type':'application/x-www-form-urlencoded'};options.body=body;}
    const response=await fetch(path,options);
    if(!response.ok){const detail=(await response.text()).slice(0,180);throw new Error(detail||'Board response '+response.status);}
    if(path==='/record'){loadedKey=null;failedKey=null;chartRows=[];drawChart();message('chart-note','Recording requested. Waiting for board status...');}
  }catch(error){byId('action-error').hidden=false;message('action-error','Command was not confirmed: '+error.message+' Check the board status before retrying.',true);}
  finally{acting=false;setControls();poll();}
}
byId('record').addEventListener('click',()=>command('/record'));
byId('stop-form').addEventListener('submit',event=>{event.preventDefault();command('/mode','mode=off');});
byId('mode-form').addEventListener('submit',event=>{if(!event.submitter)return;event.preventDefault();command('/mode','mode='+encodeURIComponent(event.submitter.value));});
function drawChart(){
  const canvas=byId('chart'), width=canvas.clientWidth, height=canvas.clientHeight, scale=Math.min(window.devicePixelRatio||1,3);
  if(!width||!height)return;canvas.width=Math.round(width*scale);canvas.height=Math.round(height*scale);
  const ctx=canvas.getContext('2d');ctx.scale(scale,scale);ctx.clearRect(0,0,width,height);
  const left=58,right=14,top=23,bottom=40,w=width-left-right,h=height-top-bottom;
  let lo=Infinity,hi=-Infinity;
  for(const row of chartRows)if(row.valid){lo=Math.min(lo,row.raw);hi=Math.max(hi,row.raw);if(row.median!==null){lo=Math.min(lo,row.median);hi=Math.max(hi,row.median);}}
  const hasValues=Number.isFinite(lo),delta=hasValues?Math.max(1,(hi-lo)*.08):1;
  if(hasValues){lo=Math.max(0,lo-delta);hi+=delta;}else{lo=0;hi=1;}
  const elapsed=latest&&latest.recording?Number(latest.recording.elapsed_ms):0;
  const end=Math.max(1,elapsed,chartRows.length?chartRows[chartRows.length-1].t:0)/1000;
  const x=t=>left+(t/1000/end)*w,y=v=>top+h-(v-lo)/(hi-lo)*h;
  ctx.font='11px system-ui, sans-serif';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){
    const yy=top+h-i*h/4;ctx.strokeStyle='#e1e8ee';ctx.beginPath();ctx.moveTo(left,yy);ctx.lineTo(width-right,yy);ctx.stroke();
    ctx.fillStyle='#526575';ctx.textAlign='right';ctx.fillText(hasValues?n(lo+(hi-lo)*i/4):'—',left-7,yy+4);
    const xx=left+i*w/4;ctx.textAlign='center';ctx.fillText((end*i/4).toFixed(end<10?1:0),xx,height-bottom+18);
  }
  ctx.fillStyle='#526575';ctx.textAlign='left';ctx.fillText('Ticks',left,12);ctx.textAlign='center';ctx.fillText('Elapsed seconds',left+w/2,height-4);
  if(!chartRows.length){ctx.fillStyle='#70808d';ctx.fillText('No completed trace loaded',left+w/2,top+h/2);return;}
  ctx.save();ctx.beginPath();ctx.rect(left,top,w,h);ctx.clip();
  for(const row of chartRows)if(!row.valid){const xx=x(row.t);ctx.strokeStyle='#a92323';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(xx,top);ctx.lineTo(xx,top+10);ctx.stroke();}
  function trace(field,color,lineWidth){ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.lineJoin='round';ctx.beginPath();let connectedPoint=false;for(const row of chartRows){const value=row[field];if(!row.valid||value===null){connectedPoint=false;continue;}const xx=x(row.t),yy=y(value);if(connectedPoint)ctx.lineTo(xx,yy);else ctx.moveTo(xx,yy);connectedPoint=true;}ctx.stroke();}
  trace('raw','#1767b3',1);trace('median','#a65e00',1.6);
  const validRows=chartRows.filter(row=>row.valid);if(validRows.length===1){ctx.fillStyle='#1767b3';ctx.beginPath();ctx.arc(x(validRows[0].t),y(validRows[0].raw),3,0,2*Math.PI);ctx.fill();}
  ctx.restore();
  if(!hasValues){ctx.fillStyle='#a92323';ctx.textAlign='center';ctx.fillText('No successful readings in this recording',left+w/2,top+h/2);}
}
window.addEventListener('resize',drawChart);
drawChart();poll();
</script></body></html>)HFHTML";
