#ifndef WEBAPP_HTML_H
#define WEBAPP_HTML_H
#include <Arduino.h>
// Auto-generated from control_app.html - do not hand edit.
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>RoboCap TEST Console</title>
<style>
  :root{--bg:#0e1116;--pan:#171c24;--ln:#2a323d;--tx:#e6edf3;--mut:#8b97a6;
        --acc:#3da9fc;--ok:#2ecc71;--warn:#f1c40f;--bad:#e74c3c;--kick:#e67e22}
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  body{margin:0;background:var(--bg);color:var(--tx);font-family:'Segoe UI',Roboto,Arial,sans-serif;
       max-width:540px;margin:0 auto;padding:8px;user-select:none}
  h1{font-size:16px;margin:2px 0 8px}
  .pan{background:var(--pan);border:1px solid var(--ln);border-radius:10px;padding:10px;margin-bottom:8px}
  .row{display:flex;gap:8px;align-items:center}
  .grow{flex:1}
  button{font:inherit;color:var(--tx);background:#222a35;border:1px solid var(--ln);
         border-radius:9px;padding:11px;font-weight:600}
  button:active{filter:brightness(1.35)} button:disabled{opacity:.35}
  select{font:inherit;color:var(--tx);background:#222a35;border:1px solid var(--ln);border-radius:9px;padding:10px;flex:1}
  .acc{background:var(--acc);border-color:var(--acc);color:#06121f}
  .kick{background:var(--kick);border-color:var(--kick);color:#1f0f02}
  .stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:800}
  .tog.on{background:var(--ok);border-color:var(--ok);color:#06210f}
  #bar{display:flex;align-items:center;gap:8px;font-size:13px}
  .dot{width:10px;height:10px;border-radius:50%;background:var(--bad);flex:none}
  .dot.on{background:var(--ok)}
  .pill{padding:2px 9px;border-radius:14px;font-size:11px;font-weight:700;background:#222a35}
  .pill.TEST{background:#143a2a;color:var(--ok)} .pill.GAME{background:#3a2a14;color:var(--warn)}
  .pill.READY{background:#1c2530;color:var(--mut)}
  .ab{display:flex;gap:4px;background:#11161d;border-radius:9px;padding:3px}
  .ab button{flex:1;padding:7px;border:none;background:transparent;font-size:13px}
  .ab button.sel{background:var(--acc);color:#06121f;border-radius:7px}
  #lock{display:none;background:#3a1414;border:1px solid var(--bad);color:#ffd7d2;
        border-radius:9px;padding:9px;font-size:12px;text-align:center;margin-bottom:8px}
  #lock.show{display:block}
  #tabs{display:flex;flex-wrap:wrap;gap:4px;margin-bottom:8px}
  #tabs button{padding:8px 10px;font-size:12px;flex:none}
  #tabs button.sel{background:var(--acc);color:#06121f;border-color:var(--acc)}
  #tabs button.dbg{border-color:var(--kick);color:var(--kick)}
  #tabs button.dbg.sel{background:var(--kick);color:#1f0f02;border-color:var(--kick)}
  #con{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:11px;line-height:1.5;
       height:300px;overflow:auto;background:#0b0e13;border:1px solid var(--ln);
       border-radius:8px;padding:6px;white-space:pre-wrap;word-break:break-word}
  #con .cl{color:var(--mut)} #con .ca{color:var(--acc);font-weight:700}
  #con .cb{color:var(--kick);font-weight:700} #con .cs{color:var(--ok)} #con .ce{color:var(--bad)}
  .tab{display:none} .tab.show{display:block}
  label{font-size:12px;color:var(--mut);display:block;margin:6px 0 4px}
  .val{color:var(--acc);font-weight:700}
  input[type=range]{width:100%}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:7px}
  .cell{background:#11161d;border:1px solid var(--ln);border-radius:8px;padding:7px}
  .cell .k{font-size:10px;color:var(--mut)} .cell .v{font-size:16px;font-weight:700;margin-top:2px}
  .v.good{color:var(--ok)} .v.bad{color:var(--mut)}
  .seg{display:flex;gap:6px} .seg button{flex:1}
  .hint{font-size:11px;color:var(--mut);margin-top:6px;line-height:1.5}
  #pad{position:relative;width:170px;height:170px;border-radius:50%;margin:6px auto;
       background:radial-gradient(circle,#1d2530,#11161d 70%);border:1px solid var(--ln);touch-action:none}
  #knob{position:absolute;width:54px;height:54px;border-radius:50%;background:var(--acc);left:58px;top:58px}
  svg{display:block;margin:0 auto}
  #estop{position:sticky;bottom:8px;width:100%;margin-top:8px;font-size:16px}
</style>
</head>
<body>
  <h1>RoboCap &mdash; TEST Console</h1>

  <div class="pan">
    <div id="bar">
      <span class="dot" id="dot"></span><span id="conn">Disconnected</span>
      <span class="pill READY" id="pill" style="margin-left:auto">--</span>
    </div>
    <div class="ab" style="margin-top:8px">
      <button id="abA" class="sel" onclick="setTarget('A')">Robot A (this)</button>
      <button id="abB" onclick="setTarget('B')">Robot B (relay)</button>
    </div>
  </div>

  <div id="lock">&#9888; Robot is in GAME mode &mdash; TEST controls are locked.</div>

  <div class="row" style="margin-bottom:8px">
    <button class="grow acc" id="btnEnter" onclick="cmd('enter_test')">Enter TEST</button>
    <button class="grow" id="btnExit" onclick="cmd('exit_test')">Exit to READY</button>
  </div>

  <div id="tabs"></div>

  <div class="tab" data-t="Status">
    <div class="pan"><div class="grid">
      <div class="cell"><div class="k">Battery</div><div class="v" id="sBatt">--</div></div>
      <div class="cell"><div class="k">Uptime</div><div class="v" id="sUp">--</div></div>
      <div class="cell"><div class="k">ESP IDs</div><div class="v" id="sIds">--</div></div>
      <div class="cell"><div class="k">ESP-NOW</div><div class="v" id="sNow">--</div></div>
      <div class="cell"><div class="k">Partner ID</div><div class="v" id="sPid">--</div></div>
      <div class="cell"><div class="k">Partner MAC</div><div class="v" id="sMac" style="font-size:11px">--</div></div>
    </div>
    <button style="width:100%;margin-top:8px" onclick="cmd('query_status')">Refresh</button>
    </div>
  </div>

  <div class="tab" data-t="Motors">
    <div class="pan">
      <label>Single motor test</label>
      <div class="seg">
        <select id="mN"><option value="1">M1</option><option value="2">M2</option>
          <option value="3">M3</option><option value="4">M4</option></select>
        <select id="mD"><option value="1">FWD</option><option value="0">REV</option></select>
      </div>
      <label>PWM: <span class="val" id="mPv">50</span>%</label>
      <input type="range" id="mP" min="0" max="100" value="50">
      <button style="width:100%;margin-top:6px" onclick="confirmCmd('motor','n='+v('mN')+'&dir='+v('mD')+'&pwm='+v('mP'),'Run motor '+v('mN')+'?')">Run motor</button>
    </div>
    <div class="pan">
      <label>Omni drive (joystick + rotation)</label>
      <div id="pad"><div id="knob"></div></div>
      <label>Rotation: <span class="val" id="rotV">0</span></label>
      <input type="range" id="rot" min="-100" max="100" value="0">
      <button class="stop" style="width:100%;margin-top:6px" onclick="cmd('stop')">STOP MOTORS</button>
    </div>
  </div>

  <div class="tab" data-t="Kicker">
    <div class="pan">
      <label>Kicker power test (Appendix C)</label>
      <div class="seg" style="flex-wrap:wrap">
        <button class="kick" onclick="confirmCmd('kick','power=30','Kick at 30%?')">30%</button>
        <button class="kick" onclick="confirmCmd('kick','power=50','Kick at 50%?')">50%</button>
        <button class="kick" onclick="confirmCmd('kick','power=70','Kick at 70%?')">70%</button>
        <button class="kick" onclick="confirmCmd('kick','power=100','Kick at 100%?')">100%</button>
      </div>
      <div class="hint">Place robot in goal touching back wall; ball must NOT return after bouncing off the far goal.</div>
    </div>
    <div class="pan">
      <label>Dribbler speed: <span class="val" id="dribV">0</span>%</label>
      <input type="range" id="drib" min="0" max="100" value="0">
      <div class="hint">0 = off. There is a usable minimum &mdash; below it the motor stalls (no torque on the ball).</div>
    </div>
  </div>

  <div class="tab" data-t="Vision">
    <div class="pan">
      <label>Live detections (4 cameras)</label>
      <div class="grid" id="visGrid"></div>
      <div class="hint">Angle to yellow / blue goal, white / black line per camera. &minus;1 = not seen.</div>
    </div>
  </div>

  <div class="tab" data-t="HSV">
    <div class="pan">
      <label>Camera / color</label>
      <div class="seg">
        <select id="hCam"><option value="1">Front</option><option value="2">Right</option>
          <option value="3">Rear</option><option value="4">Left</option></select>
        <select id="hCol" onchange="hsvShow()">
          <option value="YELLOW">Yellow</option><option value="BLUE">Blue</option>
          <option value="WHITE">White</option><option value="BLACK">Black</option></select>
      </div>
      <div id="hsvSliders"></div>
      <div class="seg" style="margin-top:8px">
        <button class="acc" onclick="hsvSend()">Apply</button>
        <button onclick="cmd('cal',{raw:'SAVE'})">Save</button>
        <button onclick="cmd('cal',{raw:'LOAD'})">Load</button>
        <button onclick="cmd('cal',{raw:'RESET'})">Reset</button>
      </div>
    </div>
  </div>

  <div class="tab" data-t="IR">
    <div class="pan">
      <label>IR ring (20 sensors) + U1 pocket</label>
      <svg id="irSvg" viewBox="-110 -110 220 220" width="220" height="220"></svg>
      <div class="grid">
        <div class="cell"><div class="k">Strongest dir</div><div class="v" id="irDir">--</div></div>
        <div class="cell"><div class="k">Ball in pocket (U1)</div><div class="v bad" id="irU1">NO</div></div>
      </div>
      <button style="width:100%;margin-top:6px" onclick="cmd('ir_raw')">Read IR</button>
    </div>
  </div>

  <div class="tab" data-t="Compass">
    <div class="pan" style="text-align:center">
      <svg id="cmpSvg" viewBox="-60 -60 120 120" width="150" height="150"></svg>
      <div class="v" id="cmpVal" style="font-size:22px">--&deg;</div>
      <div class="seg" style="margin-top:8px">
        <button onclick="cmd('compass_read')">Read</button>
        <button class="acc" onclick="cmd('compass_cal_start')">Cal start</button>
        <button onclick="cmd('compass_cal_stop')">Cal save</button>
      </div>
      <div class="hint">Cal = rotate the robot a full 360&deg; slowly, then Cal save.</div>
    </div>
  </div>

  <div class="tab" data-t="GoalLock">
    <div class="pan">
      <label>Opponent goal lock (sets 0&deg; = opponent goal)</label>
      <div class="seg">
        <button onclick="confirmCmd('goal_lock','color=yellow','Lock onto YELLOW goal?')">Yellow goal</button>
        <button onclick="confirmCmd('goal_lock','color=blue','Lock onto BLUE goal?')">Blue goal</button>
      </div>
      <div class="grid" style="margin-top:8px">
        <div class="cell"><div class="k">Goal bearing</div><div class="v bad" id="glBear">--</div></div>
        <div class="cell"><div class="k">Saved offset</div><div class="v" id="glOff">--</div></div>
      </div>
      <div class="hint">Robot rotates ~30&deg;/s until the goal is centred (|angle|&lt;5&deg;), then saves COMPASS_GOAL_OFFSET. Run once before each game.</div>
    </div>
  </div>

  <div class="tab" data-t="Partner">
    <div class="pan">
      <label>Inter-robot link (ESP-NOW)</label>
      <div class="grid">
        <div class="cell"><div class="k">Paired</div><div class="v" id="pPair">--</div></div>
        <div class="cell"><div class="k">RTT (ping)</div><div class="v" id="pRtt">--</div></div>
      </div>
      <div class="seg" style="margin-top:8px">
        <button onclick="cmd('ping_partner')">Ping B</button>
        <button class="acc" onclick="confirmCmd('pass_test','','Run A-to-B pass test?')">Pass test A&rarr;B</button>
      </div>
      <div class="hint">Select &ldquo;Robot B&rdquo; at the top to send any tab&rsquo;s action to the partner via relay.</div>
    </div>
  </div>

  <div class="tab" data-t="Console">
    <div class="pan">
      <label>Teensy state machine &amp; messages (live)</label>
      <div class="seg" style="margin-bottom:6px">
        <button id="cfAll" class="sel" onclick="conFilter('all')">All</button>
        <button id="cfA" onclick="conFilter('A')">A only</button>
        <button id="cfB" onclick="conFilter('B')">B only</button>
        <button id="cfState" class="tog" onclick="conToggleState()">STATE only</button>
      </div>
      <div id="con"></div>
      <div class="seg" style="margin-top:6px">
        <label class="grow" style="margin:0;display:flex;align-items:center;gap:6px">
          <input type="checkbox" id="cAuto" checked> auto-scroll</label>
        <button onclick="conClear()">Clear</button>
      </div>
      <div class="hint">A = this robot, B = partner (via ESP-NOW relay). &uarr; marks a state transition.</div>
    </div>
  </div>

  <button class="stop" id="estop" onclick="estop()">&#9632; EMERGENCY STOP</button>

  <div class="pan"><label>Log</label><div id="log" style="font-size:11px;color:var(--mut);max-height:120px;overflow:auto"></div></div>

<script>
'use strict';
const TABS=['Status','Motors','Kicker','Vision','HSV','IR','Compass','GoalLock','Partner','Console'];
let target='A', inTest=false, curTab='Status';
const $=id=>document.getElementById(id);
const v=id=>$(id).value;

function cmd(op,params){
  let q='target='+target+'&op='+op;
  if(typeof params==='string'&&params) q+='&'+params;
  else if(params&&typeof params==='object') for(const k in params) q+='&'+k+'='+encodeURIComponent(params[k]);
  fetch('/cmd?'+q).catch(()=>{});
  logLine('> '+op+(target==='B'?' [B]':''));
}
function confirmCmd(op,params,msg){ if(confirm(msg)) cmd(op,params); }
function estop(){ cmd('estop'); dribStop(); }
function dribStop(){ $('drib').value=0; $('dribV').textContent='0'; }
function setTarget(t){ target=t; $('abA').classList.toggle('sel',t==='A'); $('abB').classList.toggle('sel',t==='B'); }

function buildTabs(){
  const bar=$('tabs');
  TABS.forEach(t=>{ const b=document.createElement('button'); b.textContent=t; b.id='tab_'+t;
    if(t==='Console') b.classList.add('dbg');
    b.onclick=()=>showTab(t); bar.appendChild(b); });
  showTab('Status');
}
function showTab(t){ curTab=t;
  document.querySelectorAll('.tab').forEach(e=>e.classList.toggle('show',e.dataset.t===t));
  TABS.forEach(x=>$('tab_'+x).classList.toggle('sel',x===t));
  if(t==='Console'&&$('cAuto').checked){const c=$('con');if(c)c.scrollTop=c.scrollHeight;}
}
function logLine(s){ const l=$('log'); l.innerHTML=(new Date().toLocaleTimeString()+' '+s+'<br>')+l.innerHTML;
  if(l.innerHTML.length>4000) l.innerHTML=l.innerHTML.slice(0,4000); }

let conFlt='all', conStateOnly=false;
function escH(s){ return s.replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c])); }
function conVis(row){ const okSrc=(conFlt==='all'||row.dataset.src===conFlt);
  const okType=(!conStateOnly||row.dataset.ty==='STATE'); row.style.display=(okSrc&&okType)?'':'none'; }
function conPush(src,type,text){
  const con=$('con'); if(!con) return;
  const row=document.createElement('div'); row.dataset.src=src; row.dataset.ty=type;
  const tcl=(type==='STATE')?'cs':(type==='ERR')?'ce':'';
  row.innerHTML='<span class="cl">'+new Date().toLocaleTimeString()+'</span> '+
    '<span class="'+(src==='B'?'cb':'ca')+'">['+src+(type==='STATE'?'\u2191':'')+']</span> '+
    '<span class="'+tcl+'">'+escH(type+': '+text)+'</span>';
  conVis(row); con.appendChild(row);
  while(con.children.length>600) con.removeChild(con.firstChild);
  if($('cAuto').checked) con.scrollTop=con.scrollHeight;
}
function conFilter(f){ conFlt=f;
  $('cfAll').classList.toggle('sel',f==='all'); $('cfA').classList.toggle('sel',f==='A');
  $('cfB').classList.toggle('sel',f==='B'); [...$('con').children].forEach(conVis); }
function conToggleState(){ conStateOnly=!conStateOnly; $('cfState').classList.toggle('on',conStateOnly);
  [...$('con').children].forEach(conVis); }
function conClear(){ $('con').innerHTML=''; }

function connect(){
  const es=new EventSource('/events');
  es.onopen=()=>{ $('dot').classList.add('on'); $('conn').textContent='Connected'; };
  es.onerror=()=>{ $('dot').classList.remove('on'); $('conn').textContent='Disconnected'; };
  es.addEventListener('tlm', e=>route(e.data,'A'));
  es.addEventListener('tlmB',e=>route(e.data,'B'));
}
function route(line,src){
  const c=line.indexOf(':'); if(c<0) return;
  const type=line.slice(0,c), d=line.slice(c+1).split(',');
  if(type==='TLM') applyTlm(d,src);
  else if(type==='IR') applyIr(d);
  else if(type==='VIS') applyVis(d);
  else if(type==='CMP') applyCmp(d);
  else if(type==='STA') applySta(d);
  else if(type==='STATE'||type==='LOG'||type==='ACK'||type==='INFO'||type==='ERR'||type==='DBG'){
    conPush(src,type,line.slice(c+1));
    if(type==='LOG'||type==='ACK') logLine((src==='B'?'[B] ':'')+line.slice(c+1));
  }
}
const STATES=['READY','GAME','TEST'];
function applyTlm(d,src){
  if(src==='A'){
    const st=STATES[+d[0]]||'READY'; inTest=(st==='TEST');
    $('pill').textContent=st; $('pill').className='pill '+st;
    $('lock').classList.toggle('show',st==='GAME');
    $('btnEnter').disabled=(st!=='READY');
  }
  $('sBatt').textContent=d[1]+'%';
  setCell('glBear', d[8]>=0?d[8]+'\u00b0':'--', d[8]>=0);
}
function applyIr(d){
  const u1=d[20]==='1'; setCell('irU1', u1?'YES':'NO', u1);
  const vals=d.slice(0,20).map(Number), svg=$('irSvg'); svg.innerHTML='';
  let max=Math.max(1,...vals), bi=0,bv=-1;
  vals.forEach((val,i)=>{
    const a=(i/20)*2*Math.PI-Math.PI/2, r=20+80*(val/max);
    const x=r*Math.cos(a), y=r*Math.sin(a);
    svg.innerHTML+='<line x1="0" y1="0" x2="'+x.toFixed(1)+'" y2="'+y.toFixed(1)+'" stroke="#3da9fc" stroke-width="3"/>';
    if(val>bv){bv=val;bi=i;}
  });
  $('irDir').textContent=(bi*18)+'\u00b0';
}
function applyVis(d){
  const cam=+d[0], names=['Front','Right','Rear','Left'];
  let cell=$('vis'+cam);
  if(!cell){ cell=document.createElement('div'); cell.className='cell'; cell.id='vis'+cam; $('visGrid').appendChild(cell); }
  cell.innerHTML='<div class="k">'+(names[cam]||cam)+'</div><div class="v" style="font-size:12px">Y:'+d[1]+' B:'+d[2]+' W:'+d[3]+' K:'+d[4]+'</div>';
}
function applyCmp(d){
  const h=+d[0]; $('cmpVal').innerHTML=h+'&deg;'+(d[1]==='1'?' (cal)':'');
  const a=h*Math.PI/180-Math.PI/2;
  $('cmpSvg').innerHTML='<circle r="50" fill="none" stroke="#2a323d"/><line x1="0" y1="0" x2="'+(45*Math.cos(a)).toFixed(1)+'" y2="'+(45*Math.sin(a)).toFixed(1)+'" stroke="#3da9fc" stroke-width="4"/>';
}
function applySta(d){
  $('sBatt').textContent=d[0]+'%'; $('sIds').textContent=d.slice(1,5).join(' ');
  $('sPid').textContent=d[5]; $('sMac').textContent=d[6]; $('sUp').textContent=d[7]+'s';
  $('sNow').textContent=(d[5]&&d[5]!=='0')?'paired':'scanning';
  $('pPair').textContent=(d[5]&&d[5]!=='0')?'yes':'no';
}
function setCell(id,txt,good){ const e=$(id); if(!e)return; e.textContent=txt;
  e.classList.toggle('good',!!good); e.classList.toggle('bad',!good); }

let padA=false,vx=0,vy=0;
function setKnob(px,py){ $('knob').style.left=(58+px)+'px'; $('knob').style.top=(58-py)+'px'; }
function padMove(ev){ if(!padA)return; const r=$('pad').getBoundingClientRect(),t=ev.touches?ev.touches[0]:ev;
  let dx=t.clientX-r.left-85, dy=85-(t.clientY-r.top), m=Math.hypot(dx,dy);
  if(m>85){dx=dx/m*85;dy=dy/m*85;} setKnob(dx,dy);
  vx=Math.round(dx/85*100); vy=Math.round(dy/85*100); }
function padEnd(){ padA=false; vx=0; vy=0; setKnob(0,0); }

// (dribbler is a slider now — no toggle helpers needed)

const HSV6=['hMin','hMax','sMin','sMax','vMin','vMax'];
function hsvShow(){
  const col=v('hCol'), box=$('hsvSliders'); box.innerHTML='';
  const fields=(col==='WHITE')?['sMax','vMin']:(col==='BLACK')?['sMax','vMax']:HSV6;
  fields.forEach(f=>{ box.innerHTML+='<label>'+f+': <span class="val" id="hv_'+f+'">128</span></label>'+
    '<input type="range" id="h_'+f+'" min="0" max="255" value="128" oninput="document.getElementById(\'hv_'+f+'\').textContent=this.value">'; });
}
function hsvSend(){
  const col=v('hCol'); let vals;
  if(col==='WHITE') vals=[v('h_sMax'),v('h_vMin')];
  else if(col==='BLACK') vals=[v('h_sMax'),v('h_vMax')];
  else vals=HSV6.map(f=>v('h_'+f));
  cmd('cal',{cam:v('hCam'),raw:col+':'+vals.join(',')});
}

setInterval(()=>{ if(!inTest||curTab!=='Motors')return;
  const w=parseInt(v('rot'),10)||0;
  if(vx||vy||w) cmd('omni','vx='+vx+'&vy='+vy+'&r='+w);
},100);

window.addEventListener('load',()=>{
  buildTabs(); hsvShow();
  $('mP').oninput=()=>$('mPv').textContent=$('mP').value;
  $('rot').oninput=()=>$('rotV').textContent=$('rot').value;
  $('rot').addEventListener('pointerup',()=>{$('rot').value=0;$('rotV').textContent='0';});
  $('drib').oninput=()=>$('dribV').textContent=$('drib').value;        // live label
  $('drib').onchange=()=>cmd('dribbler','pct='+$('drib').value);       // send on release
  const pad=$('pad');
  pad.addEventListener('pointerdown',e=>{padA=true;padMove(e);});
  pad.addEventListener('pointermove',padMove);
  window.addEventListener('pointerup',padEnd);
  connect();
});
</script>
</body>
</html>

)rawliteral";
#endif
