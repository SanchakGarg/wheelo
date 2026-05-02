#include "WebUI.h"

// ── HTML ─────────────────────────────────────────────────────────────────────

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<title>Wheelo Control</title>
<style>
:root{color-scheme:dark;
  --bg:#0a0d12;--surface:#111620;--border:#1e2733;
  --blue:#4d9eff;--red:#e05252;--green:#52c87a;--muted:#5a6a80;
  --text:#d0dae8;--text2:#8a9ab0;}
*{box-sizing:border-box;margin:0;padding:0;}
html,body{min-height:100%;background:var(--bg);color:var(--text);
  font-family:'Courier New',Courier,monospace;}
body{display:flex;flex-direction:column;align-items:center;
  padding:24px 14px 56px;gap:20px;}
.hdr{display:flex;align-items:center;gap:10px;width:min(98vw,880px);}
.hdr-dot{width:8px;height:8px;border-radius:50%;background:var(--green);}
h1{font-size:13px;letter-spacing:.18em;text-transform:uppercase;
  color:var(--muted);font-weight:400;}
.cols{display:flex;gap:16px;width:min(98vw,880px);align-items:flex-start;}
.cols>.panel{flex:1;min-width:0;}
.full{width:min(98vw,880px);}
.panel{background:var(--surface);border:1px solid var(--border);
  border-radius:10px;padding:18px 16px;
  display:flex;flex-direction:column;gap:14px;}
.panel-title{font-size:10px;letter-spacing:.2em;text-transform:uppercase;color:var(--muted);}
.dial-wrap{display:flex;flex-direction:column;align-items:center;gap:8px;}
.dial-svg{width:min(38vw,180px);aspect-ratio:1/1;cursor:pointer;touch-action:none;}
.dial-val{font-size:44px;font-weight:700;letter-spacing:-.03em;
  font-variant-numeric:tabular-nums;line-height:1;}
.dial-sub{font-size:11px;color:var(--text2);letter-spacing:.06em;}
.row{display:flex;gap:8px;}
.row>*{flex:1;}
input[type=number]{background:var(--bg);color:var(--text);
  border:1px solid var(--border);border-radius:6px;
  padding:8px 10px;font-size:14px;font-family:inherit;
  font-variant-numeric:tabular-nums;width:100%;}
input:focus{outline:none;border-color:var(--blue);}
button{font-family:inherit;font-size:11px;letter-spacing:.1em;
  text-transform:uppercase;padding:9px 12px;border-radius:6px;
  border:1px solid var(--border);background:#1a2030;color:var(--text);
  cursor:pointer;transition:background .15s;white-space:nowrap;}
button:hover{background:#222d40;}
button.prim{background:#1a3a6e;border-color:var(--blue);color:var(--blue);}
button.prim:hover{background:#204488;}
button.danger{background:#3a1818;border-color:var(--red);color:var(--red);}
button.safe{background:#183328;border-color:var(--green);color:var(--green);}
.vdiv{width:1px;background:var(--border);align-self:stretch;flex-shrink:0;}
.limits{font-size:11px;color:var(--muted);letter-spacing:.04em;}
.limits span{color:var(--text2);}
.mpu-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;}
.mpu-cell{background:var(--bg);border:1px solid var(--border);border-radius:6px;
  padding:8px 10px;display:flex;flex-direction:column;gap:2px;}
.mpu-lbl{font-size:9px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);}
.mpu-num{font-size:18px;font-variant-numeric:tabular-nums;font-weight:700;}
.mpu-unit{font-size:9px;color:var(--muted);}
.graph-wrap{display:flex;flex-direction:column;gap:6px;}
.graph-title{font-size:9px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);}
.graph-legend{display:flex;gap:12px;flex-wrap:wrap;}
.legend-item{display:flex;align-items:center;gap:5px;font-size:10px;color:var(--text2);}
.legend-dot{width:10px;height:3px;border-radius:2px;}
.toggle-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap;}
.toggle-row label{display:flex;align-items:center;gap:8px;font-size:11px;color:var(--text2);}
.toggle-row input[type=checkbox]{width:16px;height:16px;accent-color:var(--blue);}
canvas{width:100%;height:90px;border-radius:6px;background:var(--bg);
  border:1px solid var(--border);display:block;}
hr{border:none;border-top:1px solid var(--border);width:100%;}
#toast{position:fixed;bottom:22px;left:50%;transform:translateX(-50%);
  background:#1e2d20;border:1px solid var(--green);border-radius:8px;
  padding:9px 18px;font-size:12px;color:var(--green);letter-spacing:.08em;
  opacity:0;transition:opacity .3s;pointer-events:none;white-space:nowrap;}
#toast.show{opacity:1;}
@media(max-width:540px){
  .cols{flex-direction:column;}.vdiv{display:none;}
  .dial-svg{width:min(56vw,220px);}
  .mpu-grid{grid-template-columns:repeat(2,1fr);}
}
</style>
</head>
<body>

<div class="hdr"><div class="hdr-dot"></div><h1>Wheelo &middot; Control Panel</h1></div>

<div class="cols">

  <!-- BLDC -->
  <div class="panel">
    <div class="panel-title">&#9654; BLDC Throttle</div>
    <div class="dial-wrap">
      <svg id="bldc-dial" class="dial-svg" viewBox="-100 -100 200 200">
        <circle cx="0" cy="0" r="82" fill="none" stroke="#1e2733" stroke-width="13"/>
        <path id="bldc-arc" fill="none" stroke="#4d9eff" stroke-width="13" stroke-linecap="round"/>
        <circle id="bldc-knob" r="11" fill="#4d9eff" stroke="#0a0d12" stroke-width="3"/>
      </svg>
      <div id="bldc-val" class="dial-val">0<span style="font-size:22px;color:var(--text2)">%</span></div>
      <div id="bldc-us" class="dial-sub">1000 µs</div>
    </div>
    <div class="limits">Range: <span>0 – 30 %</span> &nbsp;|&nbsp; <span>1000 – 1300 µs</span></div>
    <div class="row">
      <input id="bldc-num" type="number" min="0" max="30" step="1" value="0" inputmode="numeric"/>
      <button class="prim" id="bldc-apply">Apply</button>
    </div>
    <button class="danger" id="bldc-stop">STOP (0%)</button>
  </div>

  <div class="vdiv"></div>

  <!-- Servo -->
  <div class="panel">
    <div class="panel-title">&#9711; ST3215 Position</div>
    <div class="dial-wrap">
      <svg id="srv-dial" class="dial-svg" viewBox="-100 -100 200 200">
        <circle cx="0" cy="0" r="82" fill="none" stroke="#1e2733" stroke-width="13"/>
        <path id="srv-arc" fill="none" stroke="#52c87a" stroke-width="13" stroke-linecap="round"/>
        <circle id="srv-knob" r="11" fill="#52c87a" stroke="#0a0d12" stroke-width="3"/>
      </svg>
      <div id="srv-val" class="dial-val">1100</div>
      <div id="srv-limits" class="dial-sub limits">750 – 1450</div>
      <div class="dial-sub">Live: <span id="srv-live" style="color:var(--green);font-weight:700;">—</span></div>
    </div>
    <div class="row">
      <input id="srv-num" type="number" min="0" max="4095" step="1" value="1100" inputmode="numeric"/>
      <button class="prim" id="srv-apply">Move</button>
    </div>
    <hr/>
    <div class="limits">Default pos: <span id="srv-defshow">1100</span></div>
    <div class="row">
      <input id="srv-def-num" type="number" min="0" max="4095" step="1"
             placeholder="New default…" inputmode="numeric"/>
      <button class="safe" id="srv-savedef">Save default</button>
    </div>
  </div>

</div>

<!-- Auto-Balance PID -->
<div class="panel full" id="balance-panel">
  <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;">
    <div class="panel-title">&#9878; Auto-Balance &mdash; Roll PID</div>
    <button id="bal-toggle" class="safe">START</button>
  </div>
  <div id="bal-status" style="font-size:11px;color:var(--muted);">Stopped &mdash; servo under manual control</div>

  <div class="mpu-grid">
    <div class="mpu-cell">
      <div class="mpu-lbl">Error</div>
      <div class="mpu-num" id="bal-err" style="color:#ff9900;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">PID Output</div>
      <div class="mpu-num" id="bal-out">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">SP Accum</div>
      <div class="mpu-num" id="bal-accum" style="color:#8855cc;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Live Pos</div>
      <div class="mpu-num" id="bal-srv" style="color:var(--green);">—</div>
      <div class="mpu-unit">/ 4095</div></div>
  </div>

  <hr/>
  <div class="panel-title">PID Gains &nbsp;<span style="font-size:9px;color:var(--muted);text-transform:none;letter-spacing:.02em;">direct values &nbsp;&#9660;&#9650; steps below</span></div>
  <div class="row">
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Kp &mdash; proportional &nbsp;&#9660;&#9650;&nbsp;0.1</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-kp" data-step="-0.1" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-kp" type="number" step="any" min="0" max="20" value="1.8" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-kp" data-step="0.1" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Ki &mdash; integral &nbsp;&#9660;&#9650;&nbsp;1</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-ki" data-step="-1" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-ki" type="number" step="any" min="0" max="100" value="0.0" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-ki" data-step="1" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Kd &mdash; derivative &nbsp;&#9660;&#9650;&nbsp;0.01</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-kd" data-step="-0.01" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-kd" type="number" step="any" min="0" max="5" value="0.09" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-kd" data-step="0.01" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
  </div>
  <div class="row">
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Trim (°) &mdash; balance point offset</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-sp" data-step="-0.1" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-sp" type="number" step="0.1" min="-30" max="30" value="0.0" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-sp" data-step="0.1" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
    <div style="display:flex;align-items:flex-end;">
      <button class="prim" id="pid-apply">Apply</button>
    </div>
    <div></div>
  </div>
  <div class="toggle-row">
    <label for="pid-accum-enabled">
      <input id="pid-accum-enabled" type="checkbox"/>
      Enable SetpointAccum recentering
    </label>
    <div class="limits">Off keeps the target angle fixed at <span>Trim</span> while tuning.</div>
  </div>

  <!-- PID range reference -->
  <div style="border:1px solid var(--border);border-radius:8px;overflow:hidden;font-size:10px;">
    <div style="background:#0d1520;padding:7px 12px;letter-spacing:.12em;text-transform:uppercase;color:var(--muted);font-size:9px;">
      Gain reference &amp; recommended start
    </div>
    <table style="width:100%;border-collapse:collapse;">
      <thead>
        <tr style="background:#0a0f18;color:var(--muted);font-size:9px;letter-spacing:.08em;">
          <th style="padding:5px 10px;text-align:left;border-bottom:1px solid var(--border);">Gain</th>
          <th style="padding:5px 10px;text-align:center;border-bottom:1px solid var(--border);">Useful range</th>
          <th style="padding:5px 10px;text-align:center;border-bottom:1px solid var(--border);color:#52c87a;">Start</th>
          <th style="padding:5px 10px;text-align:left;border-bottom:1px solid var(--border);">What it does</th>
        </tr>
      </thead>
      <tbody style="color:var(--text2);">
        <tr style="border-bottom:1px solid var(--border);">
          <td style="padding:6px 10px;color:var(--text);font-weight:700;">Kp</td>
          <td style="padding:6px 10px;text-align:center;">1 &ndash; 5</td>
          <td style="padding:6px 10px;text-align:center;color:#52c87a;font-weight:700;">1.8</td>
          <td style="padding:6px 10px;">1&deg; tilt &rarr; 1.8&deg; servo deflection</td>
        </tr>
        <tr style="border-bottom:1px solid var(--border);">
          <td style="padding:6px 10px;color:var(--text);font-weight:700;">Ki</td>
          <td style="padding:6px 10px;text-align:center;">0 &ndash; 50</td>
          <td style="padding:6px 10px;text-align:center;color:#52c87a;font-weight:700;">0.0</td>
          <td style="padding:6px 10px;">Start at 0 on the stand; add only after rate noise is controlled</td>
        </tr>
        <tr style="border-bottom:1px solid var(--border);">
          <td style="padding:6px 10px;color:var(--text);font-weight:700;">Kd</td>
          <td style="padding:6px 10px;text-align:center;">0.01 &ndash; 0.5</td>
          <td style="padding:6px 10px;text-align:center;color:#52c87a;font-weight:700;">0.09</td>
          <td style="padding:6px 10px;">Multiplies filtered gyro rate directly, in deg/sec</td>
        </tr>
        <tr>
          <td style="padding:6px 10px;color:var(--text);font-weight:700;">Trim</td>
          <td style="padding:6px 10px;text-align:center;">&minus;5 &ndash; 5</td>
          <td style="padding:6px 10px;text-align:center;color:#52c87a;font-weight:700;">0.0</td>
          <td style="padding:6px 10px;">Balance-point offset. Adjust until robot holds still.</td>
        </tr>
      </tbody>
    </table>
  </div>

  <div style="font-size:10px;color:var(--muted);line-height:1.85;border-top:1px solid var(--border);padding-top:10px;">
    <b style="color:#ff9900;">&#9650; Before starting:</b> hold robot in balance pose &rarr; press <b style="color:var(--text2);">Zero Angle</b> in MPU panel so Angle reads 0&deg; &rarr; adjust Trim until stable &rarr; hit START.<br/><br/>
    <b style="color:var(--text2);">Algorithm (10ms fixed rate, filtered gyro derivative):</b><br/>
    &bull; ki_int = Ki &times; 0.01 &nbsp;|&nbsp; derivative uses filtered gyro rate directly<br/>
    &bull; integral clamps to &plusmn;35&deg;, output clamps to &plusmn;35&deg;, servo command slew-limited<br/>
    &bull; SetpointAccum can be enabled later if you want slow gimbal re-centering<br/><br/>
    <b style="color:var(--text2);">Tuning recipe:</b> Start Kp=1.8 Ki=0 Kd=0.09 &rarr; verify Rate is quiet with flywheel on &rarr; adjust Trim first &rarr; add Ki last.
  </div>
</div>

<!-- MPU6050 -->
<div class="panel full">
  <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;">
    <div class="panel-title">&#11835; MPU6050 Filtered &nbsp;<span id="mpu-status" style="color:var(--muted)">connecting…</span></div>
    <div style="display:flex;gap:8px;flex-wrap:wrap;">
      <button id="mpu-cal-btn" class="prim" style="font-size:10px;padding:7px 12px;">Bias Cal</button>
      <button id="mpu-noise-btn" class="prim" style="font-size:10px;padding:7px 12px;">Gyro Noise Cal</button>
    </div>
  </div>
  <div id="mpu-cal-msg" style="font-size:11px;color:var(--muted);display:none;"></div>

  <div class="mpu-grid" style="grid-template-columns:repeat(3,1fr);">
    <div class="mpu-cell" style="border-color:#ff9900aa;">
      <div class="mpu-lbl" style="color:#ff9900;">Angle (Pitch)</div>
      <div class="mpu-num" id="m-angle" style="color:#ff9900;font-size:28px;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Rate</div>
      <div class="mpu-num" id="m-rate" style="color:#4d9eff;">—</div>
      <div class="mpu-unit">deg/sec</div></div>
    <div class="mpu-cell" style="padding:6px;">
      <button id="reset-angles-btn" style="width:100%;height:100%;font-size:9px;
        letter-spacing:.08em;background:#1a2030;border-color:#3a4a5a;color:#5a7a9a;
        border-radius:5px;cursor:pointer;padding:4px;">Zero<br/>Angle</button>
    </div>
  </div>

  <div class="mpu-grid">
    <div class="mpu-cell">
      <div class="mpu-lbl">Accel Angle</div>
      <div class="mpu-num" id="m-accel-angle">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Accel Norm</div>
      <div class="mpu-num" id="m-accel-norm">—</div>
      <div class="mpu-unit">g</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Dropped I2C</div>
      <div class="mpu-num" id="m-dropped">—</div>
      <div class="mpu-unit">reads</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Noise Floors (3σ)</div>
      <div class="mpu-num" id="m-noise" style="font-size:13px;line-height:1.5;">—</div>
      <div class="mpu-unit" id="m-noise-unit">rate / accel innov</div></div>
  </div>

  <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;">
    <span style="font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);white-space:nowrap;">Set angle:</span>
    <input id="init-angle" type="number" step="0.1" min="-360" max="360" placeholder="Angle °"
      inputmode="decimal"
      style="flex:1;min-width:70px;background:var(--bg);color:#ff9900;border:1px solid #ff990066;
             border-radius:6px;padding:7px 8px;font-size:13px;font-family:inherit;"/>
    <button id="set-angles-btn" class="prim" style="font-size:10px;padding:7px 14px;white-space:nowrap;">Set</button>
  </div>

  <div id="vertical-banner" style="display:none;background:#3a1010;border:1px solid #e05252;
    border-radius:8px;padding:10px 18px;text-align:center;font-size:13px;
    letter-spacing:.15em;color:#e05252;text-transform:uppercase;">
    &#9650; Robot is Vertical &#9650;
  </div>

  <div class="graph-wrap">
    <div class="graph-title">Angle + Rate &nbsp;<span style="color:var(--muted)">(auto-scale)</span></div>
    <div class="graph-legend">
      <div class="legend-item"><div class="legend-dot" style="background:#ff9900"></div>Angle</div>
      <div class="legend-item"><div class="legend-dot" style="background:#4d9eff"></div>Rate</div>
    </div>
    <canvas id="gyro-graph"></canvas>
  </div>
</div>

<div id="toast"></div>

<script>
(() => {
'use strict';

function makeDial(svgId, arcId, knobId, valId, initMin, initMax, onchange) {
  const svg=document.getElementById(svgId), arc=document.getElementById(arcId);
  const knob=document.getElementById(knobId), lbl=document.getElementById(valId);
  const START=135, SWEEP=270, R=82;
  let min=initMin, max=initMax, val=initMin, dragging=false;
  const polar=d=>{const r=d*Math.PI/180;return[Math.cos(r)*R,Math.sin(r)*R];};
  const render=()=>{
    const pct=(max===min)?0:(val-min)/(max-min), end=START+SWEEP*pct;
    const[sx,sy]=polar(START),[ex,ey]=polar(end);
    arc.setAttribute('d',`M ${sx} ${sy} A ${R} ${R} 0 ${end-START>180?1:0} 1 ${ex} ${ey}`);
    knob.setAttribute('cx',ex); knob.setAttribute('cy',ey); lbl.textContent=val;
  };
  const setVal=(v,push)=>{
    v=Math.max(min,Math.min(max,Math.round(v)));
    if(v===val&&!push){render();return;} val=v; render(); if(push)onchange(v);
  };
  const setRange=(lo,hi)=>{min=lo;max=hi;val=Math.max(min,Math.min(max,val));render();};
  const fromEvent=e=>{
    const rect=svg.getBoundingClientRect(), t=e.touches?e.touches[0]:e;
    const cx=rect.left+rect.width/2, cy=rect.top+rect.height/2;
    let a=Math.atan2(t.clientY-cy,t.clientX-cx)*180/Math.PI;
    let d=a-START; if(d<0)d+=360; if(d>SWEEP)d=(d-SWEEP<360-d)?SWEEP:0;
    setVal(min+(d/SWEEP)*(max-min),true);
  };
  svg.addEventListener('mousedown',e=>{dragging=true;fromEvent(e);e.preventDefault();});
  svg.addEventListener('touchstart',e=>{dragging=true;fromEvent(e);e.preventDefault();},{passive:false});
  window.addEventListener('mousemove',e=>{if(dragging){fromEvent(e);e.preventDefault();}});
  window.addEventListener('touchmove',e=>{if(dragging){fromEvent(e);e.preventDefault();}},{passive:false});
  window.addEventListener('mouseup',()=>dragging=false);
  window.addEventListener('touchend',()=>dragging=false);
  render();
  return{setVal,getVal:()=>val,setRange,render};
}

function throttledFetch(baseUrl){
  let busy=false,next=null;
  return async v=>{
    if(busy){next=v;return;} busy=true;
    try{await fetch(baseUrl+v);}catch(e){}
    busy=false;
    if(next!==null){const n=next;next=null;setTimeout(()=>throttledFetch(baseUrl)(n),0);}
  };
}
const sendThrottle=throttledFetch('/set?v=');
const sendServo=throttledFetch('/servo/set?p=');

function toast(msg){
  const el=document.getElementById('toast');
  el.textContent=msg; el.classList.add('show');
  setTimeout(()=>el.classList.remove('show'),2000);
}

const bldcUSEl=document.getElementById('bldc-us');
const bldcNumEl=document.getElementById('bldc-num');
const bldcValEl=document.getElementById('bldc-val');
const bldcDial=makeDial('bldc-dial','bldc-arc','bldc-knob','bldc-val',0,30,v=>{
  bldcValEl.innerHTML=v+'<span style="font-size:22px;color:var(--text2)">%</span>';
  bldcUSEl.textContent=(1000+v*10)+' µs';
  if(document.activeElement!==bldcNumEl)bldcNumEl.value=v;
  sendThrottle(v);
});
document.getElementById('bldc-apply').addEventListener('click',()=>{
  const v=parseInt(bldcNumEl.value,10); if(isNaN(v))return; bldcDial.setVal(v,true);
});
bldcNumEl.addEventListener('keydown',e=>{if(e.key==='Enter')document.getElementById('bldc-apply').click();});
document.getElementById('bldc-stop').addEventListener('click',()=>bldcDial.setVal(0,true));

const SWING=350;
let srvDefault=1100;
const srvNumEl=document.getElementById('srv-num');
const srvDefNumEl=document.getElementById('srv-def-num');
const srvDefShow=document.getElementById('srv-defshow');
const srvLimitsEl=document.getElementById('srv-limits');
const srvDial=makeDial('srv-dial','srv-arc','srv-knob','srv-val',
  Math.max(0,srvDefault-SWING),Math.min(4095,srvDefault+SWING),v=>{
  if(document.activeElement!==srvNumEl)srvNumEl.value=v;
  sendServo(v);
});
function updateSrvLimits(def){
  const lo=Math.max(0,def-SWING),hi=Math.min(4095,def+SWING);
  srvDial.setRange(lo,hi); srvLimitsEl.textContent=lo+' – '+hi;
  srvNumEl.min=lo; srvNumEl.max=hi;
}
document.getElementById('srv-apply').addEventListener('click',()=>{
  const v=parseInt(srvNumEl.value,10); if(isNaN(v))return; srvDial.setVal(v,true);
});
srvNumEl.addEventListener('keydown',e=>{if(e.key==='Enter')document.getElementById('srv-apply').click();});
document.getElementById('srv-savedef').addEventListener('click',async()=>{
  const v=parseInt(srvDefNumEl.value,10);
  if(isNaN(v)||v<0||v>4095){alert('Enter 0 – 4095');return;}
  try{
    await fetch('/servo/setdefault?p='+v);
    srvDefault=v; srvDefShow.textContent=v; updateSrvLimits(v); toast('Default saved');
  }catch(e){alert('Save failed');}
});

const N=300;
function makeGraph(canvasId,channels,minRange){
  const canvas=document.getElementById(canvasId), ctx=canvas.getContext('2d');
  const bufs=channels.map(()=>new Float32Array(N).fill(0));
  let head=0,W=0,H=0;
  const peaks=channels.map(()=>minRange*0.5);
  function resize(){W=canvas.width=canvas.offsetWidth; H=canvas.height=100;}
  resize(); window.addEventListener('resize',resize);
  function push(values){
    for(let i=0;i<channels.length;i++){
      bufs[i][head]=values[i];
      peaks[i]=Math.max(peaks[i]*0.997,Math.abs(values[i]));
    }
    head=(head+1)%N;
  }
  function draw(){
    if(!W||!H)return;
    ctx.clearRect(0,0,W,H);
    const dispMax=Math.max(...peaks,minRange);
    const range=dispMax*2, mid=H/2;
    ctx.strokeStyle='#1e2733'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(0,mid); ctx.lineTo(W,mid); ctx.stroke();
    ctx.setLineDash([3,4]); ctx.strokeStyle='#161e2a';
    ctx.beginPath(); ctx.moveTo(0,H*0.25); ctx.lineTo(W,H*0.25);
                     ctx.moveTo(0,H*0.75); ctx.lineTo(W,H*0.75); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle='#3a4a5a'; ctx.font='9px monospace'; ctx.textAlign='right';
    ctx.fillText('±'+dispMax.toFixed(2),W-4,11); ctx.textAlign='left';
    for(let c=0;c<channels.length;c++){
      ctx.strokeStyle=channels[c].color; ctx.lineWidth=1.5; ctx.beginPath();
      for(let x=0;x<W;x++){
        const idx=(head+Math.floor(x*N/W))%N;
        const y=H-((bufs[c][idx]+dispMax)/range)*H;
        if(x===0)ctx.moveTo(x,y); else ctx.lineTo(x,y);
      }
      ctx.stroke();
    }
  }
  return{push,draw};
}
const gyroGraph=makeGraph('gyro-graph',[{color:'#ff9900'},{color:'#4d9eff'}],5);

const mpuStatus=document.getElementById('mpu-status');
const mpuCalMsg=document.getElementById('mpu-cal-msg');
const mpuCalBtn=document.getElementById('mpu-cal-btn');
const mpuNoiseBtn=document.getElementById('mpu-noise-btn');
const vertBanner=document.getElementById('vertical-banner');

async function fetchMpu(){
  try{
    const r=await fetch('/mpu'), j=await r.json();
    if(!j.ok){mpuStatus.textContent='not found';mpuStatus.style.color='var(--red)';}
    else if(j.cal){mpuStatus.textContent='calibrating…';mpuStatus.style.color='var(--blue)';}
    else{
      mpuStatus.textContent='live'; mpuStatus.style.color='var(--green)';
      document.getElementById('m-angle').textContent=j.angle.toFixed(2);
      document.getElementById('m-rate').textContent=j.rate.toFixed(2);
      document.getElementById('m-accel-angle').textContent=j.accelAngle.toFixed(2);
      document.getElementById('m-accel-norm').textContent=j.accelNorm.toFixed(3);
      document.getElementById('m-dropped').textContent=j.dropped;
      if(j.noiseFloor!==undefined){
        const nf=j.noiseFloor.toFixed(4), id=j.innovDb!==undefined?j.innovDb.toFixed(3):'—';
        document.getElementById('m-noise').innerHTML=nf+'°/s<br>'+id+'°';
      }
      vertBanner.style.display=j.vertical?'block':'none';
      gyroGraph.push([j.angle,j.rate]);
    }
  }catch(e){}
  setTimeout(fetchMpu,80);
}
fetchMpu();
function renderLoop(){gyroGraph.draw();requestAnimationFrame(renderLoop);}
requestAnimationFrame(renderLoop);

async function runCal(url,msg,successMsg){
  mpuCalBtn.disabled=true; mpuNoiseBtn.disabled=true;
  mpuCalMsg.textContent=msg; mpuCalMsg.style.display='block';
  mpuStatus.textContent='calibrating…'; mpuStatus.style.color='var(--blue)';
  try{const r=await fetch(url),j=await r.json();if(j.done)toast(successMsg);}
  catch(e){toast('Failed — check connection');}
  mpuCalMsg.style.display='none';
  mpuStatus.textContent='live'; mpuStatus.style.color='var(--green)';
  mpuCalBtn.disabled=false; mpuNoiseBtn.disabled=false;
}
mpuCalBtn.addEventListener('click',()=>
  runCal('/mpu/calibrate','Bias cal — keep flat & still (~2 s)…','Bias saved ✓'));

mpuNoiseBtn.addEventListener('click',async()=>{
  mpuCalBtn.disabled=true; mpuNoiseBtn.disabled=true;
  mpuCalMsg.textContent='Gyro noise cal — keep still with flywheel spinning (~30 s)…';
  mpuCalMsg.style.display='block';
  mpuStatus.textContent='calibrating…'; mpuStatus.style.color='var(--blue)';
  try{
    const r=await fetch('/mpu/noisecal'), j=await r.json();
    if(j.done){
      const nf=(j.noiseFloor||0).toFixed(4), id=(j.noiseFloor||0);
      document.getElementById('m-noise').innerHTML=nf+'°/s<br>'+(j.innovDb||0).toFixed(3)+'°';
      toast('Rate floor: '+nf+' °/s · Accel DB: '+(j.innovDb||0).toFixed(3)+'° ✓');
    }
  }catch(e){toast('Failed — check connection');}
  mpuCalMsg.style.display='none';
  mpuStatus.textContent='live'; mpuStatus.style.color='var(--green)';
  mpuCalBtn.disabled=false; mpuNoiseBtn.disabled=false;
});
document.getElementById('reset-angles-btn').addEventListener('click',async()=>{
  try{await fetch('/mpu/resetangles');toast('Angle zeroed');}catch(e){}
});
document.getElementById('set-angles-btn').addEventListener('click',async()=>{
  const angle=parseFloat(document.getElementById('init-angle').value);
  if(isNaN(angle)){toast('Enter angle');return;}
  try{
    await fetch('/mpu/setangle?angle='+angle);
    toast('Angle set: '+angle+'°');
  }catch(e){toast('Failed');}
});

const balToggle=document.getElementById('bal-toggle');
const balStatus=document.getElementById('bal-status');
const balErrEl=document.getElementById('bal-err');
const balOutEl=document.getElementById('bal-out');
const balAccumEl=document.getElementById('bal-accum');
const balSrvEl=document.getElementById('bal-srv');
const pidKpEl=document.getElementById('pid-kp');
const pidKiEl=document.getElementById('pid-ki');
const pidKdEl=document.getElementById('pid-kd');
const pidSpEl=document.getElementById('pid-sp');
const pidAccumEnabledEl=document.getElementById('pid-accum-enabled');
let balRunning=false, balGainsLoaded=false;

function updateBalUI(j){
  balRunning=j.running;
  if(j.error!==undefined)     balErrEl.textContent=j.error.toFixed(2);
  if(j.output!==undefined)    balOutEl.textContent=j.output.toFixed(1);
  if(j.spAccum!==undefined)balAccumEl.textContent=j.spAccum.toFixed(2);
  if(j.servoPos!==undefined){
    balSrvEl.textContent=j.servoPos;
    const srvLive=document.getElementById('srv-live');
    if(srvLive)srvLive.textContent=j.servoPos;
  }
  if(!balGainsLoaded&&j.kp!==undefined){
    pidKpEl.value=j.kp.toFixed(4); pidKiEl.value=j.ki.toFixed(4);
    pidKdEl.value=j.kd.toFixed(4); pidSpEl.value=j.setpoint.toFixed(1);
    balGainsLoaded=true;
  }
  if(j.accumEnabled!==undefined) pidAccumEnabledEl.checked=!!j.accumEnabled;
  if(j.running){
    balToggle.textContent='STOP'; balToggle.className='danger';
    balStatus.textContent='Running — PID active'; balStatus.style.color='var(--green)';
  }else{
    balToggle.textContent='START'; balToggle.className='safe';
    balStatus.textContent='Stopped — servo under manual control'; balStatus.style.color='var(--muted)';
  }
}

async function fetchBalance(){
  try{const r=await fetch('/balance/state');updateBalUI(await r.json());}catch(e){}
  setTimeout(fetchBalance,200);
}
fetchBalance();

balToggle.addEventListener('click',async()=>{
  const url=balRunning?'/balance/stop':'/balance/start';
  try{await fetch(url);toast(balRunning?'Balance stopped':'Balance started');}
  catch(e){toast('Error');}
});

document.getElementById('pid-apply').addEventListener('click',async()=>{
  const kp=parseFloat(pidKpEl.value),ki=parseFloat(pidKiEl.value);
  const kd=parseFloat(pidKdEl.value),sp=parseFloat(pidSpEl.value);
  if([kp,ki,kd,sp].some(isNaN)){toast('Enter all values');return;}
  try{
    const r=await fetch('/balance/pid?kp='+kp+'&ki='+ki+'&kd='+kd+'&sp='+sp);
    if(!r.ok){toast('Server error '+r.status);return;}
    const j=await r.json();
    pidKpEl.value=j.kp.toFixed(4); pidKiEl.value=j.ki.toFixed(4);
    pidKdEl.value=j.kd.toFixed(4); pidSpEl.value=j.sp.toFixed(1);
    toast('Saved: Kp='+j.kp.toFixed(4)+' Ki='+j.ki.toFixed(4)+' Kd='+j.kd.toFixed(4));
  }catch(e){toast('Failed — is ESP32 connected?');}
});

pidAccumEnabledEl.addEventListener('change',async()=>{
  const enabled=pidAccumEnabledEl.checked?1:0;
  try{
    const r=await fetch('/balance/accum?enabled='+enabled);
    if(!r.ok){toast('Server error '+r.status);return;}
    const j=await r.json();
    pidAccumEnabledEl.checked=!!j.accumEnabled;
    toast(j.accumEnabled?'SetpointAccum enabled':'SetpointAccum disabled');
  }catch(e){toast('Failed â€” is ESP32 connected?');}
});

document.querySelectorAll('.pid-step').forEach(btn=>{
  btn.addEventListener('click',()=>{
    const inp=document.getElementById(btn.dataset.id);
    const step=parseFloat(btn.dataset.step);
    inp.value=((parseFloat(inp.value)||0)+step).toFixed(4);
  });
});

fetch('/state').then(r=>r.json()).then(j=>{
  if(j.pct!==undefined){
    bldcDial.setVal(j.pct,false);
    bldcValEl.innerHTML=j.pct+'<span style="font-size:22px;color:var(--text2)">%</span>';
    bldcUSEl.textContent=(1000+j.pct*10)+' µs'; bldcNumEl.value=j.pct;
  }
  if(j.servoDefault!==undefined){
    srvDefault=j.servoDefault; srvDefShow.textContent=j.servoDefault;
    srvDefNumEl.value=j.servoDefault; updateSrvLimits(j.servoDefault);
  }
  if(j.servoPos!==undefined){srvDial.setVal(j.servoPos,false);srvNumEl.value=j.servoPos;}
  if(j.servoLive!==undefined){const el=document.getElementById('srv-live');if(el)el.textContent=j.servoLive;}
}).catch(()=>{});

window.addEventListener('resize',()=>gyroGraph.draw());
})();
</script>
</body>
</html>
)HTML";

// ── Constructor / begin / handle ──────────────────────────────────────────────

WebUI::WebUI(WebServer& server, ServoController& servo, ESCController& esc,
             MPUSensor& mpu, BalancePID& pid, Preferences& prefs)
    : _server(server), _servo(servo), _esc(esc),
      _mpu(mpu), _pid(pid), _prefs(prefs) {}

void WebUI::begin() {
    _server.on("/",                 [this](){ handleRoot(); });
    _server.on("/set",              [this](){ handleSet(); });
    _server.on("/state",            [this](){ handleState(); });
    _server.on("/servo/set",        [this](){ handleServoSet(); });
    _server.on("/servo/setdefault", [this](){ handleServoSetDefault(); });
    _server.on("/mpu",              [this](){ handleMpu(); });
    _server.on("/mpu/calibrate",    [this](){ handleMpuCalibrate(); });
    _server.on("/mpu/noisecal",     [this](){ handleMpuNoiseCal(); });
    _server.on("/mpu/resetangles",  [this](){ handleMpuResetAngles(); });
    _server.on("/mpu/setangle",     [this](){ handleMpuSetAngle(); });
    _server.on("/balance/start",    [this](){ handleBalanceStart(); });
    _server.on("/balance/stop",     [this](){ handleBalanceStop(); });
    _server.on("/balance/pid",      [this](){ handleBalancePid(); });
    _server.on("/balance/accum",    [this](){ handleBalanceAccum(); });
    _server.on("/balance/state",    [this](){ handleBalanceState(); });
    _server.begin();
}

void WebUI::handle() { _server.handleClient(); }

// ── HTTP handlers ─────────────────────────────────────────────────────────────

void WebUI::handleRoot() {
    _server.send_P(200, "text/html", INDEX_HTML);
}

void WebUI::handleSet() {
    if (!_server.hasArg("v")) { _server.send(400, "text/plain", "missing v"); return; }
    _esc.setThrottle(_server.arg("v").toInt());
    _server.send(200, "text/plain", String(_esc.throttlePct));
}

void WebUI::handleState() {
    String j = "{\"pct\":"          + String(_esc.throttlePct)
             + ",\"servoPos\":"     + String(_servo.targetPos)
             + ",\"servoLive\":"    + String(_servo.currentPos)
             + ",\"servoDefault\":" + String(_servo.defaultPos)
             + ",\"servoLo\":"      + String(_servo.getLo())
             + ",\"servoHi\":"      + String(_servo.getHi())
             + "}";
    _server.send(200, "application/json", j);
}

void WebUI::handleServoSet() {
    if (!_server.hasArg("p")) { _server.send(400, "text/plain", "missing p"); return; }
    _servo.moveTo(_server.arg("p").toInt());
    _server.send(200, "text/plain", String(_servo.targetPos));
}

void WebUI::handleServoSetDefault() {
    if (!_server.hasArg("p")) { _server.send(400, "text/plain", "missing p"); return; }
    _servo.setDefault(_server.arg("p").toInt(), _prefs);
    _server.send(200, "text/plain", String(_servo.defaultPos));
}

void WebUI::handleMpu() {
    bool vert = fabsf(_mpu.angle) > 70.0f;
    char buf[280];
    snprintf(buf, sizeof(buf),
        "{\"ok\":%s,\"cal\":%s,\"angle\":%.2f,\"rate\":%.2f,"
        "\"accelAngle\":%.2f,\"accelNorm\":%.3f,\"dropped\":%lu,"
        "\"vertical\":%s,\"noiseFloor\":%.4f,\"innovDb\":%.4f}",
        _mpu.ok ? "true" : "false",
        _mpu.calibrating ? "true" : "false",
        _mpu.angle, _mpu.rateDps, _mpu.accelAngle, _mpu.accelNormG,
        (unsigned long)_mpu.droppedReads, vert ? "true" : "false",
        _mpu.gyroNoiseFloor, _mpu.accelInnovDeadband);
    _server.send(200, "application/json", buf);
}

void WebUI::handleMpuCalibrate() {
    if (!_mpu.ok)          { _server.send(503, "text/plain", "MPU not found"); return; }
    if (_mpu.calibrating)  { _server.send(409, "text/plain", "already calibrating"); return; }
    _pid.stop();
    _mpu.calibrate(_prefs);
    _server.send(200, "application/json", "{\"done\":true}");
}

void WebUI::handleMpuNoiseCal() {
    if (!_mpu.ok)         { _server.send(503, "text/plain", "MPU not found"); return; }
    if (_mpu.calibrating) { _server.send(409, "text/plain", "already busy"); return; }
    _pid.stop();
    bool ok = _mpu.characterizeNoise(_prefs);
    char buf[100];
    snprintf(buf, sizeof(buf), "{\"done\":%s,\"noiseFloor\":%.4f,\"innovDb\":%.4f}",
             ok ? "true" : "false", _mpu.gyroNoiseFloor, _mpu.accelInnovDeadband);
    _server.send(200, "application/json", buf);
}

void WebUI::handleMpuResetAngles() {
    _mpu.resetAngle(_prefs);
    _server.send(200, "application/json", "{\"done\":true}");
}

void WebUI::handleMpuSetAngle() {
    if (!_server.hasArg("angle")) {
        _server.send(400, "text/plain", "missing angle"); return;
    }
    _mpu.setAngle(_server.arg("angle").toFloat(), _prefs);
    char buf[60];
    snprintf(buf, sizeof(buf), "{\"done\":true,\"angle\":%.2f}", _mpu.angle);
    _server.send(200, "application/json", buf);
}

void WebUI::handleBalanceStart() {
    if (!_mpu.ok) { _server.send(503, "text/plain", "MPU not found"); return; }
    _pid.start();
    _server.send(200, "application/json", "{\"running\":true}");
}

void WebUI::handleBalanceStop() {
    _pid.stop();
    _server.send(200, "application/json", "{\"running\":false}");
}

void WebUI::handleBalancePid() {
    float kp   = _server.hasArg("kp") ? _server.arg("kp").toFloat() : _pid.kp;
    float ki   = _server.hasArg("ki") ? _server.arg("ki").toFloat() : _pid.ki;
    float kd   = _server.hasArg("kd") ? _server.arg("kd").toFloat() : _pid.kd;
    float trim = _server.hasArg("sp") ? _server.arg("sp").toFloat() : _pid.trim;
    _pid.setGains(kp, ki, kd, trim, _prefs);
    char buf[100];
    snprintf(buf, sizeof(buf),
        "{\"done\":true,\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"sp\":%.4f}",
        _pid.kp, _pid.ki, _pid.kd, _pid.trim);
    _server.send(200, "application/json", buf);
}

void WebUI::handleBalanceAccum() {
    if (!_server.hasArg("enabled")) {
        _server.send(400, "text/plain", "missing enabled"); return;
    }
    _pid.setAccumEnabled(_server.arg("enabled").toInt() != 0, _prefs);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"accumEnabled\":%s}",
             _pid.accumEnabled ? "true" : "false");
    _server.send(200, "application/json", buf);
}

void WebUI::handleBalanceState() {
    float sp = _pid.setpointAccum + _pid.trim;
    char buf[400];
    snprintf(buf, sizeof(buf),
        "{\"running\":%s,\"error\":%.2f,\"output\":%.1f,\"spAccum\":%.2f"
        ",\"servoPos\":%d,\"servoTarget\":%d"
        ",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"setpoint\":%.4f,\"accumEnabled\":%s"
        ",\"angle\":%.2f,\"rate\":%.2f,\"accelAngle\":%.2f,\"accelNorm\":%.3f,\"dropped\":%lu}",
        _pid.running ? "true" : "false",
        sp - _mpu.angle, _pid.output, _pid.setpointAccum,
        _servo.currentPos, _servo.targetPos,
        _pid.kp, _pid.ki, _pid.kd, _pid.trim,
        _pid.accumEnabled ? "true" : "false",
        _mpu.angle, _mpu.rateDps, _mpu.accelAngle, _mpu.accelNormG,
        (unsigned long)_mpu.droppedReads);
    _server.send(200, "application/json", buf);
}
