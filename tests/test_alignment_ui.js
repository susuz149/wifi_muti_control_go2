// Exercise the real page script with a minimal DOM and mocked read-only API.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
class Element {
  constructor(tag='div') {
    this.tag=tag; this.children=[]; this.style={}; this.dataset={}; this.events={};
    this.value=''; this.width=1000; this.height=420; this.hidden=false;
  }
  get options() { return this.children; }
  appendChild(child) {
    child.value=String(child.value);
    this.children.push(child);
    if(this.tag==='select' && !this.value) this.value=child.value;
    return child;
  }
  append(...children) { children.forEach(c=>this.appendChild(c)); }
  replaceChildren() { this.children=[]; if(this.tag==='select') this.value=''; }
  addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
  click() { for(const fn of this.events.click || []) fn({currentTarget:this}); }
  setAttribute() {}
  contains() { return false; }
  querySelector() { return new Element(); }
  querySelectorAll() { return []; }
  getContext() { return new Proxy({}, {get:()=>()=>{}}); }
  getBoundingClientRect() { return {width:1000,height:420}; }
  setPointerCapture() {}
}
const elements = new Map();
const document = {
  activeElement:null,
  getElementById(id) {
    if(!elements.has(id)) elements.set(id,new Element(id==='adjustRobot'?'select':'div'));
    return elements.get(id);
  },
  createElement:tag=>new Element(tag), querySelectorAll:()=>[], addEventListener() {},
};
document.getElementById('referenceIp').value='192.168.31.191';
document.getElementById('layoutEnabled').checked=true;
document.getElementById('layoutLeftIps').value='214, 120, 103';
document.getElementById('layoutRightIps').value='142, 113, 232';
document.getElementById('layoutMinSpacing').value='.5';
document.getElementById('layoutMaxSpacing').value='.8';
const pose={x:0,y:0,z:0,yaw:0};
const results=[4,2].map(number=>({number,ip:number===4?'192.168.31.191':'192.168.31.120',accepted:true,reason:'匹配通过',overlap:.9,rmse:.04,transform:{...pose},local_pose:{...pose},points:[[0,0,.5],[1,0,1],[0,1,1]]}));
let submitted=null;
let layoutRequest=null;
let motionRequest=null;
const context=vm.createContext({document,window:{addEventListener(){},confirm:()=>true},setInterval:()=>0,setTimeout,console,
  fetch:async(path,options={})=>({ok:true,json:async()=>{
    if(path==='/api/frame/auto-align') { layoutRequest=JSON.parse(options.body); return {ok:true,token:'test'}; }
    if(path==='/api/frame/auto-align/test') return {ok:true,token:'test',done:true,reference:4,results:structuredClone(results),error:''};
    if(path==='/api/frame/auto-align/apply') { submitted=JSON.parse(options.body);return {ok:true,aligned:[4,2],robots:[]}; }
    if(path==='/api/telemetry') return {ok:true,robots:[{number:2,connected:true,aligned:true,last_pose_age:.1,map_pose:{x:.2,y:1,yaw:.3}}]};
    if(path==='/api/motion') {
      if(options.method==='POST') motionRequest=JSON.parse(options.body);
      return {ok:true,motion:{status:options.method==='POST'?'running':'idle',step:0,robots:{}}};
    }
    return {ok:true,devices:[],robots:[],points:[],videos:{},profiles:{}};
  }}),
});
vm.runInContext(fs.readFileSync('wifi_web/static/app.js','utf8'),context);
(async()=>{
  await new Promise(resolve=>setImmediate(resolve));
  await vm.runInContext('state.selected.add(2); autoAlign()',context);
  assert.equal(layoutRequest.layout_enabled,true);
  assert.equal(layoutRequest.left_ips,'214, 120, 103');
  assert.equal(layoutRequest.right_ips,'142, 113, 232');
  assert.equal(document.getElementById('alignmentEditor').hidden,false);
  assert.equal(document.getElementById('applyAutoAlignButton').disabled,false);
  document.getElementById('adjustForward').click();
  document.getElementById('adjustCCW').click();
  await vm.runInContext('applyAutoAlign()',context);
  assert.equal(submitted,null,'unconfirmed edits must not be applied');
  document.getElementById('acceptAdjustment').click();
  await vm.runInContext('applyAutoAlign()',context);
  assert.ok(Math.abs(submitted.adjustments['2'].x-.1)<1e-6);
  assert.ok(Math.abs(submitted.adjustments['2'].yaw-Math.PI/90)<1e-6);
  assert.equal(vm.runInContext('alignmentPreview',context),null);
  await vm.runInContext('autoAlign()',context);
  document.getElementById('cancelAutoAlignButton').click();
  assert.equal(vm.runInContext('alignmentPreview',context),null);
  assert.equal(document.getElementById('applyAutoAlignButton').disabled,true);
  await vm.runInContext('captureMotion()',context);
  assert.equal(motionRequest,null,'capturing positions must not start motion');
  const program=JSON.parse(document.getElementById('motionProgram').value);
  assert.deepEqual(program.steps[0].targets['2'],{x:.2,y:1,yaw:.3});
  document.getElementById('confirmMotion').checked=true;
  document.getElementById('confirmFormation').checked=true;
  await vm.runInContext('startMotion()',context);
  assert.equal(motionRequest.confirm_motion,true);
  assert.equal(motionRequest.program.steps[0].targets['2'].x,.2);
  assert.equal(document.getElementById('motionStart').disabled,true);
  vm.runInContext(`renderMotion({status:'completed',step:1,total_steps:1,robots:{2:{pose:{x:.2,y:1,yaw:.3},target:{x:.2,y:1,yaw:.3},distance:0,yaw_error:0,command:{vx:0,vy:0,wz:0},arrived:true}}})`,context);
  assert.equal(document.getElementById('motionRobots').children.length,1);
  assert.equal(document.getElementById('motionStart').disabled,false);
  console.log('PASS: preview, canvas draw, micro-adjustments, confirmation guard, apply, cancel');
  console.log('PASS: capture positions without motion, start program, render live feedback');
})().catch(error=>{console.error(error);process.exitCode=1;});
