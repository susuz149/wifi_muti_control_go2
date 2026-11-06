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
  focus() {}
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
document.getElementById('alignmentMode').value='row';
document.getElementById('layoutLeftIps').value='214, 120, 103';
document.getElementById('layoutRightIps').value='142, 113, 232';
document.getElementById('layoutMinSpacing').value='.5';
document.getElementById('layoutMaxSpacing').value='.8';
const pose={x:0,y:0,z:0,yaw:0};
const results=[4,2].map(number=>({number,ip:number===4?'192.168.31.191':'192.168.31.120',accepted:true,reason:'匹配通过',overlap:.9,rmse:.04,transform:{...pose},local_pose:{...pose},points:[[0,0,.5],[1,0,1],[0,1,1]]}));
let submitted=null;
let layoutRequest=null;
let motionRequest=null;
let cloudRequest=null;
const context=vm.createContext({document,window:{addEventListener(){},confirm:()=>true},setInterval:()=>0,setTimeout,clearTimeout,AbortController,console,
  fetch:async(path,options={})=>({ok:true,json:async()=>{
    if(path==='/api/frame/auto-align') { layoutRequest=JSON.parse(options.body); return {ok:true,token:'test'}; }
    if(path==='/api/frame/auto-align/test') return {ok:true,token:'test',done:true,reference:4,results:structuredClone(results),error:''};
    if(path==='/api/frame/auto-align/apply') { submitted=JSON.parse(options.body);return {ok:true,aligned:[4,2],robots:[]}; }
    if(path==='/api/telemetry') return {ok:true,robots:[{number:2,connected:true,aligned:true,last_pose_age:.1,map_pose:{x:.2,y:1,yaw:.3}}]};
    if(path==='/api/point-cloud') { cloudRequest=JSON.parse(options.body); return {ok:true,results:[{number:2,ok:true}],devices:[],robots:[]}; }
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
  await vm.runInContext('setPointCloud(true)',context);
  assert.deepEqual(cloudRequest,{targets:[2],enabled:true});
  await vm.runInContext('setPointCloud(false)',context);
  assert.deepEqual(cloudRequest,{targets:[2],enabled:false});
  vm.runInContext(`state.telemetry=[{number:2,ip:'192.0.2.1',connected:true,cloud_enabled:false,last_pose_age:.1,cloud_age:null}]; renderTelemetry()`,context);
  const card=document.getElementById('frameList').children.at(-1);
  assert.match(card.children[4].textContent,/点云未开启/);
  assert.doesNotMatch(card.children[4].textContent,/点云超时/);
  vm.runInContext(`state.devices=[{number:2,ip:'192.0.2.1',connected:false,connection_health:{status:'timeout',message:'心跳响应超过 12 秒未更新',heartbeat_age:13}}]; renderDevices()`,context);
  const device=document.getElementById('historyDeviceList').children.at(-1);
  assert.match(device.children[2].children[2].textContent,/心跳响应超过 12 秒/);
  assert.equal(device.children[3].textContent,'已断线');
  vm.runInContext(`state.devices=[
    {number:1,ip:'192.0.2.1',connected:true},
    {number:2,ip:'192.0.2.2',connected:false},
    {number:3,ip:'192.0.2.3',connected:false,busy:true}
  ]; state.selected=new Set([1,2]); renderDevices()`,context);
  assert.equal(document.getElementById('connectedDeviceList').children.length,1);
  assert.equal(document.getElementById('historyDeviceList').children.length,2);
  assert.equal(document.getElementById('connectedDeviceCount').textContent,'1');
  assert.equal(document.getElementById('historyDeviceCount').textContent,'2');
  assert.equal(document.getElementById('historyDeviceList').children[0].children[0].checked,true);
  vm.runInContext('state.devices[1].connected=true; renderDevices()',context);
  assert.equal(document.getElementById('connectedDeviceList').children.length,2);
  assert.equal(document.getElementById('connectedDeviceList').children[1].children[0].checked,true);
  document.getElementById('selectConnectedButton').click();
  assert.deepEqual(Array.from(vm.runInContext('selectedNumbers()',context)),[1,2]);
  vm.runInContext(`state.devices[0].name='前排'; state.devices[0].group='A组';
    state.devices[2].group='B组'; renderDevices()`,context);
  assert.match(document.getElementById('connectedDeviceList').children[0].children[2].children[0].textContent,/前排.*A组/);
  document.getElementById('deviceGroupSelect').value='B组';
  document.getElementById('selectGroupButton').click();
  assert.deepEqual(Array.from(vm.runInContext('selectedNumbers()',context)),[3]);
  const fetchBeforeManagement=context.fetch;
  let metadataRequest;
  let deleteRequest;
  context.fetch=async(path,options)=>{
    if(options.method==='PATCH') metadataRequest=JSON.parse(options.body);
    if(options.method==='DELETE') deleteRequest=path;
    return {ok:true,json:async()=>({ok:true,devices:options.method==='DELETE'?[]:[{number:3,ip:'192.0.2.3',name:'后排',group:'B组',connected:false}]})};
  };
  const metadata=document.getElementById('historyDeviceList').children[0].children[5];
  metadata.children[0].value='后排';
  await metadata.children[2].events.click[0]();
  assert.deepEqual(metadataRequest,{name:'后排',group:'B组'});
  assert.equal(document.getElementById('connectedDeviceList').children.length,0);
  const remove=document.getElementById('historyDeviceList').children[0].children[5].children[3];
  await remove.events.click[0]();
  assert.equal(deleteRequest,'/api/devices/3');
  assert.deepEqual(Array.from(vm.runInContext('selectedNumbers()',context)),[]);
  context.fetch=fetchBeforeManagement;
  vm.runInContext('state.devices=[]; renderDevices()',context);
  assert.equal(document.getElementById('connectedDeviceList').children.length,0);
  assert.match(document.getElementById('connectedDeviceList').textContent,/暂无/);
  assert.match(document.getElementById('historyDeviceList').textContent,/暂无/);
  console.log('PASS: device groups, counts, selection across reconnect, select connected, empty groups');
  console.log('PASS: named devices, group selection, metadata save, history deletion and selection cleanup');
  const originalFetch=context.fetch;
  context.fetch=async()=>{throw new Error('offline');};
  await assert.rejects(vm.runInContext('refresh()',context),/offline/);
  assert.equal(document.getElementById('connectedCount').textContent,'未知');
  assert.match(document.getElementById('scanMessage').textContent,/状态可能已过期/);
  context.fetch=originalFetch;
  await vm.runInContext('refresh()',context);
  assert.match(document.getElementById('scanMessage').textContent,/已恢复/);
  context.fetch=async()=>({status:502,json:async()=>{throw new Error('invalid JSON');}});
  await assert.rejects(vm.runInContext("api('/api/devices')",context),/HTTP 502/);
  context.setTimeout=fn=>setTimeout(fn,0);
  context.fetch=async(_path, options)=>new Promise((_resolve,reject)=>{
    options.signal.addEventListener('abort',()=>reject(new Error('aborted')));
  });
  await assert.rejects(vm.runInContext("api('/api/devices')",context),/状态读取超时/);
  context.setTimeout=setTimeout;
  let finishConnection;
  let connectionRequests=0;
  context.fetch=async()=>{
    connectionRequests++;
    return new Promise(resolve=>{finishConnection=()=>resolve({ok:true,json:async()=>({ok:true,devices:[],results:[]})});});
  };
  const connecting=vm.runInContext("connectionAction('/api/connect')",context);
  assert.equal(document.getElementById('connectButton').disabled,true);
  await vm.runInContext("connectionAction('/api/connect')",context);
  assert.equal(connectionRequests,1);
  finishConnection();
  await connecting;
  assert.equal(document.getElementById('connectButton').disabled,false);
  context.fetch=originalFetch;
  console.log('PASS: backend failure/recovery, invalid response, request timeout, duplicate connection guard');
  console.log('PASS: preview, canvas draw, micro-adjustments, confirmation guard, apply, cancel');
  console.log('PASS: capture positions without motion, start program, render live feedback');
  console.log('PASS: point cloud controls, no timeout when disabled, connection timeout in device list');
})().catch(error=>{console.error(error);process.exitCode=1;});
