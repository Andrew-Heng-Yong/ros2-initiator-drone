import * as THREE from 'three';
import { OrbitControls } from '/OrbitControls.js';

const $ = id => document.getElementById(id);
const host = $('scene');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#101b22');
const viewer = new THREE.PerspectiveCamera(52, 1, .01, 200);
viewer.position.set(3, 2, 4);
let renderer;
try { renderer = new THREE.WebGLRenderer({antialias:true}); }
catch (error) { $('empty-title').textContent = 'WebGL is unavailable'; $('empty-reason').textContent = error.message; throw error; }
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
host.append(renderer.domElement);
const controls = new OrbitControls(viewer, renderer.domElement);
controls.target.set(0,0,-1.5);
controls.enableDamping = true;
const grid = new THREE.GridHelper(20,20,0x344f5b,0x22343e);
grid.position.y = -1;
scene.add(grid);
const world = new THREE.Group();
// Optical camera/world axes: x right, y down, z forward -> WebGL x right, y up, z back.
world.rotation.x = Math.PI;
scene.add(world);
const cloud = new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:.025,vertexColors:true,sizeAttenuation:true}));
world.add(cloud);
const trajectory = new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xeeb76a}));
world.add(trajectory);
const cameraMarker = new THREE.Group();
cameraMarker.matrixAutoUpdate=false;
const lines=[];
const corners=[[-.16,-.12,.25],[.16,-.12,.25],[.16,.12,.25],[-.16,.12,.25]];
for(let i=0;i<4;i++) lines.push(0,0,0,...corners[i],...corners[i],...corners[(i+1)%4]);
cameraMarker.add(new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(lines,3)),new THREE.LineBasicMaterial({color:0x66dfbf})));
cameraMarker.add(new THREE.Mesh(new THREE.SphereGeometry(.035,12,8),new THREE.MeshBasicMaterial({color:0x66dfbf})));
world.add(cameraMarker);
const axes=new THREE.AxesHelper(.45);world.add(axes);
let follow=false, feed='rgb', mapVersion=-1, state=null, lastSuccess=0, lastMapFetch=0, fitted=false;
let frameCount=0;
function animate(){requestAnimationFrame(animate);controls.update();renderer.render(scene,viewer);frameCount++;}
animate();
new ResizeObserver(()=>{const {width,height}=host.getBoundingClientRect();renderer.setSize(width,height);viewer.aspect=width/height;viewer.updateProjectionMatrix();}).observe(host);
function fit(){
  cloud.geometry.computeBoundingBox();
  const box=cloud.geometry.boundingBox?.clone();
  if(!box||box.isEmpty()){controls.target.set(0,0,-1.5);viewer.position.set(3,2,4);return;}
  box.expandByPoint(new THREE.Vector3(0,0,0));
  if(state)box.expandByPoint(new THREE.Vector3(state.pose[0][3],state.pose[1][3],state.pose[2][3]));
  const center=box.getCenter(new THREE.Vector3()).applyMatrix4(world.matrixWorld);
  const size=Math.max(box.getSize(new THREE.Vector3()).length(),1);
  controls.target.copy(center);viewer.position.copy(center).add(new THREE.Vector3(.7,.45,1).normalize().multiplyScalar(size*1.25));
  follow=false;$('follow').setAttribute('aria-pressed','false');
}
$('fit').onclick=fit;
$('follow').onclick=()=>{follow=!follow;$('follow').setAttribute('aria-pressed',String(follow));};
$('show-path').onchange=e=>{trajectory.visible=e.target.checked;};
host.onkeydown=e=>{
  const offset=viewer.position.clone().sub(controls.target), spherical=new THREE.Spherical().setFromVector3(offset);
  if(e.key==='ArrowLeft')spherical.theta-=.12;else if(e.key==='ArrowRight')spherical.theta+=.12;
  else if(e.key==='ArrowUp')spherical.phi-=.12;else if(e.key==='ArrowDown')spherical.phi+=.12;
  else if(e.key==='+'||e.key==='=')spherical.radius*=.9;else if(e.key==='-')spherical.radius*=1.1;else return;
  e.preventDefault();spherical.makeSafe();viewer.position.copy(controls.target).add(new THREE.Vector3().setFromSpherical(spherical));
};
for(const tab of document.querySelectorAll('[data-feed]'))tab.onclick=()=>{
  feed=tab.dataset.feed;for(const t of document.querySelectorAll('[data-feed]'))t.setAttribute('aria-selected',String(t===tab));
  $('camera-image').alt=`Live ${feed.toUpperCase()} camera image`;imageFrame=-1;updateImage();
};
let imageFrame=-1,imageBusy=false;
async function updateImage(){
  if(imageBusy||!state)return;
  const age=state.images[feed],fresh=age!==undefined&&age<2;
  $('feed-state').textContent=fresh?(state.mode==='demo'?'Synthetic':'Live'):age===undefined?'Waiting':'Stale';
  $('camera-image').style.opacity=fresh?'1':'.4';
  $('feed-caption').textContent=state.mode==='demo'?'SIMULATION · Generated test scene, not your camera.':feed==='thermal'?`Thermal · ${state.metrics.thermal_range?.join('–')??'—'} °C · independent view.`:feed==='depth'?'Registered depth · colour scale 0–6 m.':'RGB and registered depth drive tracking.';
  $('camera-image').alt=state.mode==='demo'?`Synthetic ${feed.toUpperCase()} test image`:`Live ${feed.toUpperCase()} camera image`;
  if(age===undefined){$('camera-image').hidden=true;$('feed-empty').hidden=false;return;}
  if(imageFrame===state.frame&&feed!=='thermal')return;
  imageBusy=true;const requestedFeed=feed;
  try{
    const response=await fetch(`/api/image/${feed}`,{signal:AbortSignal.timeout(3000)});
    if(!response.ok)throw Error('Image unavailable');
    const url=URL.createObjectURL(await response.blob());
    if(requestedFeed!==feed){URL.revokeObjectURL(url);return;}
    const old=$('camera-image').src;$('camera-image').src=url;$('camera-image').hidden=false;$('feed-empty').hidden=true;
    if(old.startsWith('blob:'))URL.revokeObjectURL(old);
    imageFrame=state.frame;
  }catch{$('feed-state').textContent='Unavailable';}finally{imageBusy=false;}
}
async function command(path){const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error('Request failed');}
$('reset').onclick=async()=>{if(!confirm('Clear this scene and use the next camera frame as the new origin? Save the map first if you need it.'))return;try{await command('/api/reset');$('action-status').textContent='New map requested';}catch{$('action-status').textContent='Could not reset. Check connection.';}};
$('record').onclick=async()=>{try{await command('/api/record');}catch{$('action-status').textContent='Could not change recording.';}};
async function poll(){
  try{
    const response=await fetch('/api/state',{signal:AbortSignal.timeout(3500)});if(!response.ok)throw Error('Connection failed');
    state=await response.json();lastSuccess=performance.now();
    $('connection').textContent=state.mode==='demo'?'Synthetic demo':'Connected to module';
    $('demo-warning').hidden=state.mode!=='demo';
    document.querySelector('h1').textContent=state.mode==='demo'?'Synthetic test room':'Your surroundings, in 3D.';
    $('mode').textContent=state.mode==='demo'?'DEMO':'LIVE';$('mode').classList.toggle('demo',state.mode==='demo');
    $('tracking').textContent=state.status.charAt(0).toUpperCase()+state.status.slice(1);
    $('tracking').classList.toggle('warning',state.status!=='tracking');$('reason').textContent=state.reason;
    ['x','y','z'].forEach((id,i)=>$(id).textContent=state.frame?state.pose[i][3].toFixed(3):'—');
    $('inliers').textContent=`${state.metrics.inliers??0} / ${state.metrics.matches??0}`;
    $('latency').textContent=state.metrics.processing_ms!==undefined?`${state.metrics.processing_ms} ms`:'—';
    $('gyro').textContent=state.gyro.fusion_ready?'Assisting':state.gyro.calibrated?'Calibrated · not fused':state.gyro.state??'Unavailable';
    $('point-count').textContent=`${state.points.toLocaleString()} mapped points`;
    $('record').textContent=state.recording?'Stop recording':'Record sequence';
    $('action-status').textContent=state.recording?`Recording · ${state.recorded_frames} / 300 frames`:state.recorded_frames?`${state.recorded_frames} frames saved on module`:'';
    $('empty').hidden=state.points>0;$('empty-title').textContent=state.status==='tracking'?'Building scene':$('tracking').textContent;$('empty-reason').textContent=state.reason;
    cameraMarker.matrix.set(...state.pose.flat());cameraMarker.matrixWorldNeedsUpdate=true;
    trajectory.geometry.dispose();trajectory.geometry=new THREE.BufferGeometry().setFromPoints(state.trajectory.map(p=>new THREE.Vector3(...p)));
    if(follow){const target=new THREE.Vector3(state.pose[0][3],-state.pose[1][3],-state.pose[2][3]);viewer.position.add(target.clone().sub(controls.target));controls.target.copy(target);}
    if(state.map_version!==mapVersion&&performance.now()-lastMapFetch>900){
      const version=state.map_version;lastMapFetch=performance.now();
      const response=await fetch('/api/scene',{signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error('Scene unavailable');
      const data=new Float32Array(await response.arrayBuffer());const count=data.length/6;
      const positions=new Float32Array(count*3),colors=new Float32Array(count*3);
      for(let i=0;i<count;i++){positions.set(data.subarray(i*6,i*6+3),i*3);for(let c=0;c<3;c++)colors[i*3+c]=data[i*6+3+c]/255;}
      const first=!fitted&&count>0;
      cloud.geometry.dispose();cloud.geometry=new THREE.BufferGeometry();cloud.geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));cloud.geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));cloud.geometry.computeBoundingSphere();
      mapVersion=version;if(first){fit();fitted=true;}
    }
    updateImage();
  }catch(error){if(performance.now()-lastSuccess>3000){$('connection').textContent='Disconnected · retrying';$('tracking').textContent='Disconnected';$('tracking').classList.add('warning');$('reason').textContent='Last scene retained. Waiting for the module.';}}
  setTimeout(poll,200);
}
poll();
// Read-only diagnostics for runtime verification, never used as tracking input.
window.__SCENE__={get state(){return state;},get renderedFrames(){return frameCount;},get pointCount(){return cloud.geometry.attributes.position?.count??0;}};
