import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {applyFeedback, buildTree, feedbackUsable, feedbackValues, urdfQuaternion, createRobotDisplay, displayPositionToBase} from './model.js';
import {createFGNG} from './fgng.js';
import {createJointMonitor} from './joint_monitor.js';

const $ = id => document.getElementById(id);
const text = (id, value) => { $(id).textContent = value; };
const number = (v, digits=1) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const hex = v => Number.isFinite(v) ? '0x'+v.toString(16).toUpperCase().padStart(2,'0') : '—';
const stateNames = ['Normal','Emergency stop','No IK solution','Singularity','Angle limit','Joint comm error','Brake not released','Collision','Teaching overspeed','Joint abnormal','Other exception','Teaching record','Teaching execution','Teaching pause','Controller temperature','Resistor temperature'];
const modeNames = ['Standby','CAN control','Teaching','Ethernet','Wi-Fi','Remote','Linkage teaching','Offline trajectory'];
let tree, model, scene, camera, renderer, controls, grid, baseAxes, feedbackMarker;
let fgngGroup;
let packet=null, lastReceipt=0, selected='base_link', meshCount=0, meshError='', modelError='';
let totalMeshes=0, modelReady=false, hasPose=false, lastRate=null, rate=0;
const meshes=[], frames=[], jointCards=[], driverRows=[], linkButtons=new Map();
const viewport=$('viewport');
const jointMonitor=createJointMonitor();

function notice(message, warning=false) {
  $('notice').hidden=!message;
  $('notice').textContent=message;
  $('notice').classList.toggle('warning',warning);
}

function makeReadouts() {
  for(let i=0;i<6;i++) {
    const row=document.createElement('tr');
    for(let j=0;j<4;j++) row.append(document.createElement('td'));
    row.children[0].textContent='J'+(i+1);
    $('driver-rows').append(row);driverRows.push(row);
  }
}

function makeModelReadouts() {
  model.joints.filter(j=>/^joint[1-6]$/.test(j.name)).forEach((j,i)=>{
    const card=document.createElement('div');card.className='joint-card';
    const head=document.createElement('div');head.className='joint-head';
    const name=document.createElement('span');name.textContent='J'+(i+1);
    const value=document.createElement('b');value.textContent='—';head.append(name,value);
    const track=document.createElement('div');track.className='joint-track';const bar=document.createElement('i');track.append(bar);
    const limit=document.createElement('div');limit.className='joint-limit';
    const lo=document.createElement('span'),hi=document.createElement('span');
    lo.textContent=Math.round(j.limit.lower*180/Math.PI)+'°';hi.textContent=Math.round(j.limit.upper*180/Math.PI)+'°';limit.append(lo,hi);
    const enable=document.createElement('div');enable.className='joint-enable';enable.textContent='ENABLE —';
    card.append(head,track,limit,enable);$('joint-cards').append(card);jointCards.push({card,value,bar,enable,joint:j});
  });
  for(const link of model.links) {
    if(!link.visuals.length) continue;
    const button=document.createElement('button');button.textContent=(link.name==='base_link'?'◇ ':'↳ ')+link.name;
    button.title='Pilih '+link.name;button.onclick=()=>selectLink(link.name);
    $('link-tree').append(button);linkButtons.set(link.name,button);
  }
  text('model-source', model.urdf+(model.firmware_match_known?' · sesuai firmware':' · firmware belum diketahui'));
  $('show-camera').disabled=!model.wrist_camera;
  text('camera-note',model.wrist_camera?(model.wrist_camera.viewer_fit?.enabled?'Mount disesuaikan untuk viewer · bukan kalibrasi optik':(model.wrist_camera.bottom_mount_flip?.enabled?'Mount bawah 180° · bukan kalibrasi optik':'Pose dari assembly CAD · bukan kalibrasi optik')):'Tidak ada assembly kamera');
}

function selectLink(name) {
  selected=name;text('selected-name',name);
  for(const [link,button] of linkButtons) {button.classList.toggle('active',link===name);button.setAttribute('aria-pressed',String(link===name));}
  for(const mesh of meshes) mesh.material.emissive.setHex(mesh.userData.link===name?0x153d45:0);
  updatePositions();
}

function setupScene() {
  THREE.Object3D.DEFAULT_UP.set(0,0,1);
  renderer=new THREE.WebGLRenderer({canvas:$('scene'),antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.3;
  scene=new THREE.Scene();scene.background=new THREE.Color(0x111821);
  fgngGroup=createFGNG(scene);
  camera=new THREE.PerspectiveCamera(42,1,.005,30);camera.up.set(0,0,1);
  controls=new OrbitControls(camera,renderer.domElement);
  controls.enableDamping=true;controls.dampingFactor=.12;controls.minDistance=.12;controls.maxDistance=5;
  controls.target.set(0,0,.23);camera.position.set(.8,-.9,.68);controls.update();
  controls.addEventListener('start',()=>{document.querySelectorAll('[data-view]').forEach(b=>b.classList.remove('active'));text('view-label','Orbit');});
  scene.add(new THREE.HemisphereLight(0xe7f1ff,0x4a5866,2));
  for(const [color,power,position] of [[0xffffff,2.8,[1,-1,2]],[0x99c7ed,1.6,[-1,1,.8]],[0xffffff,1,[0,1,1.5]]]) {
    const light=new THREE.DirectionalLight(color,power);light.position.fromArray(position);scene.add(light);
  }
  grid=new THREE.GridHelper(2,40,0x40546a,0x243240);grid.rotation.x=Math.PI/2;grid.position.z=-.001;scene.add(grid);
  baseAxes=new THREE.AxesHelper(.14);scene.add(baseAxes);
  feedbackMarker=new THREE.Group();
  feedbackMarker.add(new THREE.Mesh(new THREE.OctahedronGeometry(.007),new THREE.MeshBasicMaterial({color:0x82cfab,depthTest:false})));
  feedbackMarker.add(new THREE.AxesHelper(.055));feedbackMarker.visible=false;scene.add(feedbackMarker);
  const observer=new ResizeObserver(()=>{
    const w=viewport.clientWidth,h=viewport.clientHeight;
    if(!w||!h)return;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();
  });observer.observe(viewport);
  renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();modelError='WebGL terputus. Muat ulang halaman untuk memulihkan viewer.';notice(modelError,true);});
  const ray=new THREE.Raycaster();let down=null;
  renderer.domElement.addEventListener('pointerdown',e=>{if(e.button===0)down=[e.clientX,e.clientY];});
  renderer.domElement.addEventListener('pointerup',e=>{
    if(!down||!hasPose||!tree.root.visible||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5){down=null;return;}down=null;
    const box=renderer.domElement.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2((e.clientX-box.left)/box.width*2-1,1-(e.clientY-box.top)/box.height*2),camera);
    const visibleHit=ray.intersectObjects(meshes).find(item=>{
      let node=item.object;while(node){if(!node.visible)return false;node=node.parent;}return true;
    });
    if(visibleHit)selectLink(visibleHit.object.userData.link);
  });
  let framesDrawn=0,lastFps=performance.now();
  function frame(now) {
    controls.update();
    if(lastReceipt && performance.now()-lastReceipt>1200 && packet?.live) {
      packet={...packet,live:false,error:'Koneksi ke monitor terputus'};updateTelemetry();
    }
    refreshNotice();updatePositions();renderer.render(scene,camera);
    framesDrawn++;
    if(now-lastFps>1000) {text('render-rate',Math.round(framesDrawn*1000/(now-lastFps))+' FPS');framesDrawn=0;lastFps=now;}
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}

async function loadMeshes() {
  tree=buildTree(model);tree.root.visible=false;
  // Move robot visuals and its feedback marker together; world F-GNG stays put.
  scene.add(createRobotDisplay(tree,feedbackMarker));
  const loader=new STLLoader();
  totalMeshes=model.links.reduce((n,l)=>n+l.visuals.length,0);
  const jobs=[];
  for(const link of model.links) {
    const axes=new THREE.AxesHelper(.04);axes.visible=false;tree.links[link.name].add(axes);frames.push(axes);
    for(const visual of link.visuals) {
      jobs.push((async()=>{
        try {
          const response=await fetch(visual.url,{signal:AbortSignal.timeout(15000)});
          if(!response.ok)throw Error('HTTP '+response.status);
          const geometry=loader.parse(await response.arrayBuffer());geometry.computeBoundingBox();
          const material=new THREE.MeshStandardMaterial({color:new THREE.Color(...visual.color.slice(0,3)),metalness:.23,roughness:.48});
          const mesh=new THREE.Mesh(geometry,material);mesh.position.fromArray(visual.origin.xyz);
          mesh.quaternion.copy(urdfQuaternion(visual.origin.rpy));mesh.scale.fromArray(visual.scale);
          mesh.userData.link=link.name;tree.links[link.name].add(mesh);meshes.push(mesh);meshCount++;
          text('mesh-status',`PIPER · ${meshCount}/${totalMeshes} MESH`);
        } catch(e) {meshError='Mesh gagal dimuat: '+visual.url+' · '+e.message;}
      })());
    }
  }
  await Promise.all(jobs);modelReady=true;
  if(tree.links.d405_mount_frame)tree.links.d405_mount_frame.visible=$('show-camera').checked;
  selectLink(selected);applyPacket();
  if(hasPose)fitRobot('iso');
}

function fitRobot(view='iso', includeMap=false) {
  if(!tree||!hasPose)return;
  const box=new THREE.Box3();
  // Include visual meshes only; TF helpers must not inflate fit bounds.
  for(const mesh of meshes) if(mesh.visible) box.union(new THREE.Box3().setFromObject(mesh));
  if(includeMap&&fgngGroup?.visible)box.union(new THREE.Box3().setFromObject(fgngGroup));
  if(box.isEmpty())return;
  const center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());
  const radius=Math.max(size.length()/2,.1);
  const vfov=camera.fov*Math.PI/180,hfov=2*Math.atan(Math.tan(vfov/2)*camera.aspect);
  const distance=radius/Math.sin(Math.min(vfov,hfov)/2)*1.2;
  const directions={iso:[1,-1.25,.8],front:[1,0,0],side:[0,-1,0],top:[.0001,0,1]};
  const direction=view==='current'?camera.position.clone().sub(controls.target).normalize():new THREE.Vector3(...(directions[view]||directions.iso)).normalize();
  controls.target.copy(center);camera.position.copy(center).addScaledVector(direction,distance);controls.update();
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));
  text('view-label',({iso:'Perspective',front:'Front · +X',side:'Side · −Y',top:'Top · +Z'})[view]||'Perspective');
}

function applyPacket() {
  if(!tree||!feedbackUsable(packet))return;
  const first=!hasPose;applyFeedback(tree,feedbackValues(packet.data));hasPose=true;
  tree.root.visible=$('show-model').checked;
  // Unknown gripper feedback must not look like a measured zero-width jaw.
  const gripKnown=packet.data.gripper?.mode==='width'&&Number.isFinite(packet.data.gripper.value);
  tree.links.link7.visible=gripKnown;tree.links.link8.visible=gripKnown;
  if(first&&modelReady)fitRobot();
}

function updatePositions() {
  if(!tree||!hasPose)return;
  const displayed=tree.links[selected]?.getWorldPosition(new THREE.Vector3());
  const position=displayed?displayPositionToBase(displayed):null;
  if(position)text('selected-position',position.toArray().map((v,i)=>'XYZ'[i]+' '+number(v*1000)).join('  ')+' mm');
  const pose=feedbackUsable(packet)?packet.data.flange_pose_m_rad:null;
  feedbackMarker.visible=!!(pose&&$('show-flange').checked);
  if(pose) {
    feedbackMarker.position.fromArray(pose.slice(0,3));feedbackMarker.quaternion.copy(urdfQuaternion(pose.slice(3,6)));
    const modelPosition=tree.links.link6.getWorldPosition(new THREE.Vector3());
    const feedbackPosition=feedbackMarker.getWorldPosition(new THREE.Vector3());
    text('fk-delta',number(modelPosition.distanceTo(feedbackPosition)*1000,2)+' mm');
  } else text('fk-delta','—');
}

function refreshNotice() {
  if(modelError)return notice(modelError,true);
  if(meshError)return notice(meshError,true);
  if(!modelReady)return notice(`Memuat model resmi Piper · ${meshCount}/${totalMeshes||'…'} mesh`);
  if(packet?.snapshot)return notice('SNAPSHOT · data tersimpan, bukan kondisi live',true);
  if(!packet?.live)return notice(hasPose?'FEEDBACK STALE · pose terakhir ditahan, bukan kondisi saat ini':'Menunggu feedback robot. Pose tidak diasumsikan nol.',true);
  notice('');
}

function updateTelemetry() {
  jointMonitor.update(packet);
  const valid=packet?.live||packet?.snapshot;
  const d=valid?packet.data:null;
  const connected=!!packet?.live;
  text('connection',connected?'● LIVE FEEDBACK':packet?.snapshot?'SNAPSHOT / OFFLINE':'FEEDBACK STALE');
  $('connection').classList.toggle('live',connected);
  $('state-dot').classList.toggle('live',connected);$('footer-dot').classList.toggle('live',connected);
  document.body.classList.toggle('stale',!connected);
  const s=d?.arm_status;
  text('controller',s?(stateNames[s.arm_status]??'Status '+s.arm_status):'Unknown');
  text('control-mode',s?(modeNames[s.ctrl_mode]??'Mode '+s.ctrl_mode):'Tidak ada feedback terkini');
  text('error-code',s?hex(s.err_code):'—');
  text('firmware',d?.firmware?.software_version??'—');
  const drivers=d?.drivers??[],available=drivers.filter(v=>v!=null);
  text('enabled',available.length===6?available.filter(v=>v.foc_status?.driver_enable_status).length+' / 6 ON':'Unknown');
  const xyz=$('flange-xyz').children;
  for(let i=0;i<3;i++)xyz[i].querySelector('b').textContent=number(d?.flange_pose_m_rad?.[i]*1000,2);
  text('gripper-width',d?.gripper?.mode==='width'?number(d.gripper.value*1000)+' mm':'Unknown');
  text('gripper-force',d?.gripper?number(d.gripper.force)+' N':'—');
  const flagged=[];
  driverRows.forEach((row,i)=>{
    const driver=drivers[i];row.children[1].textContent=number(driver?.motor_temp,0);
    row.children[2].textContent=number(driver?.vol);row.children[3].textContent=hex(driver?.foc_status_code);
    const alert=!!(driver?.foc_status_code&0xBF);row.children[3].classList.toggle('flagged',alert);
    if(alert)flagged.push('J'+(i+1)+' '+hex(driver.foc_status_code));
    row.title=driver?Object.entries(driver.foc_status||{}).filter(([,v])=>v).map(([k])=>k).join(', '):'Data tidak tersedia';
  });
  text('driver-alert',flagged.length?flagged.join(' · ')+'. 0x30 = collision + driver error menurut SDK. Status controller ditampilkan terpisah; bukan diagnosis.':(available.length?'Tidak ada flag driver aktif pada data yang tersedia.':'Data driver tidak tersedia.'));
  $('driver-alert').classList.toggle('warning',!!flagged.length);
  jointCards.forEach(({card,value,bar,enable,joint},i)=>{
    const q=d?.joint_angles_rad?.[i],lo=joint.limit.lower,hi=joint.limit.upper;
    value.textContent=number(Number.isFinite(q)?q*180/Math.PI:NaN,2)+(Number.isFinite(q)?'°':'');
    bar.style.width=Number.isFinite(q)?Math.min(100,Math.max(0,(q-lo)/(hi-lo)*100))+'%':'0%';
    const outside=Number.isFinite(q)&&(q<lo||q>hi);card.classList.toggle('outside',outside);
    card.title=outside?'Feedback berada di luar batas nominal URDF; pose tidak di-clamp.':'Posisi di rentang URDF';
    const on=drivers[i]?.foc_status?.driver_enable_status;
    enable.textContent=on===undefined?'ENABLE —':on?'ENABLE ON':'ENABLE OFF';
  });
  if(d) {
    const now=performance.now();
    if(lastRate&&now-lastRate.time>1000){rate=Math.max(0,(d.rx_frame_count-lastRate.count)*1000/(now-lastRate.time));lastRate={time:now,count:d.rx_frame_count};}
    else if(!lastRate)lastRate={time:now,count:d.rx_frame_count};
    text('rx-rate',`RX ${number(rate,0)} frames/s`);
    text('sample-time',new Date(d.captured_at).toLocaleTimeString('id-ID',{hour12:false})+(connected?' · feedback < 0,5 s':' · snapshot'));
    text('tx-info',`Motor TX diblokir · Query FW ${d.firmware_queries_sent??'—'} · Ditolak ${d.blocked_commands??'—'}`);
  } else {text('rx-rate','RX —');text('sample-time',packet?.error||'Tidak ada sampel baru');}
  $('snapshot').disabled=!packet?.data;
  applyPacket();refreshNotice();
}

async function poll() {
  try {
    const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(2000)});
    if(!response.ok)throw Error('HTTP '+response.status);
    packet=await response.json();lastReceipt=performance.now();
  } catch(e) {packet={...packet,live:false,snapshot:false,error:'Monitor tidak terhubung · '+e.message};}
  updateTelemetry();setTimeout(poll,100);
}

function bindControls() {
  $('show-model').onchange=()=>{if(tree)tree.root.visible=hasPose&&$('show-model').checked;};
  $('show-camera').onchange=()=>{if(tree?.links.d405_mount_frame)tree.links.d405_mount_frame.visible=$('show-camera').checked;};
  $('show-grid').onchange=()=>{if(grid)grid.visible=$('show-grid').checked;};
  $('show-axes').onchange=()=>{if(baseAxes)baseAxes.visible=$('show-axes').checked;};
  $('show-frames').onchange=()=>frames.forEach(f=>f.visible=$('show-frames').checked);
  $('show-wire').onchange=()=>meshes.forEach(m=>m.material.wireframe=$('show-wire').checked);
  $('opacity').oninput=()=>{const a=Number($('opacity').value)/100;text('opacity-value',Math.round(a*100)+'%');meshes.forEach(m=>{m.material.opacity=a;m.material.transparent=a<1;m.material.depthWrite=a===1;m.material.needsUpdate=true;});};
  $('fit').onclick=()=>fitRobot('current');$('reset').onclick=()=>fitRobot('iso');
  $('fit-map').onclick=()=>fitRobot('current',true);
  document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>fitRobot(b.dataset.view));
  $('fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await viewport.requestFullscreen();}catch{notice('Layar penuh tidak tersedia di browser ini.',true);}};
  viewport.onkeydown=e=>{if(e.key.toLowerCase()==='f'){e.preventDefault();fitRobot('current');}if(e.key.toLowerCase()==='r'){e.preventDefault();fitRobot('iso');}};
  $('theme').onclick=()=>{const light=document.body.classList.toggle('light');text('theme',light?'Tema gelap':'Tema terang');try{localStorage.setItem('piper-theme',light?'light':'dark');}catch{}};
  try{if(localStorage.getItem('piper-theme')==='light')$('theme').click();}catch{}
  $('snapshot').onclick=()=>{
    if(!packet?.data)return;
    const url=URL.createObjectURL(new Blob([JSON.stringify({...packet,model:{urdf:model?.urdf,source_commit:model?.source_commit,wrist_camera:model?.wrist_camera}},null,2)],{type:'application/json'}));
    const a=document.createElement('a');a.href=url;a.download='piper_snapshot_'+new Date().toISOString().replaceAll(':','-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
}


async function boot() {
  makeReadouts();bindControls();poll();
  try {
    const response=await fetch('/api/model',{signal:AbortSignal.timeout(5000)});
    if(!response.ok)throw Error('Deskripsi model HTTP '+response.status);
    model=await response.json();makeModelReadouts();setupScene();await loadMeshes();
  } catch(e) {modelError='Viewer 3D gagal: '+e.message;notice(modelError,true);console.error(e);}
}
boot();
