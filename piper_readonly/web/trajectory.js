import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {buildTree,applyFeedback,urdfQuaternion} from './model.js';
const $=id=>document.getElementById(id);
async function get(url,kind='json'){const r=await fetch(url);if(!r.ok)throw Error(`${url}: HTTP ${r.status}`);return r[kind]();}
async function main(){
  const base=document.body.dataset.base||'/trajectory';
  const [plan,model,gripper]=await Promise.all([get(base+'/plan.json'),get(base+'/model.json'),get(base+'/gripper.json')]);
  if(plan.execution_allowed||plan.live||plan.kind!=='offline_kinematic_simulation')throw Error('Simulation data only');
  THREE.Object3D.DEFAULT_UP.set(0,0,1);
  const renderer=new THREE.WebGLRenderer({canvas:$('scene'),antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
  const scene=new THREE.Scene();scene.background=new THREE.Color('#101a26');
  const camera=new THREE.PerspectiveCamera(40,1,.002,20);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.05;controls.maxDistance=3;
  scene.add(new THREE.HemisphereLight(0xe5f3ff,0x394c5a,2.1));
  for(const p of [[1,-1,2],[-1,1,2]]){const l=new THREE.DirectionalLight(0xffffff,2);l.position.fromArray(p);scene.add(l);}
  const grid=new THREE.GridHelper(1.4,28,0x456075,0x283b4e);grid.rotation.x=Math.PI/2;scene.add(grid,new THREE.AxesHelper(.1));
  const tree=buildTree(model),display=new THREE.Group();display.position.fromArray(plan.display_offset_m);display.add(tree.root);scene.add(display);
  const loader=new STLLoader();let loaded=0;
  await Promise.all(model.links.flatMap(link=>link.visuals.map(async v=>{
    const mesh=new THREE.Mesh(loader.parse(await get(v.url,'arrayBuffer')),new THREE.MeshStandardMaterial({color:new THREE.Color(...v.color.slice(0,3)),roughness:.5,metalness:.2}));
    mesh.position.fromArray(v.origin.xyz);mesh.quaternion.copy(urdfQuaternion(v.origin.rpy));mesh.scale.fromArray(v.scale);tree.links[link.name].add(mesh);$('stats').textContent=`Memuat ${++loaded} mesh…`;
  })));
  const optical=new THREE.Group();optical.position.fromArray(plan.optical_extrinsic.xyz);optical.quaternion.copy(urdfQuaternion(plan.optical_extrinsic.rpy));tree.links.d405_camera_body.add(optical);
  optical.add(new THREE.AxesHelper(.065),new THREE.Mesh(new THREE.SphereGeometry(.003,12,8),new THREE.MeshBasicMaterial({color:0xffb364})));
  const aim=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(),.12,0xffb364,.012,.006);optical.add(aim);
  const path=plan.samples.map(s=>s.camera_base_xyz_m.map((x,j)=>x+plan.display_offset_m[j]));
  scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(path.map(p=>new THREE.Vector3(...p))),new THREE.LineBasicMaterial({color:0xffb364})));
  const opticalPoint=new THREE.Vector3(),opticalDirection=new THREE.Vector3();
  const moving=plan.moving_joint_indices||[5];
  let index=0,playing=false,started=0,rate=1;
  function applySample(s){
    const values=Object.fromEntries(s.q_rad.map((x,j)=>['joint'+(j+1),x]));
    if(gripper.mode==='width'){values.joint7=gripper.value/2;values.joint8=-gripper.value/2;}
    applyFeedback(tree,values);display.updateMatrixWorld(true);
  }
  // Fit the entire motion, not only the compact folded starting pose.
  const motionBox=new THREE.Box3();
  for(let i=0;i<plan.samples.length;i+=50){applySample(plan.samples[i]);motionBox.union(new THREE.Box3().setFromObject(display));}
  for(const p of path)motionBox.expandByPoint(new THREE.Vector3(...p));
  function show(i){
    index=i;const s=plan.samples[i];applySample(s);optical.getWorldPosition(opticalPoint);
    if(opticalPoint.distanceTo(new THREE.Vector3(...path[i]))>1e-8)throw Error('Camera path/FK mismatch');
    if(s.camera_base_forward){optical.getWorldDirection(opticalDirection);if(opticalDirection.distanceTo(new THREE.Vector3(...s.camera_base_forward))>1e-8)throw Error('Camera direction/FK mismatch');}
    $('time').value=i;$('clock').textContent=s.t_s.toFixed(2)+' / '+plan.duration_s+' s';
    if(plan.moving_joint_indices){
      $('stats').textContent='Δ dari awal · '+moving.map(j=>{const d=THREE.MathUtils.radToDeg(s.q_rad[j]-plan.q0_rad[j]);return `J${j+1} ${d>=0?'+':''}${d.toFixed(1)}°`;}).join(' · ');
      $('phase').textContent=s.t_s===plan.duration_s?'Selesai — kembali ke pose awal':(plan.stages.find(p=>s.t_s>=p.start_s&&s.t_s<p.end_s)?.label||'Pose awal — persiapan');
    }else{
      const delta=(s.q_rad[5]-plan.q0_rad[5])*180/Math.PI;
      $('stats').textContent=`SIMULASI · J6 relatif ${delta>=0?'+':''}${delta.toFixed(2)}° · sudut ${ (s.q_rad[5]*180/Math.PI).toFixed(2)}° · ${(s.j6_speed_rad_s*180/Math.PI).toFixed(2)}°/s`;
    }
    const x=35+270*s.t_s/plan.duration_s;$('cursor').setAttribute('x1',x);$('cursor').setAttribute('x2',x);
  }
  function fit(mode){
    const origin=new THREE.Vector3(...path[index]);camera.up.set(0,0,1);
    if(mode==='wrist'){controls.target.copy(origin);camera.position.copy(origin).add(new THREE.Vector3(.18,-.22,.12));}
    else{const center=motionBox.getCenter(new THREE.Vector3()),radius=motionBox.getSize(new THREE.Vector3()).length()/2;const distance=radius/Math.sin(Math.min(camera.fov*Math.PI/180,2*Math.atan(Math.tan(camera.fov*Math.PI/360)*camera.aspect))/2)*1.1;controls.target.copy(center);camera.position.copy(center).addScaledVector(new THREE.Vector3(...(mode==='top'?[0,-.001,1]:[1,-1.4,.7])).normalize(),distance);}
    controls.update();
  }
  for(const reason of plan.preflight.reasons){const li=document.createElement('li');li.textContent=reason;$('blockers').append(li);}
  $('speed').textContent=`Maksimum ${(plan.peak_joint_speed_deg_s??plan.peak_j6_speed_deg_s).toFixed(2)}°/s, profil mulai/berhenti halus.`;
  $('excursion').textContent=`Maksimum ${plan.camera_max_distance_from_start_mm.toFixed(2)} mm (nominal).`;
  $('stamp').textContent=plan.source_snapshot_at;
  if($('initial-joints'))$('initial-joints').textContent=plan.q0_deg.map((q,j)=>`J${j+1} ${q.toFixed(3)}°`).join(' · ');
  if($('rebase-delta'))$('rebase-delta').textContent=(plan.previous_start_delta_deg||[]).map((q,j)=>`J${j+1} ${q>=0?'+':''}${q.toFixed(2)}°`).join(' · ')||'—';
  if($('feedback-audit')&&plan.feedback_audit){
    const audit=plan.feedback_audit;
    $('feedback-audit').textContent=`${audit.sample_count-audit.invalid_count}/${audit.sample_count} sampel lolos. Umur feedback maksimum ${audit.max_required_age_ms.toFixed(0)} ms; batas 150 ms. ${audit.invalid_count?'Belum siap untuk gerak fisik.':'Pemeriksaan komunikasi saja, bukan validasi keselamatan gerak.'}`;
  }
  if(plan.joint_plot_indices){
    $('excursion').textContent=`${(plan.camera_max_distance_from_start_mm/10).toFixed(1)} cm (estimasi CAD).`;
    $('comparison').textContent=`${plan.camera_displacement_vs_wrist.toFixed(1)}× lebih jauh dari posisi awal.`;
    $('direction').textContent=`${plan.camera_max_axis_change_deg.toFixed(1)}° dari arah awal (nominal).`;
    $('duration').textContent=`${plan.duration_s} detik pada pemutaran 1×.`;
    $('chart-end').textContent=plan.duration_s+' s';
    const colors=['#6cd6c2','#80baff','#ffcd73','#df9eff','#ff8597'];
    plan.joint_plot_indices.forEach((j,k)=>{
      const curve=document.createElementNS('http://www.w3.org/2000/svg','polyline');
      curve.setAttribute('fill','none');curve.setAttribute('stroke',colors[k]);curve.setAttribute('stroke-width','1.8');
      curve.setAttribute('points',plan.samples.filter((s,i)=>i%10===0).map(s=>`${35+270*s.t_s/plan.duration_s},${70-50*(s.q_rad[j]-plan.q0_rad[j])/THREE.MathUtils.degToRad(plan.chart_limit_deg)}`).join(' '));$('curves').append(curve);
      const key=document.createElement('span');key.textContent='J'+(j+1);key.style.color=colors[k];$('joint-legend').append(key);
    });
    for(const [id,t] of [['scan-start',plan.stages[2].start_s],['scan-return',plan.stages.at(-1).start_s]]){
      $(id).disabled=false;$(id).onclick=()=>{playing=false;$('play').textContent='Putar simulasi';show(Math.round(t*plan.sampling_hz));};
    }
  }else $('curve').setAttribute('points',plan.samples.filter((s,i)=>i%10===0).map(s=>`${35+270*s.t_s/plan.duration_s},${70-50*(s.q_rad[5]-plan.q0_rad[5])/THREE.MathUtils.degToRad(5)}`).join(' '));
  $('time').max=plan.samples.length-1;$('time').disabled=false;$('play').disabled=false;
  $('time').oninput=()=>{playing=false;$('play').textContent='Putar simulasi';show(Number($('time').value));};
  $('play').onclick=()=>{playing=!playing;if(playing){if(index===plan.samples.length-1)show(0);started=performance.now()/1000-plan.samples[index].t_s/rate;}$('play').textContent=playing?'Jeda':'Putar simulasi';};
  if($('playback-rate'))$('playback-rate').onchange=()=>{rate=Number($('playback-rate').value);started=performance.now()/1000-plan.samples[index].t_s/rate;};
  $('frustum').onchange=()=>{optical.visible=$('frustum').checked;};
  for(const id of ['whole','wrist','top']){$(id).disabled=false;$(id).onclick=()=>fit(id);}
  const resize=()=>{const w=$('viewport').clientWidth,h=$('viewport').clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();};new ResizeObserver(resize).observe($('viewport'));resize();show(0);fit('whole');
  function tick(now){try{if(playing){const t=(now/1000-started)*rate;const i=Math.max(0,Math.min(Math.floor(t*plan.sampling_hz),plan.samples.length-1));if(i!==index)show(i);if(i===plan.samples.length-1){playing=false;$('play').textContent='Putar simulasi';}}controls.update();renderer.render(scene,camera);requestAnimationFrame(tick);}catch(e){$('error').hidden=false;$('error').textContent=e.message;playing=false;}}requestAnimationFrame(tick);
}
main().catch(e=>{$('error').hidden=false;$('error').textContent='Simulasi gagal: '+e.message;});
