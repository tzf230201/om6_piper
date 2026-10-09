import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {buildTree, applyFeedback, urdfQuaternion} from './model.js';

const $ = id => document.getElementById(id);
document.querySelectorAll('[data-view]').forEach(b=>{b.disabled=true;});
async function get(url, kind='json') {const r=await fetch(url);if(!r.ok)throw Error(`${url}: HTTP ${r.status}`);return r[kind]();}
async function main(){
  const [data, buffer]=await Promise.all([get('/camera/cad/scene.json'),get('/camera/cad/frames.bin','arrayBuffer')]);
  if(data.live || data.calibrated)throw Error('This page accepts recorded, nominal CAD data only.');
  THREE.Object3D.DEFAULT_UP.set(0,0,1);
  const renderer=new THREE.WebGLRenderer({canvas:$('scene'),antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
  const scene=new THREE.Scene();scene.background=new THREE.Color('#101a26');
  const camera=new THREE.PerspectiveCamera(45,1,.005,30);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
  controls.minDistance=.03;controls.maxDistance=8;
  scene.add(new THREE.HemisphereLight(0xe6f2ff,0x45515b,2.3));
  for(const position of [[1,-2,3],[-2,1,2]]){const l=new THREE.DirectionalLight(0xffffff,2.2);l.position.fromArray(position);scene.add(l);}
  const grid=new THREE.GridHelper(3,60,0x476176,0x26384a);grid.rotation.x=Math.PI/2;scene.add(grid,new THREE.AxesHelper(.15));
  const tree=buildTree(data.model),robot=new THREE.Group();robot.position.fromArray(data.robot_display_offset);robot.add(tree.root);scene.add(robot);
  applyFeedback(tree,data.joint_values);
  const loader=new STLLoader();let count=0;
  await Promise.all(data.model.links.flatMap(link=>link.visuals.map(async v=>{
    const geometry=loader.parse(await get(v.url,'arrayBuffer'));
    const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:new THREE.Color(...v.color.slice(0,3)),metalness:.2,roughness:.55}));
    mesh.position.fromArray(v.origin.xyz);mesh.quaternion.copy(urdfQuaternion(v.origin.rpy));mesh.scale.fromArray(v.scale);tree.links[link.name].add(mesh);
    $('stats').textContent=`Memuat ${++count} mesh…`;
  })));
  robot.updateMatrixWorld(true);
  const matrix=new THREE.Matrix4().set(...data.camera_to_display.flat());
  // Independently compare browser FK + nominal optical frame to exported transform.
  const local=new THREE.Matrix4().compose(new THREE.Vector3(...data.optical_extrinsic.xyz),urdfQuaternion(data.optical_extrinsic.rpy),new THREE.Vector3(1,1,1));
  const viaTree=tree.links.d405_camera_body.matrixWorld.clone().multiply(local);
  if(Math.max(...matrix.elements.map((v,i)=>Math.abs(v-viaTree.elements[i])))>1e-8)throw Error('Browser FK/optical transform mismatch');
  const map=new THREE.Group();map.matrixAutoUpdate=false;map.matrix.copy(matrix);scene.add(map);
  const points=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0x7ae0cc,size:.004,sizeAttenuation:true}));
  const lines=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x329c8f,transparent:true,opacity:.48}));
  map.add(lines,points);
  const optical=new THREE.Group();optical.matrixAutoUpdate=false;optical.matrix.copy(matrix);scene.add(optical);
  optical.add(new THREE.Mesh(new THREE.SphereGeometry(.006,12,8),new THREE.MeshBasicMaterial({color:0xffb364})),new THREE.AxesHelper(.055));
  const intr=data.intrinsics,z=.15;
  const corners=[[0,0],[intr.width,0],[intr.width,intr.height],[0,intr.height]].map(([u,v])=>[(u-intr.cx)/intr.fx*z,(v-intr.cy)/intr.fy*z,z]);
  const segments=[];for(let i=0;i<4;i++)segments.push(0,0,0,...corners[i],...corners[i],...corners[(i+1)%4]);
  optical.add(new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(segments,3)),new THREE.LineBasicMaterial({color:0xffb364,transparent:true,opacity:.7})));
  const bounds=new THREE.Box3(new THREE.Vector3(...data.bounds[0]),new THREE.Vector3(...data.bounds[1]));bounds.union(new THREE.Box3().setFromObject(robot));
  const center=bounds.getCenter(new THREE.Vector3()),radius=bounds.getSize(new THREE.Vector3()).length()/2;
  let sensorView=false;
  function view(name){
    sensorView=name==='sensor';controls.enabled=!sensorView;
    optical.visible=!sensorView&&$('frustum').checked;grid.visible=!sensorView;
    camera.fov=45;camera.up.set(0,0,1);
    const origin=new THREE.Vector3().setFromMatrixPosition(matrix);
    if(name==='sensor'){
      camera.fov=2*Math.atan(intr.height/(2*intr.fy))*180/Math.PI;
      camera.position.copy(origin);camera.up.set(0,-1,0).transformDirection(matrix);
      controls.target.copy(origin).add(new THREE.Vector3(0,0,.5).transformDirection(matrix).multiplyScalar(.5));
      camera.quaternion.setFromRotationMatrix(matrix).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI));
    }else if(name==='wrist'){
      controls.target.copy(origin);camera.position.copy(origin).add(new THREE.Vector3(.20,-.25,.13));
    }else{
      const distance=radius/Math.sin(Math.min(camera.fov*Math.PI/180,2*Math.atan(Math.tan(camera.fov*Math.PI/360)*camera.aspect))/2)*1.08;
      controls.target.copy(center);camera.position.copy(center).addScaledVector(new THREE.Vector3(...({iso:[1,-1.3,.8],top:[0,-.001,1],side:[0,-1,.001]}[name]||[1,-1,.8])).normalize(),distance);
    }
    camera.updateProjectionMatrix();if(!sensorView)controls.update();document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('selected',b.dataset.view===name));
  }
  let index=0,playing=false,started=0;
  function show(i){
    index=i;const f=data.frames[i];const p=new Float32Array(buffer,f.byte_offset,f.nodes*3);
    const e=new Uint32Array(buffer,f.byte_offset+f.nodes*12,f.edges*2);
    if(f.byte_offset+f.nodes*12+f.edges*8>buffer.byteLength)throw Error('Frame outside binary');
    points.geometry.dispose();points.geometry=new THREE.BufferGeometry().setAttribute('position',new THREE.BufferAttribute(p,3));
    lines.geometry.dispose();lines.geometry=new THREE.BufferGeometry().setAttribute('position',new THREE.BufferAttribute(p,3)).setIndex(new THREE.BufferAttribute(e,1));
    $('time').value=i;$('clock').textContent=f.elapsed_s.toFixed(2)+' / '+data.duration_s.toFixed(2)+' s';
    $('stats').textContent=`Frame ${f.index+1}/300 · ${f.nodes.toLocaleString()} titik · ${f.edges.toLocaleString()} sambungan · ${count} mesh`;
    $('rgb').src=f.rgb;$('rgb-note').textContent=`RGB sebagai konteks · selisih timestamp ${f.rgb_offset_ms.toFixed(1)} ms. Graph disimpan setiap 5 frame.`;
  }
  $('time').max=data.frames.length-1;$('time').disabled=false;$('play').disabled=false;
  $('time').oninput=()=>{playing=false;$('play').textContent='Putar';show(Number($('time').value));};
  $('play').onclick=()=>{playing=!playing;if(playing){if(index===data.frames.length-1)show(0);started=performance.now()/1000-data.frames[index].elapsed_s;}$('play').textContent=playing?'Jeda':'Putar';};
  for(const [id,node] of [['robot',robot],['points',points],['edges',lines],['frustum',optical]])$(id).onchange=()=>{node.visible=$(id).checked&&!(id==='frustum'&&sensorView);};
  document.querySelectorAll('[data-view]').forEach(b=>{b.disabled=false;b.onclick=()=>view(b.dataset.view);});
  $('capture').textContent=data.captured_at+' (rekaman)';
  const resize=()=>{const w=$('viewport').clientWidth,h=$('viewport').clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();};
  new ResizeObserver(resize).observe($('viewport'));resize();show(data.frames.length-1);view('iso');
  function tick(now){if(playing){const t=now/1000-started;let next=index;while(next+1<data.frames.length&&data.frames[next+1].elapsed_s<=t)next++;if(next!==index)show(next);if(t>=data.duration_s){playing=false;$('play').textContent='Putar';}}if(!sensorView)controls.update();renderer.render(scene,camera);requestAnimationFrame(tick);}requestAnimationFrame(tick);
}
main().catch(error=>{$('error').hidden=false;$('error').textContent='Preview gagal: '+error.message;$('stats').textContent='Gagal dimuat';});
