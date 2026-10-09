import * as THREE from 'three';

export function graphGeometry(data) {
  const nodes=data.nodes??[], edges=data.edges??[], support=data.support??[];
  if(![nodes,support].every(points=>Array.isArray(points)&&points.every(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite))))throw Error('Invalid graph XYZ');
  if(!Array.isArray(edges)||!edges.every(e=>e.length===2&&e.every(i=>Number.isInteger(i)&&i>=0&&i<nodes.length)))throw Error('Invalid graph edges');
  const paths=data.edge_paths;
  if(paths&&(!Array.isArray(paths)||paths.length!==edges.length||!paths.every(p=>p.length>=2&&p.every(x=>x.length===3&&x.every(Number.isFinite)))))throw Error('Invalid surface paths');
  const lines=paths?paths.flatMap(path=>path.slice(1).flatMap((p,i)=>[...path[i],...p])):edges.flatMap(([a,b])=>[...nodes[a],...nodes[b]]);
  return {nodes:new Float32Array(nodes.flat()),support:new Float32Array(support.flat()),
    edges:new Float32Array(lines)};
}

export function mappingSummary(data) {
  const s=data.stats;
  if(!s)return data.message??'';
  if(data.algorithm==='hybrid-voxel-fgng-v3') {
    const age=s.graph_age_frames===null?'warming':`cache ${s.graph_age_frames} frame`;
    return `${data.message} · map ${s.geometry_voxels} voxel @ ${(s.geometry_voxel_m*1000).toFixed(1)} mm · view ${s.display_points} titik · graf ${data.nodes.length}/${data.edges.length} (${age}) · ${Math.round(s.pipeline_ms)} ms`;
  }
  return data.message+` · ${data.nodes.length} nodes · ${data.edges.length} edges`+(s.components!==undefined?` · ${s.represented_components}/${s.components} komponen · ${Math.round(s.pipeline_ms)} ms`:'');
}

export function createFGNG(scene) {
  const group=new THREE.Group();group.name='world_fgng';scene.add(group);
  const cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0x71abb5,size:.003,transparent:true,opacity:.7}));
  const edges=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x46c7ce,transparent:true,opacity:.8}));
  const nodes=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0xffce70,size:.005}));
  const optical=new THREE.AxesHelper(.055);optical.visible=false;
  const baseline=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xe86677,transparent:true,opacity:.35}));
  group.add(cloud,edges,nodes,optical,baseline);
  let lastFrame=-1,lastReceived=0,lastMessage='',lastLive=false;
  const status=document.getElementById('fgng-note');
  function visibility() {
    group.visible=document.getElementById('show-fgng').checked;
    cloud.visible=document.getElementById('show-cloud').checked;
    baseline.visible=document.getElementById('show-native').checked;
  }
  document.getElementById('show-fgng').onchange=visibility;
  document.getElementById('show-cloud').onchange=visibility;
  document.getElementById('show-native').onchange=visibility;
  async function poll() {
    try {
      const response=await fetch('/api/fgng',{cache:'no-store',signal:AbortSignal.timeout(2000)});
      if(!response.ok)throw Error('HTTP '+response.status);
      const data=await response.json();
      lastReceived=performance.now();lastLive=!!data.live;
      lastMessage=mappingSummary(data);
      if(data.frames!==undefined&&data.frames!==lastFrame) {
        const arrays=graphGeometry(data);
        for(const [object,array] of [[nodes,arrays.nodes],[edges,arrays.edges],[cloud,arrays.support]]) {
          object.geometry.dispose();object.geometry=new THREE.BufferGeometry();
          object.geometry.setAttribute('position',new THREE.BufferAttribute(array,3));
          if(array.length)object.geometry.computeBoundingSphere();
        }
        baseline.geometry.dispose();baseline.geometry=new THREE.BufferGeometry();
        const raw=graphGeometry({nodes:data.native_nodes??data.nodes,edges:data.native_edges??[]});
        baseline.geometry.setAttribute('position',new THREE.BufferAttribute(raw.edges,3));
        if(raw.edges.length)baseline.geometry.computeBoundingSphere();
        if(data.T_world_depth_optical) {
          optical.matrix.fromArray(data.T_world_depth_optical.flat()).transpose();
          optical.matrix.decompose(optical.position,optical.quaternion,optical.scale);optical.visible=true;
        }
        lastFrame=data.frames;
      }
    } catch(e) {lastLive=false;lastMessage='F-GNG terputus · peta terakhir ditahan';}
    status.textContent=lastMessage;status.classList.toggle('warning',!lastLive);
    setTimeout(poll,200);
  }
  setInterval(()=>{
    if(lastReceived&&performance.now()-lastReceived>1500){status.textContent='F-GNG stale · peta terakhir ditahan';status.classList.add('warning');}
  },500);
  visibility();poll();return group;
}
