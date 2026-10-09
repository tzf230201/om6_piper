import test from 'node:test';
import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {applyFeedback, buildTree, feedbackUsable, feedbackValues} from './model.js';
import * as THREE from 'three';
import {createRobotDisplay, displayPositionToBase} from './model.js';

const python=process.env.PIPER_PYTHON || fileURLToPath(new URL('../../../piper_readonly/.venv/Scripts/python.exe',import.meta.url));
const fixture=JSON.parse(execFileSync(python,['-c',`
import json
from robot_model import load_model
from test_web_monitor import independent_fk
model,assets=load_model('S-V1.5-7')
values=dict(zip(['joint'+str(i) for i in range(1,9)], [.7,-.01,.028,.15,.16,-.14,.02,-.02]))
poses=independent_fk(model,values)
print(json.dumps({'model':model,'values':values,'poses':{k:v.T.flatten().tolist() for k,v in poses.items()},'assets':{k:str(v) for k,v in assets.items()}}))
`],{encoding:'utf8',cwd:fileURLToPath(new URL('../',import.meta.url))}));

test('every link transform agrees with independent SciPy URDF FK',()=>{
  const tree=buildTree(fixture.model);applyFeedback(tree,fixture.values);
  for(const [name,expected] of Object.entries(fixture.poses)) {
    tree.links[name].matrixWorld.elements.forEach((v,i)=>assert.ok(Math.abs(v-expected[i])<1e-10,`${name}[${i}] ${v} != ${expected[i]}`));
  }
});

test('full measured jaw width splits into signed prismatic joints',()=>{
  const values=feedbackValues({gripper:{mode:'width',value:.04}});
  assert.equal(values.joint7,.02);assert.equal(values.joint8,-.02);
  assert.deepEqual(feedbackValues({gripper:null}),{});
});

test('measured out-of-limit angle is preserved, not clamped',()=>{
  const tree=buildTree(fixture.model);applyFeedback(tree,{joint2:-.011});
  assert.ok(Math.abs(tree.links.link2.quaternion.z-Math.sin(-.011/2))<1e-12);
});

test('stale or incomplete feedback cannot drive model; snapshot explicitly allowed',()=>{
  const data={all_joint_feedback_fresh:true,joint_angles_rad:[0,0,0,0,0,0]};
  assert.equal(feedbackUsable({data,live:true}),true);
  assert.equal(feedbackUsable({data,live:false}),false);
  assert.equal(feedbackUsable({data,snapshot:true}),true);
  assert.equal(feedbackUsable({data:{...data,joint_angles_rad:[0,0]},live:true}),false);
});

test('all official and wrist CAD STL files parse with finite metre-scale bounds',()=>{
  const loader=new STLLoader();
  assert.equal(Object.keys(fixture.assets).length,13);
  for(const [url,path] of Object.entries(fixture.assets)) {
    const bytes=readFileSync(path);
    const geo=loader.parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    geo.computeBoundingBox();assert.ok(geo.attributes.position.count>0,url);
    const box=geo.boundingBox;
    for(const value of [...box.min.toArray(),...box.max.toArray()])assert.ok(Number.isFinite(value)&&Math.abs(value)<2,url);
    geo.dispose();
  }
});

test('camera follows J6 rigidly and is independent of finger opening',()=>{
  const tree=buildTree(fixture.model);
  applyFeedback(tree,{...fixture.values,joint6:0,joint7:0,joint8:0});
  const before=tree.links.d405_camera_body.matrixWorld.clone();
  const local=tree.links.link6.matrixWorld.clone().invert().multiply(before);
  applyFeedback(tree,{joint6:.7,joint7:.035,joint8:-.035});
  const after=tree.links.d405_camera_body.matrixWorld.clone();
  assert.ok(before.elements.some((v,i)=>Math.abs(v-after.elements[i])>1e-3));
  const current=tree.links.link6.matrixWorld.clone().invert().multiply(after);
  current.elements.forEach((v,i)=>assert.ok(Math.abs(v-local.elements[i])<1e-10));
});

test('display lift moves every robot link 20 mm once, preserving base telemetry and flange delta',()=>{
  const original=buildTree(fixture.model), shown=buildTree(fixture.model);
  const marker=new THREE.Group();
  const display=createRobotDisplay(shown,marker);
  for(const values of [fixture.values,{...fixture.values,joint6:.8}]) {
    applyFeedback(original,values);applyFeedback(shown,values);
    const measured=original.links.link6.getWorldPosition(new THREE.Vector3()).add(new THREE.Vector3(.001,0,0));
    marker.position.copy(measured);
    display.updateMatrixWorld(true);
    for(const name of Object.keys(original.links)) {
      const a=original.links[name].getWorldPosition(new THREE.Vector3());
      const b=shown.links[name].getWorldPosition(new THREE.Vector3());
      assert.ok(b.clone().sub(a).distanceTo(new THREE.Vector3(0,0,.02))<1e-12,name);
      assert.ok(displayPositionToBase(b).distanceTo(a)<1e-12,name);
    }
    const flange=shown.links.link6.getWorldPosition(new THREE.Vector3());
    assert.ok(Math.abs(flange.distanceTo(marker.getWorldPosition(new THREE.Vector3()))-.001)<1e-12);
  }
  assert.deepEqual(shown.root.position.toArray(),[0,0,0]);
  assert.deepEqual(display.position.toArray(),[0,0,.02]);
});
