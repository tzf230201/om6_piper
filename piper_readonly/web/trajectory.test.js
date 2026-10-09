import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {buildTree,applyFeedback,urdfQuaternion} from './model.js';

const directory=new URL('../../../output/trajectory_wrist_20261005_v1/',import.meta.url);
const read=name=>JSON.parse(fs.readFileSync(new URL(name,directory),'utf8'));
const plan=read('plan.json'),model=read('model.json');

test('all 2101 camera positions agree with frontend FK, with display lift exactly once',()=>{
  const tree=buildTree(model),display=new THREE.Group(),optical=new THREE.Group();
  display.position.fromArray(plan.display_offset_m);display.add(tree.root);
  optical.position.fromArray(plan.optical_extrinsic.xyz);
  optical.quaternion.copy(urdfQuaternion(plan.optical_extrinsic.rpy));
  tree.links.d405_camera_body.add(optical);
  assert.deepEqual(plan.display_offset_m,[0,0,.02]);
  assert.equal(plan.samples.length,2101);
  const actual=new THREE.Vector3();
  for(const s of plan.samples){
    applyFeedback(tree,Object.fromEntries(s.q_rad.map((q,i)=>['joint'+(i+1),q])));
    display.updateMatrixWorld(true);optical.getWorldPosition(actual);
    const expected=new THREE.Vector3(...s.camera_base_xyz_m).add(display.position);
    assert.ok(actual.distanceTo(expected)<1e-10,`FK mismatch at t=${s.t_s}`);
  }
});

test('frozen candidate cannot be mistaken for live data or an executable plan',()=>{
  assert.equal(plan.kind,'offline_kinematic_simulation');
  assert.equal(plan.live,false);
  assert.equal(plan.execution_allowed,false);
  assert.equal(plan.preflight.execution_allowed,false);
  assert.equal(plan.hardware_motion_commands_sent,0);
  assert.equal(plan.gripper_commanded,false);
  assert.deepEqual(plan.samples[0].q_rad,plan.samples.at(-1).q_rad);
  assert.deepEqual(plan.samples[0].camera_base_xyz_m,plan.samples.at(-1).camera_base_xyz_m);
  assert.ok(plan.preflight.reasons.length>0);
});
