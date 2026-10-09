import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {buildTree,applyFeedback,urdfQuaternion} from './model.js';

const dir=new URL('../../../output/'+(process.env.PIPER_SCAN_FIXTURE||'trajectory_scan_20261005_v1')+'/',import.meta.url);
const read=name=>JSON.parse(fs.readFileSync(new URL(name,dir),'utf8'));
const plan=read('plan.json'),model=read('model.json');

test('all 7201 scan positions AND optical directions match the frontend FK',()=>{
  const tree=buildTree(model),display=new THREE.Group(),optical=new THREE.Group();
  display.position.fromArray(plan.display_offset_m);display.add(tree.root);
  optical.position.fromArray(plan.optical_extrinsic.xyz);
  optical.quaternion.copy(urdfQuaternion(plan.optical_extrinsic.rpy));
  tree.links.d405_camera_body.add(optical);
  assert.deepEqual(plan.display_offset_m,[0,0,.02]);
  assert.equal(plan.samples.length,7201);
  const actual=new THREE.Vector3(),forward=new THREE.Vector3();
  for(const s of plan.samples){
    applyFeedback(tree,Object.fromEntries(s.q_rad.map((q,i)=>['joint'+(i+1),q])));
    display.updateMatrixWorld(true);optical.getWorldPosition(actual);optical.getWorldDirection(forward);
    const expected=new THREE.Vector3(...s.camera_base_xyz_m).add(display.position);
    assert.ok(actual.distanceTo(expected)<1e-10,`Position mismatch at t=${s.t_s}`);
    assert.ok(forward.distanceTo(new THREE.Vector3(...s.camera_base_forward))<1e-10,`Direction mismatch at t=${s.t_s}`);
  }
});

test('new export is simulation-only and does not redirect to old model assets',()=>{
  assert.equal(plan.execution_allowed,false);assert.equal(plan.preflight.execution_allowed,false);
  assert.equal(plan.live,false);assert.equal(plan.hardware_motion_commands_sent,0);
  for(const link of model.links)for(const v of link.visuals)assert.ok(v.url.startsWith('/trajectory/scan/mesh/'));
  assert.deepEqual(plan.samples[0].q_rad,plan.samples.at(-1).q_rad);
});

test('snapshot provenance, starting pose and gripper are from the same capture',()=>{
  const snapshot=read('robot_status.json');
  assert.equal(plan.source_snapshot_at,snapshot.captured_at);
  assert.deepEqual(plan.samples[0].q_rad,snapshot.joint_angles_rad);
  // Old exports are preserved; new exports carry this rebase field.
  if(plan.previous_start_delta_deg)assert.deepEqual(read('gripper.json'),snapshot.gripper);
});
