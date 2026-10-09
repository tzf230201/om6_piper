import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {buildTree,applyFeedback,urdfQuaternion} from './model.js';
const directory=new URL('../../../output/cad_preview_20261005_v1/',import.meta.url);
const data=JSON.parse(fs.readFileSync(new URL('scene.json',directory),'utf8'));
const matrix=new THREE.Matrix4().set(...data.camera_to_display.flat());
test('recorded CAD transform matches frontend FK; world lift is applied once',()=>{
  assert.equal(data.live,false);assert.equal(data.calibrated,false);
  const tree=buildTree(data.model),display=new THREE.Group();
  display.position.fromArray(data.robot_display_offset);display.add(tree.root);
  applyFeedback(tree,data.joint_values);display.updateMatrixWorld(true);
  const optical=new THREE.Matrix4().compose(new THREE.Vector3(...data.optical_extrinsic.xyz),urdfQuaternion(data.optical_extrinsic.rpy),new THREE.Vector3(1,1,1));
  const actual=tree.links.d405_camera_body.matrixWorld.clone().multiply(optical);
  for(let i=0;i<16;i++)assert.ok(Math.abs(actual.elements[i]-matrix.elements[i])<1e-12);
  assert.deepEqual(data.robot_display_offset,[0,0,.02]);
});
test('sensor-view camera uses optical X right, Y down, Z forward',()=>{
  const camera=new THREE.PerspectiveCamera();
  camera.position.setFromMatrixPosition(matrix);
  camera.quaternion.setFromRotationMatrix(matrix).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI));
  camera.updateMatrixWorld(true);
  for(const [x,y,z] of [[0,0,.3],[.1,.05,.4],[-.2,-.1,1]]){
    const actual=new THREE.Vector3(x,y,z).applyMatrix4(matrix).applyMatrix4(camera.matrixWorldInverse);
    assert.ok(actual.distanceTo(new THREE.Vector3(x,-y,-z))<1e-12);
  }
});
test('all 60 binary graph frames have valid finite coordinates and endpoints',()=>{
  const bytes=fs.readFileSync(new URL('frames.bin',directory));
  assert.equal(data.frames.length,60);let end=0,last=-1;
  for(const f of data.frames){
    assert.equal(f.byte_offset,end);assert.ok(f.elapsed_s>last);last=f.elapsed_s;
    for(let i=0;i<f.nodes*3;i++)assert.ok(Number.isFinite(bytes.readFloatLE(end+i*4)));
    end+=f.nodes*12;
    for(let i=0;i<f.edges*2;i++)assert.ok(bytes.readUInt32LE(end+i*4)<f.nodes);
    end+=f.edges*8;
  }
  assert.equal(end,bytes.length);
});
