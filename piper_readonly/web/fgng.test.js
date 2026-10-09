import test from 'node:test';
import assert from 'node:assert/strict';
import {graphGeometry,mappingSummary} from './fgng.js';

test('world graph coordinates are rendered unchanged, without second camera transform',()=>{
  const g=graphGeometry({nodes:[[1,2,.02],[2,3,.12]],edges:[[0,1]],support:[[1,2,.02]]});
  assert.deepEqual([...g.edges], [...g.nodes]);
  assert.equal(g.support.length,3);
});
test('invalid nodes and edges rejected before GPU upload',()=>{
  assert.throws(()=>graphGeometry({nodes:[[NaN,1,2]]}));
  assert.throws(()=>graphGeometry({nodes:[[0,0,0]],edges:[[0,1]]}));
  assert.equal(graphGeometry({}).nodes.length,0);
});

test('surface paths render intermediate vertices instead of chords across holes',()=>{
  const g=graphGeometry({nodes:[[0,0,0],[1,0,0]],edges:[[0,1]],edge_paths:[[[0,0,0],[.5,1,0],[1,0,0]]]});
  assert.deepEqual([...g.edges],[0,0,0,.5,1,0,.5,1,0,1,0,0]);
  assert.throws(()=>graphGeometry({nodes:[[0,0,0]],edges:[],edge_paths:[[[0,0,0]]]}));
});

test('hybrid distinguishes metric map from sampled display and cached graph',()=>{
  const data={algorithm:'hybrid-voxel-fgng-v3',message:'Hybrid',nodes:[],edges:[],support:[[1,2,.02]],
    stats:{geometry_voxels:100000,geometry_voxel_m:.008,display_points:16000,graph_age_frames:null,pipeline_ms:12}};
  assert.equal(graphGeometry(data).support.length,3);
  assert.match(mappingSummary(data),/map 100000 voxel @ 8.0 mm/);
  assert.match(mappingSummary(data),/view 16000 titik/);
  assert.match(mappingSummary(data),/warming/);
  data.stats.graph_age_frames=4;
  assert.match(mappingSummary(data),/cache 4 frame/);
});
