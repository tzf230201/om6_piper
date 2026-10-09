import test from 'node:test';
import assert from 'node:assert/strict';
import {monitorColumns,monitorCSV} from './joint_monitor.js';
function packet(){return {live:true,sample_age_s:.01,data:{captured_at:'2026-10-07T15:00:00+09:00',
  rx_age_s:{'0x261':.02,'0x251':.01,'0x2A5':.01,'0x2A1':.01},
  raw_latest:{'0x261':'00e6003026400000','0x251':'fffd001000000000','0x2A5':'00003aaeffffff2c','0x2A1':'0100000000000000'}}};}
test('raw units, signed motor speed/current and temperatures',()=>{
  const v=monitorColumns(packet())[0];
  assert.equal(v.code.text,'0x40');assert.equal(v.voltage.text,'23.0');assert.equal(v.motorTemp.text,'38');assert.equal(v.driverTemp.text,'48');
  assert.equal(v.speed.text,'-0.003');assert.equal(v.current.text,'16');assert.equal(v.angle.text,'15.022');assert.equal(v.enabled.text,'Enabled');assert.equal(v.age.text,'30.0');
});
test('unknown fields do not look like normal zero',()=>{
  const v=monitorColumns(packet())[2];assert.equal(v.code.text,'—');assert.equal(v.enabled.tone,'unknown');assert.equal(v.angle.text,'—');
});
test('stale per-type driver leaves independently fresh angle intact',()=>{
  const p=packet();p.data.rx_age_s['0x261']=.2;const v=monitorColumns(p)[0];assert.equal(v.collision.text,'—');assert.equal(v.angle.text,'15.022');
});
test('network stale and negative timestamp do not claim normal flags',()=>{
  const p=packet();p.live=false;assert.equal(monitorColumns(p)[0].comm.text,'—');p.live=true;p.data.rx_age_s['0x261']=-1;assert.equal(monitorColumns(p)[0].enabled.text,'—');
});
test('collision/stall/error are decoded separately from enabled',()=>{
  const p=packet();p.data.raw_latest['0x261']='00e6003026f00000';const v=monitorColumns(p)[0];assert.equal(v.collision.text,'Active');assert.equal(v.error.text,'Active');assert.equal(v.stall.text,'Active');assert.equal(v.enabled.text,'Enabled');
});
test('SDK err_code low-byte comm and high-byte angle mapping',()=>{
  const p=packet();p.data.raw_latest['0x2A1']='0100000000000201';const v=monitorColumns(p);assert.equal(v[0].comm.text,'Active');assert.equal(v[0].limit.text,'Normal');assert.equal(v[1].limit.text,'Active');assert.equal(v[1].comm.text,'Normal');
});
test('CSV marks frozen snapshot historical',()=>{
  const p=packet();const csv=monitorCSV(p,monitorColumns(p),true);assert.match(csv,/frozen \/ historical/);assert.match(csv,/"Motor current","mA","16"/);
});
