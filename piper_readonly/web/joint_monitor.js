// Read-only presentation. No CAN writes or control HTTP requests.
export const ROWS=[
  ['code','Status code','hex'], ['speed','Motor speed','rad/s'],
  ['angle','Joint angle','°'], ['voltage','Voltage','V'],
  ['current','Motor current','mA'], ['motorTemp','Motor temperature','°C'],
  ['driverTemp','Driver temperature','°C'], ['collision','Collision flag',''],
  ['stall','Stall / blockage flag',''], ['error','Driver error flag',''],
  ['enabled','Enable state',''], ['comm','Communication flag',''],
  ['limit','Angle limit flag',''], ['age','Driver feedback age','ms']
];
const unknown=()=>({text:'—',tone:'unknown'});
const numeric=(v,n=0)=>({text:Number.isFinite(v)?v.toFixed(n):'—',tone:Number.isFinite(v)?'value':'unknown'});
const flag=v=>({text:v?'Active':'Normal',tone:v?'fault':'ok'});

function frame(packet,id) {
  if(!packet||packet.error||(!packet.live&&!packet.snapshot))return null;
  const d=packet.data, age=d?.rx_age_s?.[id], sourceAge=packet.sample_age_s;
  if(!Number.isFinite(age)||age<0||!Number.isFinite(sourceAge)||sourceAge<0)return null;
  const total=age+(packet.snapshot?0:sourceAge);
  if(total>.15)return null;
  const raw=d?.raw_latest?.[id];
  if(typeof raw!=='string'||!/^[0-9a-f]{16}$/i.test(raw))return null;
  const bytes=Uint8Array.from(raw.match(/../g),s=>parseInt(s,16));
  return {v:new DataView(bytes.buffer),age:total};
}

export function monitorColumns(packet) {
  const controller=frame(packet,'0x2A1');
  return Array.from({length:6},(_,i)=>{
    const values=Object.fromEntries(ROWS.map(([key])=>[key,unknown()]));
    const driver=frame(packet,'0x'+(0x261+i).toString(16));
    const motor=frame(packet,'0x'+(0x251+i).toString(16));
    const joint=frame(packet,'0x'+(0x2A5+Math.floor(i/2)).toString(16).toUpperCase());
    if(joint)values.angle=numeric(joint.v.getInt32((i%2)*4)/1000,3);
    if(motor){values.speed=numeric(motor.v.getInt16(0)/1000,3);values.current=numeric(motor.v.getInt16(2));}
    if(driver){
      const code=driver.v.getUint8(5);
      values.code={text:'0x'+code.toString(16).toUpperCase().padStart(2,'0'),tone:code&0xBF?'fault':'value'};
      values.voltage=numeric(driver.v.getUint16(0)/10,1);
      values.motorTemp=numeric(driver.v.getInt8(4));values.driverTemp=numeric(driver.v.getInt16(2));
      values.collision=flag(!!(code&16));values.stall=flag(!!(code&128));values.error=flag(!!(code&32));
      values.enabled={text:code&64?'Enabled':'Disabled',tone:code&64?'ok':'disabled'};
      values.age=numeric(driver.age*1000,1);
    }
    if(controller){
      // Matches installed pyAgxArm err_status: low byte comm; high byte angle.
      const err=controller.v.getUint16(6);
      values.comm=flag(!!(err&(1<<i)));
      values.limit=flag(!!(err&(1<<(8+i))));
    }
    return values;
  });
}

export function monitorCSV(packet,columns,frozen=false) {
  const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  const rows=[['sample_time',packet?.data?.captured_at],['display',frozen?'frozen / historical':packet?.snapshot?'offline snapshot':'telemetry snapshot'],
    ['parameter','unit','J1','J2','J3','J4','J5','J6'],
    ...ROWS.map(([key,label,unit])=>[label,unit,...columns.map(c=>c[key].text)])];
  return rows.map(row=>row.map(quote).join(',')).join('\r\n');
}

export function createJointMonitor() {
  const $=id=>document.getElementById(id), cells=new Map();
  let latest=null,displayed=null,receipt=0,columns=monitorColumns(null);
  for(const [key,label,unit] of ROWS){
    const row=document.createElement('tr'),head=document.createElement('th');head.scope='row';
    head.textContent=label;
    if(unit){const small=document.createElement('small');small.textContent=unit;head.append(small);}
    row.append(head);const values=[];
    for(let i=0;i<6;i++){const cell=document.createElement('td');cell.textContent='—';row.append(cell);values.push(cell);}
    cells.set(key,values);$('monitor-rows').append(row);
  }
  function render(){
    const frozen=$('monitor-freeze').checked;
    if(!frozen){displayed=latest?{...latest,sample_age_s:latest.sample_age_s+(performance.now()-receipt)/1000}:null;columns=monitorColumns(displayed);}
    for(const [key] of ROWS)cells.get(key).forEach((cell,i)=>{cell.textContent=columns[i][key].text;cell.className='status-'+columns[i][key].tone;});
    const status=$('monitor-status');
    const fresh=columns.every(c=>c.code.tone!=='unknown'&&c.angle.tone!=='unknown'&&c.speed.tone!=='unknown'&&c.comm.tone!=='unknown');
    status.textContent=frozen?'FROZEN / HISTORICAL':displayed?.snapshot?'SNAPSHOT / OFFLINE':displayed?.live&&!displayed.error&&fresh?'LIVE TELEMETRY':'STALE / PARTIAL FEEDBACK';
    status.classList.toggle('live',!frozen&&!!displayed?.live&&!displayed.error&&fresh);
    $('monitor-view').classList.toggle('frozen',frozen);
    $('monitor-time').textContent=displayed?.data?.captured_at?'Sampel '+new Date(displayed.data.captured_at).toLocaleTimeString('id-ID',{hour12:false})+(frozen?' · bukan kondisi saat ini':''):'Belum ada sampel';
    $('monitor-export').disabled=!displayed?.data;
  }
  function selectTab(monitor){
    $('monitor-view').hidden=!monitor;document.querySelector('main').classList.toggle('monitor-mode',monitor);
    for(const [id,active] of [['menu-monitor',monitor],['menu-3d',!monitor]]){$(id).classList.toggle('active',active);$(id).setAttribute('aria-pressed',String(active));}
    const url=new URL(location.href);if(monitor)url.searchParams.set('tab','monitor');else url.searchParams.delete('tab');history.replaceState(null,'',url);
  }
  $('menu-monitor').onclick=()=>selectTab(true);$('menu-3d').onclick=()=>selectTab(false);
  $('monitor-freeze').onchange=render;
  $('monitor-export').onclick=()=>{
    if(!displayed?.data)return;
    const url=URL.createObjectURL(new Blob(['\uFEFF'+monitorCSV(displayed,columns,$('monitor-freeze').checked)],{type:'text/csv;charset=utf-8'}));
    const a=document.createElement('a');a.href=url;a.download='piper_joint_state_'+new Date().toISOString().replaceAll(':','-')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  selectTab(new URL(location.href).searchParams.get('tab')==='monitor');render();
  setInterval(()=>{if(!$('monitor-freeze').checked)render();},100);
  return {update(packet){latest=packet;receipt=performance.now();render();}};
}
