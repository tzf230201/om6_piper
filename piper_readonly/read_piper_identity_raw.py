"""Read-only diagnostic: one firmware query, independent of the SDK FPS gate.

This does not replace commissioning identity checks or establish motor state.
Uses the existing firmware-only TX allowlist, never configuration or control.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time

import can
from readonly_bus import QueuedReadOnlyGsUsbBus


def decode_identity(frames):
    """Strictly decode one 11-frame legacy identity response, or reject it."""
    if len(frames) != 11:
        raise ValueError('Expected exactly 11 firmware response frames')
    if any(f['id'] != '0x4AF' or len(bytes.fromhex(f['data_hex'])) != 8 for f in frames):
        raise ValueError('Invalid firmware frame identifier or length')
    data=b''.join(bytes.fromhex(f['data_hex']) for f in frames)
    result={name:data[a:b].decode('ascii') for name,a,b in (
        ('hardware_version',0,8),('motor_ratio_and_batch',16,18),
        ('node_type',32,38),('software_version',60,68),
        ('production_date',68,74),('node_number',76,78))}
    if not re.fullmatch(r'H-V\d\.\d-\d',result['hardware_version']):
        raise ValueError('Invalid legacy hardware-version header')
    if not re.fullmatch(r'S-V\d\.\d-\d',result['software_version']):
        raise ValueError('Invalid legacy software-version field')
    if result['node_type'] != 'ARM_MC':
        raise ValueError('Unexpected responding node')
    if not re.fullmatch(r'\d{6}',result['production_date']) or not result['node_number'].isdigit():
        raise ValueError('Invalid legacy node identity')
    return result


def read_window(bus,seconds):
    frames=[]
    until=time.perf_counter()+seconds
    while time.perf_counter()<until:
        msg=bus.recv(timeout=.02)
        if msg is None:
            continue
        frames.append(dict(id=f'0x{msg.arbitration_id:03X}',data_hex=bytes(msg.data).hex(),
            received_perf_s=bus.usb_receiver.last_received_perf,is_rx=msg.is_rx,
            is_error=msg.is_error_frame,is_extended=msg.is_extended_id,is_remote=msg.is_remote_frame))
    return frames


def console_summary(result,report_path):
    """Keep raw captures on disk; never flood the console with an error storm."""
    summary={key:result[key] for key in (
        'captured_at','state','error','execution_allowed','motor_state_known',
        'firmware_queries_attempted','firmware_queries_sent','blocked_commands',
        'firmware_identity','close_error') if key in result}
    summary['captured_frame_counts']={key:len(result.get(key,[]))
        for key in ('pre_query_frames','response_frames')}
    summary['transport_counts']=result.get('transport_health',{}).get('counts',{})
    summary['rx_ids']=result.get('rx_ids',{})
    summary['report_path']=str(Path(report_path).resolve())
    return summary


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    bus=None
    result=dict(captured_at=datetime.now().astimezone().isoformat(),execution_allowed=False,
        motor_state_known=False,firmware_queries_attempted=0,firmware_identity=None)
    try:
        bus=QueuedReadOnlyGsUsbBus(channel=0,bitrate=1000000,timeout=.02)
        result['pre_query_frames']=read_window(bus,1)
        result['query_started_perf_s']=time.perf_counter()
        result['firmware_queries_attempted']=1
        bus.send(can.Message(arbitration_id=0x4AF,data=[1],is_extended_id=False),timeout=.02)
        result['response_frames']=read_window(bus,2)
        firmware=[f for f in result['response_frames'] if f['id']=='0x4AF' and f['is_rx']
            and not (f['is_error'] or f['is_extended'] or f['is_remote'])
            and f['received_perf_s']>=result['query_started_perf_s']]
        result['firmware_identity']=decode_identity(firmware)
        result['state']='IDENTITY_RESPONSE_DECODED'
    except Exception as exc:
        result.update(state='DIAGNOSTIC_INCOMPLETE',error=f'{type(exc).__name__}: {exc}')
    finally:
        if bus is not None:
            result['transport_health']=bus.usb_receiver.diagnostics()
            result['firmware_queries_sent']=bus.queries_sent
            result['blocked_commands']=bus.blocked_commands
            result['rx_ids']={f'0x{k:03X}':v for k,v in bus.received.items()}
            try:
                bus.shutdown()
            except Exception as exc:
                result['close_error']=str(exc)
        (args.output/'report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(console_summary(result,args.output/'report.json'),indent=2),flush=True)
    return 0 if result['state']=='IDENTITY_RESPONSE_DECODED' else 2


if __name__=='__main__':
    raise SystemExit(main())
