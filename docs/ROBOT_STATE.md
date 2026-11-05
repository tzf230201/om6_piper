# Available robot state

Sources: `../../piper_readonly/read_robot.py`, `feedback_health.py`,
`web/joint_monitor.js`, and the local pyAgxArm parser. This describes field mappings,
not a new live reading. Historical logs: `../../output`, `../../piper_readonly/output`;
RGB-D/joint recordings: `../output`.

| Group | Field / CAN ID | Meaning |
|---|---|---|
| J1--J6 | `joint_angles_rad`, `joint_angles_deg`; 0x2A5--0x2A7 | Controller angles; raw millidegrees divided by 1000 for the GUI |
| Flange | `flange_pose_m_rad`; 0x2A2--0x2A4 | Flange pose; not automatically the gripper TCP or optical center |
| Controller | `arm_status`; 0x2A1 | `ctrl_mode`, `arm_status`, `mode_feedback`, `teach_status`, `motion_status`, `trajectory_num`, `err_code`, `err_status` |
| Per-joint driver | `drivers[0..5]`; 0x261--0x266 | Voltage, driver/motor temperature, FOC status and bus current |
| Per-joint motor | `motors[0..5]`; 0x251--0x256 | SDK velocity/current/position/torque; check units before using these as joint angles |
| Gripper | `gripper`; 0x2A8 | Width/value, force, mode, status code and driver/sensor/homing flags |
| Identity | `sdk`, `firmware`, `firmware_driver` | SDK, version query and parser profile; does not establish readiness |
| Transport | `rx_ids`, `rx_frame_count`, `transport_health` | RX counts and errors/overflow/clock information since connection |
| Validity | `captured_at`, `rx_age_s`, `snapshot_perf_s`, `all_joint_feedback_fresh` | Sample times and validity of each group |
| Audit | `raw_latest`, `frozen_control`, `receive_timing`, `firmware_queries_sent`, `blocked_commands` | Frozen payloads, decoded results and guard records |

`None`, `—`, stale or missing frames mean unknown, not zero, disabled or normal.
Old snapshots remain historical. `frozen_control` is a consistent host cache,
not simultaneous hardware sampling of all six joints.

## Driver status bits

Local SDK mapping for `drivers[i].foc_status_code`:

| Bit | Mask | Field |
|---:|---:|---|
| 0 | 0x01 | `voltage_too_low` |
| 1 | 0x02 | `motor_overheating` |
| 2 | 0x04 | `driver_overcurrent` |
| 3 | 0x08 | `driver_overheating` |
| 4 | 0x10 | `collision_status` |
| 5 | 0x20 | `driver_error_status` |
| 6 | 0x40 | `driver_enable_status` |
| 7 | 0x80 | `stall_status` |

Decoding example only: 0x30 indicates active collision and driver-error bits; it
does not establish a physical cause or current condition. 0x40 is an enable flag,
not a fault. A zero controller `err_code` does not cancel driver faults. The GUI
parser uses the low `err_code` bits for per-joint communication and bits 8--13
for angle limits.

The Monitor displays speed in rad/s, angles in degrees, voltage in V, raw-frame
motor current in mA and temperature in °C. Check the parser before equating raw mA
with SDK fields. The SDK notes that some motors do not sample bus current and
send zero; zero does not prove that a motor is off. Gripper flags differ: bit 4
is sensor status and bit 7 is homing, rather than joint collision/stall.

## Freshness and export

The general reader checks for frames younger than 0.5 s. The Monitor table and
some maintenance policies use 150 ms. These are local software thresholds, not
vendor specifications or realtime guarantees. Freeze holds the display, not the
robot. CSV/snapshot exports must preserve timestamps and historical labels.

`piper.cmd joints` reads an existing viewer. A nominal 50 Hz print rate does not
prove 50 Hz acquisition. `--source can` uses the adapter exclusively; do not run
it alongside the viewer. To take a new snapshot without enabling or moving:

```powershell
.\piper.cmd state --seconds 5 --output .\output\state_baru.json
```

This command was not executed during consolidation.
