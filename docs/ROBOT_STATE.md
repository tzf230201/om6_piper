# State robot yang tersedia

Sumber: `../../piper_readonly/read_robot.py`, `feedback_health.py`,
`web/joint_monitor.js`, dan parser pyAgxArm lokal. Ini pemetaan field, bukan
pembacaan live baru. Log historis: `../../output`, `../../piper_readonly/output`;
rekaman RGB-D/joint: `../output`.

| Kelompok | Field / CAN ID | Makna |
|---|---|---|
| J1--J6 | `joint_angles_rad`, `joint_angles_deg`; 0x2A5--0x2A7 | Sudut controller; raw milli-degree dibagi 1000 untuk GUI |
| Flange | `flange_pose_m_rad`; 0x2A2--0x2A4 | Pose flange, bukan otomatis TCP gripper atau pusat optik |
| Controller | `arm_status`; 0x2A1 | `ctrl_mode`, `arm_status`, `mode_feedback`, `teach_status`, `motion_status`, `trajectory_num`, `err_code`, `err_status` |
| Driver tiap joint | `drivers[0..5]`; 0x261--0x266 | Tegangan, temperatur driver/motor, status FOC dan bus current |
| Motor tiap joint | `motors[0..5]`; 0x251--0x256 | Velocity/current/position/torque menurut SDK; bukan pengganti sudut joint tanpa memeriksa satuan |
| Gripper | `gripper`; 0x2A8 | Width/value, force, mode, status code dan flag driver/sensor/homing |
| Identitas | `sdk`, `firmware`, `firmware_driver` | SDK, query versi dan profile parser; bukan readiness |
| Transport | `rx_ids`, `rx_frame_count`, `transport_health` | Jumlah RX, error/overflow/clock sejak koneksi |
| Validitas | `captured_at`, `rx_age_s`, `snapshot_perf_s`, `all_joint_feedback_fresh` | Waktu sampel dan validitas masing-masing kelompok |
| Audit | `raw_latest`, `frozen_control`, `receive_timing`, `firmware_queries_sent`, `blocked_commands` | Payload beku, hasil decode dan catatan guard |

`None`, `—`, stale atau tidak ada frame berarti unknown, bukan nol, disabled
atau normal. Snapshot lama tetap historis. `frozen_control` adalah cache host
yang konsisten, bukan sampling serentak keenam joint secara hardware.

## Bit status driver

Pemetaan SDK lokal untuk `drivers[i].foc_status_code`:

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

Contoh decode saja: 0x30 berarti bit collision dan driver-error aktif, bukan
bukti penyebab fisik atau kondisi sekarang. 0x40 berarti enable flag, bukan fault.
Controller `err_code` nol tidak membatalkan fault driver. Komunikasi per joint
memakai bit rendah `err_code`, angle-limit memakai bit 8--13 menurut parser GUI.

Monitor menampilkan speed rad/s, angle derajat, voltage V, current motor mA dari
raw frame, dan temperatur °C. Jangan menyamakan raw mA dengan field SDK tanpa
memeriksa parser. SDK menyebut sebagian motor tidak menyediakan bus-current
sampling dan mengirim nol; nol bukan bukti motor mati. Flag gripper berbeda:
bit 4 sensor status dan bit 7 homing, bukan collision/stall seperti joint.

## Freshness dan ekspor

Reader umum memeriksa umur frame kurang dari 0,5 s. Tabel Monitor dan policy
maintenance tertentu memakai 150 ms. Itu batas lokal software, bukan spesifikasi
vendor atau jaminan realtime. Freeze membekukan tampilan, bukan robot. Ekspor
CSV/snapshot harus mempertahankan timestamp dan label historis.

`piper.cmd joints` membaca viewer yang sudah berjalan. Print nominal 50 Hz bukan
bukti acquisition 50 Hz. `--source can` memakai adapter secara eksklusif, jangan
bersamaan dengan viewer. Snapshot baru tanpa enable/gerak:

```powershell
.\piper.cmd state --seconds 5 --output .\output\state_baru.json
```

Perintah tersebut tidak dijalankan dalam konsolidasi ini.
