# Figure and screenshot provenance

All images are local repository assets. No generated photograph or fabricated
robot state is used. `provenance.json` records image and input SHA-256 values.
Source paths below are relative to the parent ICRA2027 workspace.

| Asset | Origin and interpretation |
| --- | --- |
| `robot-camera-overview.png` | Unchanged `output/cad_preview_20261005_v1/video_preview.png`. Existing nominal-CAD visualization of stationary recorded observations; includes the legacy +20 mm presentation offset. |
| `archived-surface-views.png` | Unchanged `output/delta_archive_20261009_v1/video/final_map.png`. Final offline archive after draining the writer, not an early live frame. |
| `viewer-snapshot.png` | Actual full-page browser screenshot of `web_monitor.py --snapshot`, captured for the README. |
| `joint-monitor-snapshot.png` | Actual Monitor screenshot from the same offline viewer. Fault flags remain as recorded. |
| `architecture.svg` | Editable vector diagram of code paths. Live capture and offline archive research are intentionally separate. |

Both screenshots use `output/motion_preflight_20261005_v1/robot_status.json`.
They document the UI, not the current robot state. The temporary viewer did not
connect to CAN, acquire camera frames or send motor commands.

Do not remove snapshot/final-archive labels when reusing these figures. See
[Mechanical and CAD](../MECHANICAL_CAD.md) for the difference between visual
offsets and the physical mapping frame.
