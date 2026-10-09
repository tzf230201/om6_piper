# Source guide

The current code uses script-style Python modules. It is not an independently
installable Python package; shared SDKs, models, native builds and evidence still
live in the parent workspace. `workspace_paths.py` makes that boundary explicit.

| Area | Entry points |
| --- | --- |
| Operator-run motion | `trajectory_test.py` — physical motion after confirmation |
| RGB-D capture | `vision_capture.py`, `vision_motion_profile.py` |
| Online mapping | `live_mapping.py` |
| Offline trajectory planning | `table_scan.py`, `preview_table_scan.py` |
| Replay and capture review | `replay_robot_mapping.py`, `replay_live_mapping.py`, `review_live_scan.py` |
| Archive experiments | `archive_retention_study.py`, `block_archive_study.py`, `analyze_delta_archive.py` |
| Regional retrieval | `regional_read_study.py`, `regional_read_job.py`, `regional_job_study.py` |
| Demo rendering | `render_*.py` |
| Frozen-delivery checks | `verify_*.py` — run-specific, not general health checks |

Use [the launcher](../README.md#quick-start-existing-workspace) for common tasks.
When calling a source file directly, use `src/<name>.py` and preserve the intended
working directory for relative dataset/output arguments. Historical commands in
the development log predate this layout; they are evidence, not current setup instructions.
