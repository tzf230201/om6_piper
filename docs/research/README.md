# Research and demonstrations

The [development log](DEVELOPMENT_LOG.md) preserves the previous README byte for
byte. Paths and commands in it describe the workspace at that time. Current
source is now under `src/`, tests under `tests/`; run outputs have not moved.

## Implemented live path

The operator-run tabletop scan shares one RGB-D capture and robot connection.
Joint feedback and nominal CAD camera placement supply the mapping pose. The
native recent/history buffers use depth-image adjacency and bounded storage;
their output is a diagnostic observed-surface graph, not current free space.

The recorded 7 October run admitted 5,758 mapping frames from 5,759 RGB-D pairs.
Reported admission rate was 29.98 Hz; worker p95 was 37.15 ms. These quantities
do not establish a 30 Hz end-to-end deadline. See the log for the timing scope,
retention failure and exact reproduction protocol.

## Offline archive work

Persistent storage, delta coalescing and bounded regional retrieval were studied
on the recorded stream. They are not silently substituted into live mapping.
Matching the persistent RAM graph checks storage fidelity; it does not establish
surface accuracy, dynamic-object robustness or independent pose calibration.

## Local demo inventory

The following paths are relative to the **parent ICRA2027 workspace**, not this
repository. Videos remain in the local evidence archive and are not embedded as
remote public links in the README.

| Demonstration | File |
| --- | --- |
| Recorded tabletop scan, RGB/depth and map | `output/table_scan_review_20261007_v1/video_final/demo.mp4` |
| Delta archive replay and final saved graph | `output/delta_archive_20261009_v1/video/demo.mp4` |
| Bounded regional read-back | `output/regional_read_20261009_v2/video/demo.mp4` |

The images on the landing page are selected from these existing outputs or from
an explicitly offline viewer. No new experiment or motion was run to make them.
