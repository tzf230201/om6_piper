# om6_piper

## Piper entry points

See the [main guide](docs/START_HERE.md) for the GUI, wake-up, error clearing,
state explanations and CAD. Run `piper.cmd doctor` for offline checks or
`run_viewer.cmd` to open the existing viewer. The application/GUI (24 modules),
wrist assets and gateway are now in this folder's `piper_readonly/` directory.
The SDK, environments, state and research archives remain shared; the
[migration status](docs/MIGRATION.md) explains compatibility and limitations.
The experiment notes below retain their historical paths.

## Physical tabletop scan results — 7 October 2026, 18:57

The operator ran `table_scan_live_01_20261007_185721_940567`.
Recording lasted 192.05 seconds: 5,759 RGB-D pairs recorded, 5,758 frames mapped,
and one frame replaced by the latest-only queue, with no recorder/mapper errors.
Admission rate was 29.98 frames/second; **not** a 30 Hz realtime deadline guarantee.
Worker p95 was 37.15 ms, maximum 64.03 ms; 606 calls exceeded 33.33 ms.
Host receipt to worker completion p95 was 40.18 ms, excluding sensor/USB latency,
asynchronous encoding/rendering, networking and display.

RGB shows the tabletop during the sweep; the feedback + CAD optical orientation
points down by approximately 55–60 degrees. These data do not measure hand-eye
accuracy. Offline auditing/review did not access or move the robot.

Review files are in `output/table_scan_review_20261007_v1/` at the workspace root:

- `rgb_contact_sheet.jpg`: RGB overview throughout motion, including the return.
- `reconstruction/live_audit.json`: original live timing and joint checks.
- `reconstruction/reconstruction.json`: replay of exactly 5,758 admitted frames;
  all four final arrays (recent/history nodes/edges) are **identical** to live output.
- `retention_diagnostic.json`: geometry comparison before/after returning.
- `video_final/demo.mp4`: 94-second video at 3x playback; RGB + depth + graph,
  three viewpoints, memory limitations and timing. This reconstructs original
  admissions; it is not a viewer screen recording or replay benchmark.
- `video_final/verification.json`: video, graph and original input hash validation.

Confirmed limitation: history reached 40k nodes in 346/364 snapshots. In the ROI
near the nominal table plane (X 0.25–0.85 m, Y -0.45–0.45 m, Z ±0.05 m), history
nodes fell from 12,573 before return to 2,832 afterward. About 30.8% of earlier ROI
positions had a final neighbor within 20 mm. This is a geometric proximity
diagnostic, **not** tabletop/ground-truth area coverage or a change-detection score.
The current history buffer is not a global map archive. The next priority is
storage for inactive regions that are not evicted merely because the camera
changes view, evaluated with the same retention and latency tests.

Reproduce the review using a new output folder:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\review_live_scan.py .\om6_piper\output\table_scan_live_01_20261007_185721_940567 --output .\output\review_baru\reconstruction
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\render_live_scan_review.py .\output\review_baru\reconstruction --output .\output\review_baru\video_final
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\verify_live_scan_review.py .\output\review_baru --video-dir video_final
```

## Tabletop scanning + online mapping (7 October 2026)

The earlier sweep trajectory was almost horizontal. The new candidate tilts the
nominal CAD optical axis downward by 55–60 degrees, then sweeps two rows. Quintic
command speed is limited to 4 degrees/second; planned duration is 186 seconds
(the program runs slightly longer due to startup and the final hold).

**Not physically executed by the agent.** The preview starts from the
`piper_scan_vision_slow_v1` recording, not current robot feedback. The table is
assumed to be Z=0 in the base frame; change `--table-z` for a different height.
Do not add the 20 mm visual offset to physical table height. FK uses nominal CAD
and the old fallback URDF; firmware agreement and hand-eye calibration are unmeasured.

From PowerShell in `C:\Users\teuku\OneDrive\Documents\ChatGPT\ICRA2027`:

```powershell
# Offline: no USB/camera access or robot commands.
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\table_scan.py --table-z 0 --output .\output\table_scan_baru
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\preview_table_scan.py .\output\table_scan_baru\plan.json
```

Preview available: `output/table_scan_20261007_v1/preview.mp4` (6x simulation),
`plan.json`, and `preview.jpg`. Gray lines are projected mesh hulls, not camera
observations. Checking the mesh against the table plane is **not** a check of
self-collision, cables, payload, gripper fingers or objects on the table. The
gripper is unchanged. The candidate rejects starting poses differing from the
preview by >2 degrees and targets outside joint limits; it does not clamp them.

### Operator physical test after reviewing the preview and table height

Only one program may own CAN and D405. Close other device owners; do not run a
second CAN reader. Ensure the sweep, gripper and cables are clear and the physical
emergency stop is available. This command **can move the robot after ENTER confirmation**:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\trajectory_test.py --table-plan .\output\table_scan_20261007_v1\plan.json --map-live --vision-output .\om6_piper\output\table_scan_live_01
```

Open `http://127.0.0.1:8770/` while the program runs. For Tailscale, add
`--map-host <active-PC-Tailscale-IP>`, then open that address on port 8770 from an
authorized peer. This server has no motion-control buttons. Do not expose the port
to the public Internet. The bind address is checked before the robot connects;
an unavailable IP or occupied port stops startup without a silent fallback.
On 7 October 2026, the old IP `100.93.232.22` was not assigned to this PC;
use localhost until Tailscale connectivity is restored.

If `--vision-output` already exists, the recorder creates a sibling folder with a
date/time suffix, for example `table_scan_live_01_20261007_190000_123456`. The actual
name is printed as `New vision capture directory` and `Vision recording`. Old
recordings are not deleted, overwritten or resumed. Other permission/path errors
still stop startup. Offline replay/preview continues to require a new output folder.

`--map-live` automatically enables `--record-vision`. Camera and joint feedback
share the connection already owned by `trajectory_test.py`. Startup waits for one
successfully mapped frame before requesting enable. If the starting pose changes
by >2 degrees during preparation, enable is not sent. Ctrl+C stops the command
loop; **it does not confirm disabled motors**. A physical stopping mechanism is
still required.

### What is mapped

- C++ extracts depth adjacency and stores 1 cm cells, stride 8, depth 0.07–1.5 m.
  Each of two buffers holds at most 40k nodes / 80k edges.
- Recent updates on admitted frames and has a 0.5-second lifetime. History samples
  at most 5 Hz with TTL 180 seconds. It can retain changed objects and is **not**
  a current obstacle map or the paper's complete ledger.
- Input and display use separate latest-only queues. Old frames are replaced,
  not accumulated. Drop counters and processing times are recorded.
- Poses use CAD + feedback after RGB-D receipt. All three joint-cache pairs must
  show timestamp updates; a watchdog rejects stopped caches. This does not yet
  establish CAN-wire data age or exposure synchronization.
- Frames with depth times incompatible with the host clock, excessive age,
  regressing numbers or unsuitable poses are not integrated. RGB is context, not
  texture registration. The viewer dims and labels STALE for late/disconnected data.
- Mapping uses the physical base frame without the 20 mm display lift. Do not use
  this diagnostic map for collision avoidance or robot control.

Recordings: `rgbd.bag`, `rgb.mp4`, `frames.jsonl`, `metadata.json`, `motion_events.json`.
Mapping: `live_map/updates.jsonl`, `result.json`, `final.npz`, `last_view.jpg`.
The viewer stops when capture closes; offline results remain saved.

### Replay without a robot

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\replay_live_mapping.py --seconds 35 --output .\output\table_map_replay_baru
```

Uses the old scan BAG at original depth timestamp intervals, RGB from the matching
log index and the same mapper pipeline. `demo.mp4` records the replay display at
5 fps. **Not a new tabletop recording** or a live sensor latency measurement; the
old pose logs lack the newer cache watchdog. Mapping speed, rendering, queueing,
camera acquisition and video FPS are different quantities.

Tests without hardware:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe -m unittest discover -s om6_piper -p "test_table_live_mapping.py" -v
```

## Storage ablation: actual tabletop recording (8 October 2026)

Results: `output/archive_retention_20261008_v1/comparison.json`.
Demo: `output/archive_retention_20261008_v1/video/demo.mp4`.
This is **offline research**, not a replacement for the live mapper. No robot
commands, camera acquisition, trajectory changes, or manuscript changes were made.

The experiment uses exactly the 854 history admissions (maximum 5 Hz) from
`table_scan_live_01_20261007_185721_940567`, selected from its 5,758 admitted live
mapping frames. Every variant receives identical depth frames, timestamps,
recorded joint/CAD poses, 1 cm cells and stride 8. The prepared depth file and
original source files are fingerprinted. Frame base coordinates have no 20 mm
presentation offset. This recording has no independently measured hand-eye or
ground-truth map, and does not test object relocation.

### What the comparison established

The bounded baseline reproduces the actual live final history arrays exactly.
The disk archive and large persistent RAM graph have identical final oriented
keys, positions and edge keys: 283,706 nodes / 1,749,244 edges. Increasing only the
SQLite cache from 4 to 32 MiB does not change that result. These are observed
depth-adjacency links, not newly inferred nearest-neighbour links.

We fixed 12,573 reference samples in a tabletop-height ROI at scan time 131.27 s,
then queried their nearest final mapped point after return at 192.05 s. The ROI
is X [0.25,0.85], Y [-0.45,0.45], Z [-0.05,0.05] metres, not semantic segmentation.

| Storage | Reference samples within 5 mm | Within 20 mm | History update p95 | Peak incremental process RSS |
| --- | ---: | ---: | ---: | ---: |
| 40k nodes / 80k edges, TTL 180 s | 5.70% | 30.78% | 6.71 ms | 36.34 MiB |
| 500k / 3M, TTL 180 s | 95.94% | 100% | 6.36 ms | 795.99 MiB |
| 500k / 3M, no expiry during study | 95.98% | 100% | 6.08 ms | 804.98 MiB |
| Disk, 32 MiB page-cache target | 95.98% | 100% | 862.63 ms | 44.89 MiB |

These are exploratory **single-pass offline** timings for extraction + history
update, excluding decoding, 30 Hz recent mapping, full graph export, rendering,
camera/USB and robot IO. Large RAM buffers are preallocated and deliberately
oversized to avoid eviction; this is not a byte-budget-matched comparison.
Process RSS increments exclude OS file cache and final full-map export memory.
The 4 MiB disk pilot overlapped build/diagnostic activity; its exact latency is
not an isolated benchmark. Raw logs retain it, but the table uses the later
32 MiB run. In that run, 820/854 calls exceeded 200 ms; it took 378.4 s to process
191.85 s of history inputs. Thus even the separate 5 Hz archive path cannot keep
up with this implementation. A background thread alone would only hide backlog.

Both disk variants occupy 384,966,656 bytes after close (367.13 MiB); database +
WAL + shared-memory files peaked at 419,471,112 bytes. The 4/32 MiB settings are
SQLite page-cache targets, **not** total-memory guarantees. Full export is an
explicit offline operation and can load the entire graph into memory.

Retaining a nearby point is not evidence of accuracy, free space, semantic area
coverage, or dynamic robustness. In particular, an archive can preserve obsolete
surfaces. The experiment identifies **capacity eviction**, not the 180 s TTL,
as the dominant cause of the reference tabletop loss in this one recording.

### Code and verification

- `cpp_surface/include/observed_archive.hpp`, `src/observed_archive.cpp`: persistent
  observed-surface storage, 16-cell region index, stable endpoint keys, per-frame
  atomic transactions, bounded SQLite page-cache target and database-page quota.
  Only observed keys are updated; unseen regions are neither erased nor marked free.
- `src/archive_study_api.cpp`: separate offline C API. It never replaces
  `camera_local_api.dll` or calls robot/camera hardware.
- `om6_piper/archive_retention_study.py`: matched-input pack, isolated-process
  storage runs, memory/timing logs, canonical geometry/topology parity and retention.
- `om6_piper/render_archive_study.py`: causal old-history replay with RGB/depth,
  followed by comparisons at the final time only, and three views of the archive.
- Native archive tests passed: negative region coordinates, 100 seeded updates
  against a non-evicting RAM graph, reopen, unknown gaps, invalid input, disk-full
  rollback and retry. Five Python helper/trace tests passed. Windows Application
  Control blocked the separate active-graph regression executable and the rebuilt
  v2 test executable; security settings were not changed. Full-recording disk/RAM
  parity and original-live baseline parity provide additional integration checks.

Native binaries are frozen separately in `build_archive_20261008_v1` and `v2`.
The original live build `build_robot_replay_20261007` and all recordings remain
unchanged. Sources used for analysis/video are copied into the experiment's
`source_evidence`; run-specific source copies and DLL hashes are recorded where
available. The initial 4 MiB pilot hashes the runner at completion, not a frozen
startup copy; do not treat that hash as a full execution-source attestation.

Example offline rerun from the workspace, using a **new** output directory:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\archive_retention_study.py run .\output\archive_retention_20261008_v1\pack disk .\output\archive_disk_repeat --dll .\cpp_surface\build_archive_20261008_v2\Release\archive_study_api.dll --cache-kib 32768
```

### Next research gate (at the end of the 8 October experiment)

Keep the recent 30 Hz surface buffer independent. Coalesce repeated updates to
dirty spatial blocks in a bounded working set before writing archive chunks;
preserve last-observation timestamps and edge witnesses across coalescing.
Measure storage lag, backlog/overflow, disk failures, read-back latency and total
memory under a paced replay before connecting the candidate to live capture.
Then add visibility-based change evidence: unseen/occluded is not contradicted.
Repeat retention, geometry and latency tests across recordings and the existing
dataset families. Neither this storage prototype nor its video is evidence of a
new realtime, dynamic-robust, or conference-ready mapping method.

## Bounded background block archive — 9 October 2026

The offline candidate now implements the storage gate above. It does **not**
replace `live_mapping.py`, connect to a device, or change the robot trajectory.
Evidence is in `output/block_archive_20261009_v1/`; older experiments and native
builds are preserved. The input is the same fingerprinted 854-frame history
pack, not a new recording or a new dataset.

### What changed

- A single C++ producer coalesces repeated oriented-cell and edge updates before
  enqueueing them. The newest value, timestamp and source-frame/pixel witnesses
  survive; observed depth-adjacency links are not replaced by inferred links.
- A separate C++ writer updates only touched spatial blocks in SQLite. Stable
  endpoint keys are stored, never indices into evictable active slots. Native
  extraction is unchanged (1 cm cells, stride 8); 8/16 cm archive blocks are an
  independent storage partition, not a lower-resolution map.
- Blocks use lossless Windows XPRESS_HUFF compression, exact floating-point
  values, bounded decoded-size checks, and an FNV-1a corruption checksum. The
  checksum is not authentication. This candidate is Windows-specific.
- One pending batch is limited to 40k unique nodes / 80k unique edges; at most
  two further batches wait for the writer. There is also one incoming frame,
  one in-flight batch, a decoded region, serialization/compression buffers,
  hash-table overhead and SQLite memory. These capacities are **not a total-RAM
  byte guarantee**. The 8 MiB page-cache setting is a target, not an RSS cap.
- Paced submissions do not wait for queue capacity. When full, the complete
  incoming history frame is rejected and counted, with no partial admission.
  Burst mode instead waits explicitly. The recent/live map is separate and was
  not part of this timing experiment.
- A batch's graph changes and complete-frame watermark commit atomically with
  WAL / synchronous FULL. Accepted is not durable. On write failure the writer
  latches an error, rejects further input, and does not acknowledge that batch.
  Explicit `flush()` drains accepted data. No idle timer flushes pending input;
  shutdown integration must call `flush()`. Process failure can lose unsaved
  accepted updates. The SQLite page quota does not bound WAL or all disk use.

### Experiments and unsuccessful candidates

`burst_r1` (raw 16 cm blocks) and `compressed_burst_r1` (lossless 16 cm blocks)
both recover the full reference graph, but burst submission waits on backlog;
its call latency is not a nonblocking live result. Compression reduces the
closed archive from 241,201,152 to 62,111,744 bytes in those pilots.

Pacing the compressed 16 cm / 1 s candidate at original source intervals
(`paced_1s_probe`) rejected **88/854** frames. Using 8 cm blocks with the same
1 s window (`region8_probe`) still rejected **59/854**. Both failures are kept.
Their parity checks refer to a RAM replay of **accepted frames only**, not all
854 frames. Do not mistake `final_reference_parity=true` in those result files
for complete input coverage.

The 8 cm / 1 s profile serialized 9.017 GB of raw block data and passed 2.193 GB
of compressed payload to SQLite to produce a 56.7 MB closed file. These are
application payload counts, **not physical disk-write traffic**. Decode and
encode together consumed about 70% of writer time. Smaller regions alone did
not eliminate the repeated-block rewrite cost.

The selected candidate uses **8 cm blocks and a 2 s coalescing window**, with
the same capacities and input cadence. This trades persistence delay for fewer
repeated writes. Parameters were selected on this same recording: it is
development evidence, not held-out generalization. The first paced run
(`paced_r1`) saved 854/854 frames with no rejection; front-end call p95 was
11.98 ms, oldest-update-to-commit p95 4.60 s, and final drain 5.00 s. Its closed
file was 60,723,200 bytes. Graph geometry, edge keys and node times match the
full RAM reference (283,706 cells, 1,749,244 links).

The sequential repeat (`paced_r2`) also saved 854/854 with zero rejection:

| Selected candidate | Paced run 1 | Paced run 2 |
| --- | ---: | ---: |
| Front-end call p95 | 11.98 ms | 13.81 ms |
| Oldest accepted update to commit p95 | 4.60 s | 5.34 s |
| Worst oldest-update-to-commit delay | 5.71 s | 7.40 s |
| Final drain after last submission | 5.00 s | 6.70 s |
| Sampled incremental RSS | 59.47 MiB | 62.11 MiB |
| Closed archive | 57.91 MiB | 57.91 MiB |
| Accepted / committed frames | 854 / 854 | 854 / 854 |

Both runs used the same v3 DLL and depth pack on an Intel Core i7-14650HX / Windows.
No CPU affinity, real-time OS scheduling or thermal/background-process isolation
was imposed. There were respectively 2 and 1 front-end calls above 33.33 ms.
No input was rejected in these two finite runs, but the late writer batches can
still take longer than a 2 s accumulation interval: sustained long-run headroom
is **not established**. Front-end submission latency must not be compared with
the previous synchronous commit latency as though it were an equivalent speed-up.

`pressure` deliberately removes pacing, feeds 80 inputs, reduces the accumulation
window to 0.2 s and inserts a 1 s delay in each write. Four complete frames are
accepted/committed and 76 rejected. Reopening matches an accepted-only RAM
reference; rejected frames contribute no partial graph. This is an overload
contract test, not a plausible camera throughput result. Its `group_ms` includes
the artificial test delay. Its maximum file-size sample during admission misses
the subsequent drain growth; do not treat that sample as an overall disk peak.

The replay measures extraction plus archive admission, a background writer,
and wall-clock storage delay. It excludes 30 Hz recent mapping, live camera/USB,
FK, rendering and final full-archive export. RSS is sampled, excludes OS file
cache, and is not an instantaneous allocator peak. Maximum DB+WAL size is also
sampled during admission. Full export is deliberately offline and can require
memory proportional to the whole archive. Neither fast submission nor complete
retention makes obsolete observations safe for collision checking.

### Reproduce without hardware

The current adapter loads `build_block_archive_20261009_v3/Release/`.
The raw v1 and compressed 16 cm v2 DLLs and each run's startup source copy remain
available. Do not replace their paths with a different binary when reproducing
older results. Run from the workspace and choose a **new output directory**:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\block_archive_study.py .\output\block_archive_new_repeat --mode paced --interval 2 --region-cells 8
.\piper_readonly\.venv_mapping\Scripts\python.exe -m unittest discover -s om6_piper -p "test_block_archive.py" -v
```

Five Python/native-binding tests passed, including partition/coalescing parity
for all geometry and witnesses, close/reopen, timestamp rejection, checksum and
size corruption rejection, and disk-quota rollback/error latching. The raw v1
and compressed v2 native contract executables passed. Windows Application
Control blocked the v3 native test executable; its standalone native test suite
is not reported as passed and no security settings were changed.

Full-scene checks in `summary.json` also confirm exact normals, radii, node and
edge observation times, and frame/pixel witnesses across both selected runs and
the complete compressed 16 cm reference. `verification_addendum.json` records
test scope and measurement caveats; frozen analysis/video sources are in
`source_evidence/`. Run-specific C++ source and DLL hashes are in each protocol.

The new demo is `output/block_archive_20261009_v1/video/demo.mp4`. It presents
recorded RGB and metric-coloured depth beside causally sampled admission/commit
counters at 4x playback. A separate stop/drain card precedes the final graph in
oblique, top and side views. It never overlays the full final archive on early
camera frames. Export and drawing are excluded from mapper timing. The video
uses two-run results and retains the unsuccessful 1 s cases in its explanation.

### Remaining gate before connecting this archive to live mapping

1. Reduce repeated whole-block reconstruction: test per-region delta records
   with bounded compaction work and explicit recovery rules. Measure payload
   amplification, commit lag and memory; retain the exact graph/witness contract.
2. Add bounded regional read-back and test latency on revisits. Whole-archive
   `snapshot()` must remain an offline operation, not the renderer's live API.
3. Test a longer recording and held-out recordings/dataset families. The late
   batches here leave insufficient evidence of sustainable storage throughput.
4. Only then run the archive beside the existing recent mapper and remeasure
   camera-to-worker timing, queue drops, display latency and shutdown draining.
   Do not use last-observed archive geometry as a current collision/free-space map.

No manuscript claims or new citations were added from this single-recording
storage development study. The paper still needs independent pose/geometry
validation and changed-object evaluation; compression does not address those.

## Delta archive with bounded compaction — 9 October 2026 follow-up

The next candidate is in `output/delta_archive_20261009_v1/`. The previous block
archive, original recordings, live DLL, paper and videos are preserved. This is
an **offline archive study**, not deployment on the robot or a new mapping model.

### Storage contract

Instead of loading and rewriting a complete region on every batch, the writer
stores a compressed delta containing the new node/edge values, original seen
times, and source-frame/pixel witnesses. A region is reconstructed by loading
its last checkpoint and folding its deltas in monotonically increasing batch
order. The last observation of each oriented key wins, exactly as before.

Pending limits remain 40k nodes / 80k edges, two waiting batches, an 8 MiB SQLite
cache target, 8 cm regions and 1 cm cells. Compaction becomes eligible after
eight deltas and is prioritized by tail length. A transaction processes at most
64 regions and 32 MiB of checkpoint-plus-tail raw input. Selection reads at most
128 candidate metadata rows from an index; no global dirty-region list is kept
in process memory. Startup aggregates segment counts once; updates use local
metadata and the indexed maximum tail, not a full-map scan.

A region may retain at most 32 delta records. If bounded compaction cannot keep
the next batch within that limit, the **whole batch rolls back**, the writer
latches an error and further admissions fail. This does not silently claim
accepted-but-unsaved input as durable. It is a resource-failure rule, not a
guarantee of sustained throughput on arbitrary scenes. Region reconstruction
also checks raw-byte/count bounds and checksum/metadata agreement.

Checkpoint creation, replacement of its deltas, graph updates and the complete
frame watermark share one SQLite FULL/WAL transaction. Remaining tails are
already durable, even when not compacted. `flush()` commits pending input but
does not perform an unmeasured final all-region compaction. Closed-file sizes
therefore include the remaining tails and SQLite free pages. No VACUUM is hidden
outside the timing window. Merge memory includes an accumulated region, a
decoded segment, raw/compressed blobs, hash-table and SQLite overhead; the
32 MiB input-work limit is **not** a 32 MiB RSS or hard execution-time guarantee.

### Failure checks and pilot

Six Python-driven native tests passed: checkpoint/tail parity and resume,
unknown-depth preservation, malformed delta size/checksum rejection, hard-tail
limit rollback and recovery, disk-quota rollback, and abrupt child-process exit.
The unclean-exit test recovers all 12 committed frames while excluding one
unflushed pending frame. It is a process-crash test, not a physical power-loss
experiment. Small fixtures remain under `tmp/delta_archive_test_*` for inspection.

The initial `matched_2s` pilot reduced batch-write time but exposed a large WAL:
sampled database + WAL + shared-memory files reached 471,536,872 bytes. The
region-budget SELECT still had an active cursor at COMMIT. Revision v2 finalizes
that cursor before COMMIT so automatic checkpointing can run. No synchronization
mode or filesystem security setting was weakened. This is consistent with
[SQLite's explanation of reader gaps and WAL checkpointing](https://sqlite.org/wal.html#avoiding_excessively_large_wal_files).
The pilot remains intact; it is not substituted for the corrected measurements.

Native builds are frozen separately under `build_delta_archive_20261009_v1` and
`v2`. The current `--delta` adapter uses v2. The default non-delta adapter still
loads the previous frozen v3 block DLL. Choose a new output directory for every
experiment:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\block_archive_study.py .\output\delta_new_run --delta --mode paced --interval 0.5
.\piper_readonly\.venv_mapping\Scripts\python.exe -m unittest discover -s om6_piper -p "test_delta_archive.py" -v
```

The coalescing interval is checked against source timestamps **on the next
submission**; it is not an independent wall-clock flush timer. When input stops,
the producer must call `flush()` to submit the remaining batch. A live adapter
will need an explicit shutdown/idle-flush policy, rather than assuming that a
0.5 s setting guarantees persistence within 0.5 s. Likewise, the hard database
page quota is not a quota on DB + WAL + shared-memory files combined.

The matched-window v2 replay saved all 854 inputs and exactly recovered all
283,706 cells and 1,749,244 links, including normals, radii, observation times
and frame/pixel witnesses. At the same 2 s window, application-written payload
fell from 1,454,956,182 to 334,756,485 bytes. Writer batch p95 fell from 2236.16
to 722.71 ms; oldest-accepted-update-to-commit p95 fell from 4604.26 to 2823.86 ms.
These are single-run development comparisons, not independent validation.
Submission p95 instead increased from 11.98 to 18.82 ms, and the closed file grew
from 57.91 to 72.62 MiB. No camera-to-display speed-up is inferred from this.

With the cursor fixed, the largest sampled DB + WAL + shared-memory footprint
was 87,029,520 bytes (83.00 MiB), down from the retained pilot's 471,536,872 bytes
(449.69 MiB). File sizes were sampled after admissions and after draining;
unobserved short-lived peaks are not ruled out. Full synchronous commit remains
enabled. The corrected binary SHA-256 is
`a7ada4f52231b583395eda7ecb85c5c93f6fb2661d9379e511602af1da842c0f`.

### Shorter-window repeatability

Two sequential 0.5 s paced replays (`fast_r1`, `fast_r2`) used that same binary,
depth pack, original input timestamps, poses and working-set limits. Both saved
all 854 inputs with zero rejected frames, and exact graph/metadata equality
before and after reopening. Values below are measured, not target settings:

| Measurement | Repeat 1 | Repeat 2 |
|---|---:|---:|
| Front-end call p95 | 11.54 ms | 12.35 ms |
| Writer batch p95 | 459.16 ms | 485.60 ms |
| Oldest accepted update to commit p95 | 1.133 s | 1.157 s |
| Maximum oldest-update-to-commit delay | 1.230 s | 1.272 s |
| Final drain | 0.714 s | 0.769 s |
| Sampled incremental RSS | 36.95 MiB | 37.77 MiB |
| Closed database, including remaining tails | 66.04 MiB | 66.04 MiB |
| Sampled DB + WAL + shared-memory files | 76.12 MiB | 76.12 MiB |
| Application-written payload | 715.02 MiB | 715.02 MiB |

Each run produced 285 transactions. Compaction processed 7,779 regions in total,
with at most 64 regions and 26,292,216 raw input bytes in any transaction. The
largest committed tail was 10 segments per region; 5,606 durable segments remained
at shutdown, with at most seven per region and no region at the eight-segment
compaction threshold. Remaining tails still have a read-back cost; they are not
missing input or a claim of zero future maintenance work.

The shorter window reduces disk-save delay, but increases application-written
payload relative to the 2 s delta policy (715.02 versus 319.25 MiB). The matched
2 s comparison is needed to isolate the representation change from this policy
trade-off. Native front-end calls above 33 ms are not erased from the logs;
the measured Python-to-native call exceeded 33.33 ms four and two times in the
two runs. Only the history stream (up to 5 Hz) is replayed here. Windows scheduling,
OS caches and background activity were not controlled; no hard realtime or
end-to-end camera throughput claim follows.

### Repeated-input and overload tests

`repeat3` repeats the same 854-input clip three times with synthetic, monotonic
frame IDs/timestamps. Geometry and poses are reused; this is **not three datasets
or an independent longer robot recording**. It feeds input without camera pacing
and deliberately blocks the producer when the two-batch waiting queue is full.
All 2,562 inputs were committed in 854 transactions, in 263.00 s including final
drain. Front-end p95 was 444.49 ms because it includes deliberate backpressure;
this must not be presented as live camera performance. Complete geometry and all
metadata matched the correctly time/ID-shifted reference after reopening.

The repeated run compacted 24,673 region instances. Per-transaction maxima were
64 regions and 33,554,072 raw input bytes, below the 33,554,432-byte budget. The
maximum committed tail was 10. At the end, 6,596 durable segments remained, with
at most seven per region. Sampled incremental RSS was 55.27 MiB, higher than the
single-clip runs; this finite reuse test does not prove memory bounds for
unbounded new geometry. The closed database was 69.51 MiB.

`pressure` feeds 80 inputs without pacing, uses a 0.2 s window, and deliberately
adds 1 s to each write. Four frames were accepted and committed, and 76 rejected.
The final graph keys, positions and node seen-times match an accepted-only RAM
reference. Reopening preserves all stored metadata. No rejected frame partially
enters the archive. This is a failure/backpressure contract, not an accuracy or
realtime benchmark; `group_ms` includes the artificial sleep.

Final analysis and demo sources are frozen in `source_evidence_v2/` and checked
against `comparison.json`. The initial `source_evidence/` and
`comparison_initial.json` are retained; the final video subtitle explicitly
clarifies the unpaced, blocking nature of the repeated-clip test.
The demo is `output/delta_archive_20261009_v1/video/demo.mp4`.
It uses the first 0.5 s run, shows recorded RGB and metric depth, causal
admission/commit counts, a final-drain card, then the graph in three views.
The final graph is never displayed as though it existed during an earlier
camera frame. The longer burst test is labelled as unpaced with producer waits.
Results retain the matched-window comparison, the larger file-size trade-off,
and the two-run shorter-window delay rather than implying a universal speed-up.

### What still gates live integration

1. A bounded, region-only read-back API. `snapshot()` currently reconstructs the
   entire archive offline and must not be called on a camera/rendering hot path.
2. Producer idle/shutdown flushing, explicit accepted-versus-durable states,
   disk-full/error handling, and a nonblocking admission policy for live input.
3. Different held-out recordings and dataset families, new-area growth, and
   long-duration runs. Repeating this clip is not a substitute for those tests.
4. Integration beside recent mapping, followed by camera-to-worker/display
   latency and queue-loss measurements. Neither old surfaces nor nominal CAD
   poses establish current obstacle occupancy or independently measured accuracy.

No physical robot motion, live mapper changes or manuscript edits were made in
this follow-up. The original live DLL, block baseline and previous demo remain
unchanged. These are development measurements; the paper still needs independent
calibration/geometry evidence and changed-object tests before stronger claims.

Delivery checks are in `delivery_verification.json`: six delta and five baseline
Python/native-binding tests passed. The 1280 x 720, 30 fps demo has 3,059 frames,
lasts 101.97 s and is 4,306,896 bytes. Every frame decoded successfully; replay
timestamp assertions passed. Encoded midpoints of all seven scenes were visually
checked for readable layout, including the quantitative depth colorbar and three
final graph views (`video/visual_review.json`). These checks do not certify scene
geometry. Preserved baseline/live binary and previous-video hashes also match.

## Bounded regional read-back (2026-10-09)

The next development step addresses item 1 in the preceding integration gates:
a consumer can now retrieve a small spatial region without loading the complete
archive or waiting for the writer to flush. This is an offline-tested component,
**not yet deployed in the live mapper**. The source recording, old archive,
previous DLLs and videos remain intact. No robot commands or manuscript changes
were made in this step.

### Read and boundary-link contracts

`cpp_surface/include/regional_archive_reader.hpp` exposes a native C++ reader;
`cpp_surface/src/regional_read_api.cpp` provides the separate C ABI. The frozen
v2 build is `cpp_surface/build_region_read_20261009_v2/Release/`. The Python
binding in `regional_read_study.py` opens only an existing archive, using an
independent read-only connection. It neither creates an archive, submits input,
flushes a writer nor reconstructs the global graph. SQLite WAL/SHM side files
can still be used by a read-only connection; read-only does not mean zero
filesystem side effects.

Requests are half-open integer region boxes `[lower, upper)`. Each region is
8 cm wide (eight 1 cm cells); negative coordinates use floor division. The
prototype currently accepts only the compressed delta archive signature with
this cell size, region size and 32-segment hard tail limit. Default per-query
limits are 64 regions, 8 MiB raw input, 8 MiB compressed input, 40,000 output
nodes and 80,000 decoded owned edges, including outgoing boundary edges.
Metadata and compressed lengths are checked before copying/decompressing their
payloads. Decoded geometry, ownership, checksums and observation watermarks are
validated. Missing regions produce empty **unknown** space, not free space.

The first state query fixes one committed snapshot. All region data for that
query comes from it. Statements and the read transaction are released before
final graph assembly; no read transaction is retained while rendering or between
job steps. Contention is reported immediately rather than using a long busy wait.
These choices follow [SQLite's snapshot isolation](https://sqlite.org/isolation.html)
and [WAL reader/checkpoint behavior](https://sqlite.org/wal.html).
A result describes a committed historical revision, which can be stale as soon
as a newer commit arrives. It is not current collision or free-space evidence.

Only edges with both endpoints inside the requested box enter the local graph.
Owned outgoing links are returned separately as exact endpoint keys, seen-times
and frame/pixel witnesses. They must not be drawn as interior links until both
endpoints are present. The v1 read-only pilot omitted these separate boundary
arrays; v2 adds them without inventing nearest-neighbor connections. A failed
query invalidates the previous result, so a caller cannot mistake old output for
a successful new read.

`regional_read_job.py` loads one 8 cm region per `step()`, retaining at most
40,000 nodes, 200,000 owned links and 32 MiB of array payload for at most 64
regions. These are distinct aggregate limits; per-step raw/compressed limits
remain 8 MiB each. A step either succeeds or fails; it is not preemptible halfway
through a native call. All steps must have the same durable revision tuple. A
revision change or capacity failure clears the unfinished job. There is no
infinite automatic retry and no partially loaded graph labelled complete.
An already completed graph remains a snapshot of its recorded revision, not a
promise of freshness at display time.

After all reads succeed, `assemble()` restores cross-region links by exact
integer-key joins. Native C++ performs retrieval, but this job scheduler and the
final vectorized NumPy assembly are still a Python prototype. Neither assembly
nor reads should run on the camera capture thread. Raw input and retained-array
limits are **not** hard wall-clock or total RSS bounds: decoded maps, temporary
arrays, native/Python output copies, allocator state and SQLite overhead also
consume memory.

### Recorded measurements and failures

Evidence is in `output/regional_read_20261009_v2/`. It uses the closed `fast_r1`
archive from the previous delta study: 854 inputs, 283,706 nodes and 1,749,244
links from **one development recording**, with nominal CAD/recorded-joint poses.
For each cube size, 16 costly regions and 32 seeded samples define 48 queries.
Planning uses an offline global metadata scan; that scan is not part of the
bounded read API. Direct reads repeat the same boxes twice in separate processes
with persistent connections, without purging OS caches. These are not cold-disk
measurements or held-out validation.

| Direct read | Completed, each repeat | Call p95, repeat 1 / 2 |
|---|---:|---:|
| 8 cm cube | 48 / 48 | 20.41 / 17.90 ms |
| 16 cm cube | 44 / 48 | 57.77 / 54.87 ms |
| 32 cm cube | 28 / 48 | 53.51 / 54.08 ms |

All 240 successful direct queries exactly match crops of the complete archive,
including geometry, topology, normals, radii, observation times and witnesses.
The other queries exceed the raw read budget; their failures remain in the logs.
Percentiles above describe successful queries only. On an isolated archive copy,
three full-graph exports took 4.08--5.90 s with a sampled RSS increment of
263.76 MiB. They return much more data, so this is **not an equal-work algorithm
speed-up comparison**.

Splitting the same boxes into one-region steps recovers all 16 cm requests and
41 of the 48 requests at 32 cm. The seven rejected 32 cm jobs exceed aggregate
retention capacity; no data is silently removed to claim completion. Initial
Python per-edge stitching and the optimized exact-key NumPy join are retained
in separate `jobs/` and `jobs_vectorized/` outputs:

| Job measurement | 16 cm query | 32 cm query |
|---|---:|---:|
| Completed requests | 48 / 48 | 41 / 48 |
| One successful step p95, optimized | 19.96 ms | 13.24 ms |
| Largest successful step, optimized | 33.11 ms | 39.01 ms |
| Complete-job p95, original -> optimized | 251.58 -> 115.90 ms | 919.82 -> 411.83 ms |
| Final-join p95, original -> optimized | 164.60 -> 19.16 ms | 616.19 -> 92.95 ms |
| Sampled RSS increment, optimized | 10.99 MiB | 26.31 MiB |

All 89 completed optimized jobs exactly match both the original job outputs and
their full-map crops. These are single sequential passes over 48 requests per
size, not repeated independent timing trials. Whole-job times include reads,
bookkeeping, memory sampling and assembly; they exclude NPZ serialization.
Step percentiles include only successful recorded steps (including those before
a job fails), not the failing call itself. The 32 cm sample contains many empty
regions, so its smaller step p95 does not mean a larger query is cheaper.
Memory is sampled after steps/assembly, includes allocator and prior sample-save
residue, and excludes the full-reference validator and OS cache. Short-lived
peaks can be missed. Complete-job p95 excludes rejected jobs.

The graph has not become geometrically more accurate merely because read-back
is faster. Exact equality is a storage/retrieval check, not surface ground truth.
Complete retrieval remains slower than a 33.3 ms camera frame in the tail; no
30 Hz camera-to-display claim follows.

### Reproduction and evidence

Use fresh output folders; never overwrite an earlier run:

```powershell
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\regional_read_study.py prepare .\output\regional_read_new
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\regional_read_study.py run .\output\regional_read_new
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\regional_job_study.py run .\output\regional_read_new
.\piper_readonly\.venv_mapping\Scripts\python.exe .\om6_piper\regional_job_study.py analyze .\output\regional_read_new
.\piper_readonly\.venv_mapping\Scripts\python.exe -m unittest discover -s om6_piper -p "test_regional_reader.py" -v
```

The study scripts use the saved archive and frozen v2 reader DLL explicitly;
they do not attach to hardware. `source/` preserves the initial loop-based job,
`jobs_vectorized/source/` preserves the optimized experiment, and
`final_source_evidence/` captures the delivered implementation and renderer.
`protocol.json`, summaries and `delivery_verification.json` retain hashes,
budgets, full test logs and measurement caveats. The final regression runner
uses the new binaries for seven regional and five block tests plus six delta
tests. For the abrupt-exit delta test, the child still produces its fixture with
the previous frozen delta DLL and the new DLL verifies recovery; this is explicitly
a compatibility check, not a new-writer hardware power-loss test.

The demo is `output/regional_read_20261009_v2/video/demo.mp4`: 1280 x 720, 30 fps,
2,076 frames, 69.2 s, 1,177,436 bytes. All frames decoded successfully. Encoded
midpoints of all 16 scenes were inspected for layout. It shows recorded RGB as
context, illustrated block reads, exact completed patches in three views, timings
and rejected-query counts. It intentionally uses a **fixed post-scan archive**;
it does not depict those complete patches as if they existed during an earlier
live camera frame. The four displayed patches are cases rejected by a direct
16 cm read but recovered by the incremental job, not randomly selected accuracy
examples. Video checks are in `video/verification.json` and
`video/visual_review.json`.

### Remaining integration gates

1. Sustained concurrent writer/reader replay with queue loss, commit delay, query
   starvation and WAL growth measurements. A small concurrent unit fixture is
   not enough to demonstrate sustained realtime behavior.
2. A version-aware tile cache or changed-region protocol: the current conservative
   global-revision check can abort a job even when a commit affects another region.
   Do not hide these aborts or spin indefinitely to get a complete result.
3. Put this consumer on its own worker, add idle/shutdown flush and error policies,
   then measure camera-to-worker/display latency before changing the live mapper.
   Move job orchestration/assembly into C++ if profiling justifies it.
4. Test different recordings and dataset families, growing geometry, independent
   alignment/geometry truth and changed objects. This one-clip optimization does
   not satisfy the multi-dataset research evaluation by itself.

The measured Windows SQLite runtime is 3.51.1. Upstream documents a
[WAL-reset bug and fixed releases](https://sqlite.org/wal.html#walresetbug) for
concurrent writing/checkpointing connections. This prototype has one writer
connection and separate readers that never write or checkpoint, but the runtime
is not claimed to be patched. Resolve this dependency before broader deployment
or adding checkpoint/writer connections; no runtime or OS security settings were
changed in this work.
