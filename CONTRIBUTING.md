# Development guide

This is a hardware-connected research workspace. Keep documentation, software
tests and hardware experiments separate.

## Before editing

1. Use `piper.cmd doctor` to inspect local dependencies without opening devices.
2. Edit application code in `piper_readonly/` and mapping/research code in `src/`.
   The two Python adapters at the repository root exist only for older commands.
3. Keep new offline tests in `tests/`. Never import the hardware scripts in
   `examples/manual/` during collection; several open devices on import.

## Verify

```powershell
.\piper.cmd test
```

This explicit suite blocks USB discovery and RealSense acquisition. HTTP tests
use temporary loopback servers and saved/synthetic data. Native mapping tests
require the existing workspace DLLs. Logs go to `.local/test_runs/`.

For a specific repository test, from `om6_piper`:

```powershell
..\piper_readonly\.venv\Scripts\python.exe -m unittest discover -s tests -p test_table_live_mapping.py -v
```

Do not run unrestricted test discovery over the whole workspace. Test success
does not authorize a hardware experiment or certify collision safety.

## Research changes

- Use a new output directory for every run. Preserve original recordings,
  timestamps, poses, frozen sources, native binaries and previous videos.
- Separate measured live latency from offline replay, processing throughput and
  video playback speed. Label snapshots and simulated trajectories explicitly.
- Keep nominal CAD extrinsics distinct from measured hand-eye calibration.
- Source-bound historical verifiers may reject edited source. Use the run's
  frozen evidence for provenance; never rewrite old hashes to make a check pass.
- Document behavior and limitations before promoting an archive prototype into
  the live capture path. The map is not a collision-avoidance controller.

## Repository hygiene

Commit code, small documentation images and intentionally reviewed CAD files.
Do not commit recordings, caches, venvs, `node_modules`, local audit directories,
private logs or control permits. Existing third-party rights still apply; no
project-wide redistribution license has been selected by the authors.
