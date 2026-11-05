# Piper: start here

**Application/GUI migration completed.** The launcher, 24 Piper application modules,
GUI, wrist assets, guides and CAD copies are in `ICRA2027/om6_piper`. Legacy
application paths still work through compatibility adapters. The SDK, environments,
state/interlock, official robot model and research archives remain shared. Copying
this folder alone to another PC does not provide a portable installation.

```text
om6_piper/
  piper.cmd / piper.py       common entry point for existing programs
  run_viewer.cmd            3D mesh GUI and Monitor menu
  src/                      trajectories, capture, mapping, replay and experiments
  tests/                    offline tests, separate from hardware examples
  examples/manual/          historical hardware scripts; not unit tests
  trajectory_test.py        compatibility adapter for src/trajectory_test.py
  piper_readonly/            application source, state reader, maintenance, gateway
    web/                    3D viewer, Monitor table, trajectory preview
    assets/wrist_cad/        active D405 meshes and mounting configuration
    app_paths.py            boundary between local application and shared resources
  docs/                     operations, state, mechanics and migration
  cad/source/               original STEP
  cad/inventor/             IAM and four referenced IPT copies, plus arm STL
  cad/reference/            D405 references and vendor optical model
  output/                   existing robot recordings
  cad/copy_manifest.json    original locations and CAD copy hashes
  .local/                   backups, migration audits and test reports (Git ignored)
```

## Check the installation without accessing the robot

```powershell
cd "C:\Users\teuku\OneDrive\Documents\ChatGPT\ICRA2027\om6_piper"
.\piper.cmd doctor
.\piper.cmd --help
```

`doctor` checks files and modules and prints their actual locations. It does not
power motors, clear errors, connect to CAN or open the camera.

## Viewer

Run `run_viewer.cmd` or `piper.cmd gui --open`. The launcher forwards to
`piper_readonly/web_monitor.py` inside `om6_piper`, without duplicating control state.
This opens the robot reader and camera according to the existing viewer behavior.
For monitoring without a camera, use `piper.cmd gui --no-fgng --open`. The local URL
is `http://127.0.0.1:8765/`; the **Monitor** menu contains the J1--J6 table.
Do not run a second CAN/camera reader alongside a trajectory session.
The existing Tailscale configuration was preserved during migration.

For offline inspection without opening CAN or the camera:

```powershell
.\piper.cmd gui --snapshot ..\output\motion_preflight_20261005_v1\robot_status.json --port 8768 --open
```

This view is labeled as a snapshot, **not the current robot state**. Old snapshots
may contain faults that no longer apply; they do not establish readiness.

## Further guides

- [Wake-up, error clearing, disabling and motion](ROBOT_OPERATIONS.md)
- [Meaning of all available state groups](ROBOT_STATE.md)
- [Mechanics, CAD and camera transforms](MECHANICAL_CAD.md)
- [Moved components and path compatibility](MIGRATION.md)
- [Experiment and mapping notes](research/README.md)
- [Repository structure and dependencies](REPOSITORY.md)
- [Earlier monitor documentation](../../piper_readonly/README.md)

External resources remain in `../piper_readonly` (SDK, environments, official
URDF/meshes, state, shared mapping, historical CAD tools and logs), `../cpp_surface`,
`../fgng_realsense`, `../fgng_upstream`, `../research_20260930`, `../output`, and
`../tmp/video_tools`. The legacy `../piper_readonly/web` and `assets` paths are
junctions to the new application assets, not a second active copy. Do not delete
the legacy directory: it still holds dependencies. Relative launcher arguments
are resolved from `om6_piper`, not the terminal directory. The interlock and
one-shot permits remain at their original locations and were not reset.

`piper.cmd` uses `../piper_readonly/.venv` because it contains the `usb` and `gs_usb`
modules required by the robot application. `.venv_mapping` remains available for
older research workflows but lacks those modules. No packages were reinstalled.

For GUI development: `cd piper_readonly`, then `npm test` or `npm run build`.
JavaScript dependencies remain shared through `../../piper_readonly/node_modules`.
Run the full migration test suite from `om6_piper`:

```powershell
.\piper.cmd test
```

The suite does not open hardware.
