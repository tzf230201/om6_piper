# Piper application migration — 9 October 2026

**Repository layout update:** mapping/trajectory sources are now in `src/`, tests
in `tests/`, and migration audits in `.local/`. Audit folder names in the historical
notes below refer to the original locations; see the [current structure](REPOSITORY.md)
and main guide for active commands.

## Application/GUI stage results

This limited stage is complete, as authorized by the user: move the application
first and leave shared research archives at their original locations. There were
no CAN/USB/camera readings, enable/reset/error-clear commands or physical motion.

| Component | Location relative to `om6_piper` |
| --- | --- |
| 24 application modules, state reader, maintenance, gateway | `piper_readonly/` |
| GUI, J1–J6 Monitor, trajectory previews and bundle | `piper_readonly/web/` |
| Active D405 meshes and mounting configuration | `piper_readonly/assets/wrist_cad/` |
| Existing trajectory, capture and map-live | `om6_piper/` root |
| Mechanics, CAD, operations and state guides | `docs/` and `cad/` |
| Single interlock and historical permits/claims | Original `../piper_readonly/commissioning_interlock.json` and evidence paths |
| SDK, venv, official URDF/meshes, shared mapping | Original `../piper_readonly/` |
| Shared C++/F-GNG, datasets, paper, research output and videos | Original workspace |

Full module list: `../migration_app_20261009/plan.json`.
`app_paths.py` defines the application/dependency boundary. `robot_model.ROOT` still
points to shared data for mapping compatibility; web and wrist assets explicitly
use the new `APP_ROOT`.

### Compatibility and backups

- Legacy `.py` files are small adapters to the new implementations, not a second
  controller. Module namespaces are preserved for imports, CLI and mocking.
- `../piper_readonly/web` and `../piper_readonly/assets` are junctions to the new
  folders. Original assets were backed up, not deleted.
- `piper_readonly/node_modules` is a junction to existing Node dependencies.
  No venv, SDK, vendor repository or dataset was moved.
- The interlock and `fgng_config.json` match their pre-migration SHA-256 hashes.
  No second gate, new permit, claim consumption or permission reset was introduced.
  Path adjustments changed some executor hashes; old hash-bound permissions were
  not carried forward.
- `piper.cmd` now uses `.venv`, which has `usb`/`gs_usb`, rather than `.venv_mapping`,
  which lacks those two modules. Nothing was newly installed.

Backups are in `../migration_app_20261009/original_sources/` and
`original_directories/`. These are archives, **not runnable entry points**.
Recovery requires reviewing manifests/hashes/junctions, not rerunning migration
or recursively deleting junctions. `../piper_readonly` is still required; the
project is not portable as a single folder.

### Verification of this stage

Before cutover: **192 Python + 26 JavaScript tests passed**. Final post-cutover
verification expanded to **216 Python + 26 JavaScript tests**, all passing,
including 24 integration tests for the recorder, tabletop scanning, joint printing
and C++ mapping. Coverage included freshness, state, reset/clear/disable policies,
one-shot permissions, read-only HTTP, gateway, launcher, kinematics, CAD and
trajectory previews. Models for two firmware versions and an unknown version,
optical transforms and all meshes were identical before/after. The GUI bundle was
unchanged. An esbuild build from the new sources succeeded into a separate audit
file without overwriting the active bundle. `doctor` and CLI help worked through
both new and legacy paths.

USB discovery and the RealSense pipeline were blocked during the offline suite.
HTTP tests used temporary loopback snapshot servers. Eight socket tests were
initially denied by the sandbox (WinError 10013), then passed with loopback
permission. The limited migration and junctions were separately authorized.
These tests do not establish hardware readiness.

Evidence: `offline_staging_verified.json/.log`, `offline_post_verified.json/.log`,
`*_javascript.log`, `model_before.json`, `source_hashes.json`,
`protected_hashes.json`, `application_path_changes.diff`, and `completed.json`
in `../migration_app_20261009/`. No live viewer restart or mapping algorithm change
was made. Any subsequent shared-dependency stage needs its own scope.

## Earlier stage archive — not current status

The notes below explain the unsuccessful broader attempt. That broad plan remains
disabled. The application migration above used the new `migrate_piper_app.ps1`
script rather than rerunning the rejected plan.

Request: consolidate Piper programs/GUI/mechanics/CAD/operations/state in `om6_piper`.

## Work completed at that earlier stage

- Component/dependency inventory; pre-change source backups in
  `../migration_20261009/original_sources`, hashes in `protected_hashes.json`.
- `piper.cmd`, `piper.py` and `run_viewer.cmd` launchers; offline doctor by default.
- Start, wake-up/maintenance, available state groups and CAD guides.
- Copies of 15 CAD/reference files with SHA-256 hashes; originals unchanged.

## Components not yet moved at that earlier stage

`piper_readonly`, C++/F-GNG/research, environments, datasets and workspace output
remained at their original locations. The first attempt failed to move
`piper_readonly` because Windows reported it was in use. No destination directory
or junction was created. Automatic approval review rejected the request for broad
migration outside the sandbox because its scope could disrupt shared archives and
dependencies. No alternative route was attempted.

`consolidate_piper.ps1` and `plan.json` describe a plan that was **not applied**.
Execution of that broad script is now explicitly disabled; it is not required to
use the launcher. No data rollback is needed. Do not resume the broad plan without
scope review and renewed permission. The old blueprint does not imply that every
listed directory should belong to Piper.

## Steps that still required approval at that earlier stage

1. Migrate only the Piper application/GUI first, after identifying the process
   locking the directory and having the operator close it normally.
2. Keep shared paper/dataset archives outside that first stage. Inventory executable,
   environment and control-file references before changing paths.
3. Stage with hashes and offline tests, then cut over one file group at a time;
   do not duplicate interlock/permit files into two active control authorities.
4. After application checks pass, decide which mapping/recording folders should
   move and which remain shared dependencies. Only then clean up with approval,
   without recursive deletion through junctions.

The transitional launcher resolves local resources when available, then legacy
locations. At that stage the GUI and maintenance still used original programs and
state/interlock. The launcher bypassed no gates, sent no automatic commands and
made no claim that the robot was powered or healthy. No motors, camera, reset or
error-clear operations were accessed while preparing these guides and checks.

## Final checks at that earlier stage

Six launcher and nine offline GUI tests passed; `doctor` found all listed
dependencies. Three HTTP tests were initially blocked by sandbox socket restrictions
(WinError 10013), then passed with permission limited to loopback verification.
That test permission did not authorize broad migration. Control sources, interlock,
GUI assets and native DLL hashes still matched the baseline; the only legacy file
edited was the `om6_piper/README.md` introduction. All 15 CAD/reference copies matched
the source hashes. Full results: `../migration_20261009/verification.json`.
