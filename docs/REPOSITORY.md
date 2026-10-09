# Repository layout and shared dependencies

The landing page describes the project; detailed research history belongs in
`research/DEVELOPMENT_LOG.md`. Source, offline tests and manual hardware examples
are physically separated. This reorganization does not change robot-control
policy, calibration, mapping algorithms or recorded evidence.

## Where to work

| Location | Purpose |
| --- | --- |
| `src/` | Capture, live mapping, trajectory, replay, experiments and renderers |
| `piper_readonly/` | Robot-state reader, guarded maintenance, gateway, GUI and wrist assets |
| `tests/` | Offline tests; bootstrap only configures import paths |
| `examples/manual/` | Original hardware-opening experiments, excluded from tests |
| `tools/` | Explicit test runner and repository checks |
| `docs/`, `cad/` | Documentation, figures and CAD dependencies |
| `output/` | Existing robot recordings; ignored by Git, not moved or deleted |
| `.local/` | Backups, migration records, previews and test reports; ignored by Git |

Paths in this table are relative to the repository root. Root `trajectory_test.py`
and `print_joints.py` are compatibility adapters. `piper.cmd` and `run_viewer.cmd`
retain their names. Other source files formerly at the root are now in `src/`;
use that path for direct invocation. Tests previously at the root are in `tests/`.
There is intentionally no duplicate root file for every historical research script.

The full previous README is preserved unchanged in
`research/DEVELOPMENT_LOG.md`. Its commands refer to the old layout. A source
reference resolver locates current files, but does not bypass frozen source hashes.

## Existing-workspace requirement

The application is not yet a portable fresh-clone distribution. It expects:

```text
ICRA2027/
├── om6_piper/                   this repository
├── piper_readonly/
│   ├── .venv/                   robot-compatible Python environment
│   ├── .venv_mapping/           existing research environment
│   ├── pyAgxArm/                SDK source
│   ├── piper_model_source/      official robot URDF and meshes
│   ├── commissioning_interlock.json  single authoritative gate
│   └── ...                     shared mapping, fixtures and configuration
├── cpp_surface/                frozen native mapping/archive builds
├── fgng_realsense/              native geometry backend
├── output/                     shared experiment evidence and demos
└── tmp/video_tools/             existing ffmpeg tools
```

`piper.cmd doctor` reports actual resource locations and missing dependencies.
No new install, SDK copy or duplicated interlock is hidden behind the launcher.
Do not delete the sibling directory just because application code moved.
GUI `node_modules` and old web/assets paths use the existing compatibility
junctions; `.gitignore` prevents vendoring the dependency tree.

## Recovery and provenance

The 46 moved files were hash-checked before path edits. Originals, the old root
README and the move manifest are under `.local/repository_layout_20261009/`.
The earlier application migration records are now under
`.local/migration_app_20261009/` and `.local/migration_20261009/`.
The five manual hardware examples and the canonical trajectory are moved, not
rewritten to bypass any control. Runtime state, recorded datasets and frozen
native binaries have not been relocated.

Backups are inactive evidence, not alternate executors. Inspect exact paths and
junction targets before restoring anything. Do not recursively delete a shared
directory or junction. Existing run-specific verifiers can legitimately reject
newly edited source; consult frozen source evidence instead of rewriting hashes.

## Before publishing

Review third-party SDK/model/CAD notices, private logs, local IPs and author
information. No public repository, remote, commit, release or project-wide
license is created by this local cleanup. Recordings and audit backups are not
included in the Git-facing project view.
