# Piper application and GUI

This is the active application source location following the 9 October 2026 migration.
Start with the [main guide](../docs/START_HERE.md). Run the `../piper.cmd` launcher;
use `doctor` to check the installation without accessing hardware.

`web/` contains the GUI and frontend tests; `assets/wrist_cad/` contains D405 meshes.
`app_paths.py` explicitly identifies shared dependencies. State/interlock, SDK,
environments, official URDF and research archives are not duplicated here.

Developer commands: `npm test` and `npm run build`. Neither moves the robot.
Node dependencies remain shared through a junction. Full tests and migration
reports are described in the [migration status](../docs/MIGRATION.md).

Do not run backup sources in `../.local/migration_app_20261009/`; they are retained
for auditing and recovery. Edit active sources here, not the legacy adapters.
