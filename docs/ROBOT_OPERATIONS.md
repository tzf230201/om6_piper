# Piper operations: wake-up, state, maintenance and motion

This guide indexes the implementations on this PC. It is not safety certification
or an instruction to bypass a failed gate. Folder changes do not establish motion
readiness. Do not run two CAN/RealSense owners simultaneously.
Commands below use `piper.cmd` from the `om6_piper` folder.

## Wake-up / starting a session

1. The operator checks mechanical mounting, cables, the motion area and the physical
   emergency stop. Controller power, USB connection and motor enable are three
   different states. Releasing the emergency stop is not a home command.
2. `piper.cmd doctor` is an offline check only. Choose one hardware owner:
   `piper.cmd gui --no-fgng --open` for monitoring or
   `piper.cmd state --seconds 5` for a snapshot. Neither sends enable/target commands;
   ordinary readers may query firmware identity. Close the monitor before control.
3. Check timestamps/freshness, all drivers, controller mode and faults; do not rely
   solely on `arm_status.err_code == 0`. An old file is not live feedback.
4. Run the existing commissioning program's preflight:

   ```powershell
   .\piper.cmd commission --mode preflight --output .\output\preflight_baru
   ```

   The folder must be new. Preflight does not enable motors or prove a physical stop.
   `enable-stop`/`jog` still require operator flags and the existing gates. The launcher
   does not add `--execute` or change `commissioning_interlock.json`.
5. In scans previously run by the operator, enabling occurs inside `trajectory_test.py`
   after confirmation and start/camera checks. Opening the GUI/launcher does not
   automatically enable motors.

There is no universal wake-up script that automatically resets, clears, enables
and moves. Read legacy `read_joint.py`, `gripper_test.py`, `gripper_piper_sdk.py`
and `joint_raw_test.py` before use: some have hardware effects at module scope.
Do not import them to check installation. The launcher does not import or run them.
They are now isolated in `../examples/manual/`. The active trajectory is in
`../src/trajectory_test.py`; the root adapter retains compatibility with old commands.

## Clear errors once

`piper.cmd clear-error --help` shows program options. Without `--execute-once`,
it performs observation/preflight and firmware queries: it is not fully offline,
but sends no clear command. The actual operation is available as:

```powershell
.\piper.cmd clear-error --execute-once --output .\output\clear_baru
```

Use only after the operator checks the fault cause and supports the arm. The
existing policy requires idle/disabled/fresh state and sends at most one
`clear_joint_error(255)`. It does not enable, send targets, change limits or reset
automatically. `flags_persist_no_retry` is not success; do not silently retry.
Save `before.json`, `result.json` and subsequent observations.

## Reset/recovery differs from clearing

`piper.cmd recover --help` forwards to `recover_piper_state_once.py`.
Without `--execute-once`, it only performs preflight/observation. Execution requires
`--operator-stable` and the existing disabled/pose checks. The program may reset
once, wait for Standby, then clear if the next gate passes. Reset does not guarantee
that the arm will stay still. Errors do not trigger an automatic reset.
The reset-only version is `../piper_readonly/reset_piper_disabled_once.py`.
Permits and commissioning records were not changed during consolidation.
The authoritative file remains `../../piper_readonly/commissioning_interlock.json`.
Relocated sources received path adjustments, so old permission evidence tied to
executor hashes is not considered valid for the new sources. Migration did not
renew, grant or use any permit.

## Disable and stop

`piper.cmd disable` alone displays an offline plan. Hardware mode requires
`--execute-once --operator-supported --output ...`. The program checks feedback
after TX; the SDK return value alone is insufficient. Disabling can remove holding
torque and allow the arm to fall. Damping stop, disabling and power removal differ;
closing the browser does not stop the robot. Ctrl+C during a scan does not confirm
that all motors are disabled. This is not certified emergency-stop software.

## Tabletop scanning, vision recording and map-live

The following command can move the robot after confirmation. The operator must
review the plan and actual area, including table height, gripper and cables:

```powershell
.\piper.cmd scan --table-plan ..\output\table_scan_20261007_v1\plan.json --map-live --vision-output .\output\table_scan_baru
```

The plan remains in the original workspace; new output goes into `om6_piper`.
Mapping viewer: `http://127.0.0.1:8770/`. The recorder shares the existing robot
connection, waits for mapping startup before enabling, and does not open a second
CAN connection. CAD previews are not collision certification. The recorder does
not overwrite old recordings.

The launcher does not change firmware drivers, CAN backend, units, limits, payload,
reset/clear/disable policies, interlock or trajectories. Forwarding uses the same
operator arguments and console, without automatic confirmation, background motion
or retries. No hardware readings or commands were made during consolidation.
