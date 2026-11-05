# Mechanics, CAD and camera coordinates

## Camera mount design source

The user identified
[RealSense wrist mount for PiPER arm — GrabCAD](https://grabcad.com/library/realsense-wrist-mount-for-piper-arm-1)
as the camera mount design source on 9 October 2026.

The page returned HTTP 403 during inspection, so the author, license, version and
agreement between the download and local CAD have not been verified. This link
records the user's reference; it does not establish matching hashes or permission
to redistribute. No downloads or local CAD replacements were made in that update.

Distinguish the source design, local files listed in `copy_manifest.json`, and
derived viewer meshes adjusted to seat on the official gripper mesh. The visual
corrections are described below; the source link does not change the active camera
transform or mounting configuration.

## CAD collection

`../cad/source/arm_with_wirst_mount_d405.stp` is an identical copy of the user's STEP.
`../cad/inventor/` contains the IAM, four IPT files listed in the earlier Inventor
report, and a supporting arm STL. `../cad/reference/` retains the D405 reference
copies already in use. Hashes and original locations are in `../cad/copy_manifest.json`.
Original files in `kubota_lab/moonshot_demo` were not moved or edited.

This is a collection of direct dependencies based on `inventor_assembly.json`,
**not a verified Inventor Pack and Go**. Stored IAM references may still point to
original paths; resolve them to the local IPT copies when opening the assembly.
No Inventor COM calls, regeneration, constraint changes or CAD saves were performed.

The STEP contains only three camera/mount solids. Its Piper component contains
no robot solids. The arm mesh comes from separate STL/URDF files; this STEP is
not a complete robot assembly.

## Active viewer model

Active locations: `../../piper_readonly/piper_model_source/` for robot URDF/meshes,
and `../piper_readonly/assets/wrist_cad/` inside `om6_piper` for the three D405 parts.
The legacy wrist asset path points to this same new folder. `attachment.json`
defines the pose relative to `gripper_base`; `fitted_mount.json` defines the active
viewer seating corrections. The mount and camera follow joint 6, not finger
opening. `bottom_mount_flip.json` retains an earlier reversal experiment.

The lower mesh correction changes hole pitch from 12.5 to 12 mm and adds a 0.6 mm
recess to fit the tab on the official mesh. The upper camera/mount moves as a rigid
body to align the hinge. These are **derived visualization meshes**, not approved
manufacturing dimensions, collision certification or hand-eye calibration.
Original CAD, initially converted STL files and fit reports are retained.

Full explanation: [CAD provenance](../piper_readonly/assets/wrist_cad/README.md).
Tools: `import_wrist_step.mjs`, `inspect_inventor.ps1`, `register_wrist_cad.py`,
`refine_wrist_registration.py`, `fit_wrist_mount.py`, `test_fitted_mount.py`.
Do not run generators merely to check paths: some rewrite assets/configuration.
Tests that are fully offline are kept separate.

## Camera transforms

Base model:

`T_base_optical(q) = T_base_gripper(q) × T_gripper_camera_body × T_body_optical`.

The nominal D405 estimate is in
`../../output/cad_optical_20261005_v1/fgng_config.cad_nominal.json`:
`body_from_depth_optical.xyz = [0.009, 0, -0.0038] m`,
`rpy = [0, 0, pi] rad`. Optical X/Y oppose CAD X/Y; Z points the same way.
This is a CAD + vendor mechanical estimate with `calibrated: false`, not a
measured hand-eye result or an error bound.

The legacy viewer lifts the robot visualization by +20 mm in world Z; the older
F-GNG mode also has a configured world offset. Newer tabletop replay/scans use
the physical base frame: `table_scan.configuration()` sets the mapping offset
to `[0,0,0]`, separately from the display lift. Do not add 20 mm twice, change CAD,
or interpret display values as physical positions.

The tabletop plan is in `../../output/table_scan_20261007_v1/plan.json`. Table height
is a planning assumption; the mesh preview does not fully check cables, fingers,
the environment or self-collision. Consolidation did not change kinematics,
calibration or mounting.
