# D405 wrist CAD import and assembly registration

## Active viewer-only seating correction

`fitted_mount.json` supersedes the earlier bounding-box half-turn below. The user
authorized a derived lower mesh and moving the camera/upper mount after observing
tab penetration. The original STEP, IAM, three converted STLs and registration
remain unchanged. Disable this fit to return to the legacy assembly override.

`fit_wrist_mount.py` makes `realsense_d405_mounting_bot_fitted.stl`: screw pitch
12.5 -> 12 mm (3 mm diameter unchanged), recess roof X=44 -> 43.4 mm, providing
0.1 mm clearance above the official gripper tab. The two screw axes align with
gripper bores at X=-36 mm, Y=+/-6 mm. The upper assembly moves as one rigid body
to make its hinge coaxial with the lower mount, with interleaved fork fingers.
These edits adapt visualization to the official mesh; they are NOT approved
manufacturing dimensions, collision-safety certification, or optical calibration.

Run `.venv/Scripts/python.exe fit_wrist_mount.py`, followed by
`.venv/Scripts/python.exe -m unittest test_fitted_mount test_web_monitor`.
Development-only geometry dependencies: trimesh 4.7.4, manifold3d 3.5.3 and rtree
1.4.1. The generator validates watertightness, source hashes, boolean intersections
and sampled penetration before activating configuration. Further tests fit actual
exported hole rims and check hinge/camera transforms. Numerical contact tolerance
is 0.02 cubic mm for boolean intersections; the lower/gripper intersection is zero.
The report is `output/wrist_cad/fitted_mount_validation.json`.

## Confirmed lower-mount reversal

The user requested a 180-degree reversal of **only the lower mount**, based on
their physical photo, while leaving the camera and upper mount unchanged.
`bottom_mount_flip.json` records that override separately from the original STEP
and mesh-registration result. Its pivot is the centre of the lower part's
12.5 x 36 mm foot in the original part frame; local X is the upright axis.
The half-turn preserves that pivot, reverses the screw-facing side, and changes
neither the upper mount nor the camera transform. Set `enabled` to false to
restore the original CAD arrangement. Regenerating CAD assets preserves this
separate override. This remains a visual configuration, not optical calibration.

Source: `arm_with_wirst_mount_d405.stp` in the parent workspace.
Original source is not changed. Conversion is entirely local using
`occt-import-js@0.0.23` (OpenCascade); this is a development-only dependency.

Run `node import_wrist_step.mjs` to regenerate the three STL files and manifest.
Run `node test_wrist_cad.mjs` to verify all triangle vertices round-trip into the
original STEP assembly coordinates within 1 micrometre.

The source contains three solid parts:

- `realsense_d405_mounting_bot`
- `realsense_d405_mounting_top`
- `RealSense_D405`

The `piper_factor_1000.0` component is empty geometry: STEP entity #10474 is a
SHAPE_REPRESENTATION containing only the coordinate placement #3737. It has no
solid, shell, surface, tessellation, or connection to an alternate geometry
representation. The OpenCascade import independently reports no meshes for it.

The user subsequently supplied the original Inventor assembly:
`C:/Users/teuku/Documents/kubota_lab/moonshot_demo/2026_06/ranger_mini_3/arm_with_wirst_mount_d405.iam`.
Its occurrences were read through the running Inventor COM API without saving
or altering the document. It references `piper_factor_1000.0.ipt`; the matching
`piper_factor_1000.0.stl` is present in the same directory. Its positions were
converted from millimetres to metres and transformed into assembly space using
the occurrence matrix (Inventor translation units: centimetres).

The official viewer gripper mesh was registered to this robot STL using 24
orientation seeds, point ICP and point-to-plane surface refinement. On 6000
independent surface samples: median distance 0.039 mm, RMS 0.868 mm, 95th percentile
1.986 mm. These are geometric fit residuals, NOT a guaranteed physical accuracy.
The two CAD descriptions have residual surface differences.

Each exported STL is expressed in its own original CAD product frame, in metres.
The manifest stores its fixed transform relative to the mounting_top frame and
the latter's original assembly transform. Relative geometry between all three
parts is preserved within 0.044 micrometres of the tessellated STEP result.
STL has no material colors; the viewer uses dark mounting and light camera
finishes. The source mesh colors are retained in the import JSON.

`attachment.json` stores the derived mounting transform relative to
`gripper_base`. The camera and both mounting parts are connected by fixed joints,
so all follow joint 6, not finger opening. The mounting frame, component frames,
and meshes are visual CAD geometry, **not calibrated optical extrinsics**.
No point cloud, field of view or camera-to-robot calibration is asserted.

Reproduction (local, no robot access):

1. `powershell.exe -NoProfile -STA -File inspect_inventor.ps1` reads the open
   assembly, or opens it invisibly and closes without saving.
2. `node import_wrist_step.mjs` converts the source CAD.
3. `.venv/Scripts/python.exe register_wrist_cad.py` performs coarse registration.
4. `.venv/Scripts/python.exe refine_wrist_registration.py` refines and validates.
5. `.venv/Scripts/python.exe activate_wrist_attachment.py` checks quality gates
   and writes the attachment configuration.
6. `node test_wrist_cad.mjs` checks exported geometry and the calibration label.

Registration-only dependencies: trimesh 4.7.4 and rtree 1.4.1, installed in the
project venv. They are not imported by the live robot monitor.
