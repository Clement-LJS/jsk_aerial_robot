# Beetle module, frame and center-of-mass inspector

This ROS1 viewer uses Python Qt controls and ordinary RViz markers. It observes a
running robot or previews a local URDF using internal forward kinematics. It does
not start bringup, write robot descriptions, broadcast TF, publish joint states,
or send robot commands. Source execution requires no package build.

The supplied reference records are **conditional migration examples from the
current URDF**, not independently obtained CAD data or physical measurements.
The roll assembly example assumes that its original CAD CoG was expressed in
the STL frame. Verify that assumption before using any suggested correction.
Flight inertials, visuals, collisions, joint definitions and controller/pivot
parameters are never edited by the viewer.

## Start here: inspect one STL module

**RViz is the 3D display. The launch command below starts both RViz and the control
window automatically. You do not need to open another RViz or start robot bringup.**

On this workstation, run these exact commands in a terminal:

```bash
source /opt/ros/one/setup.bash
source /home/clement/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch gimbalrotor module_cog_viewer.launch instance:=inspect_roll preset:=roll_yaw
```

`roslaunch` starts a ROS master if needed. The model is loaded from the local Beetle
Xacro automatically; you do not need a running robot or a description parameter.

1. In the **Beetle module / frame / CoG inspector** window, type
   `roll_joint_asm_link` in **Search links**, then click that link's row.
2. Click **Start assembly from selection**, then **Isolate geometry**.
3. Look at the **RViz** window. The mesh, coordinate axes and red URDF CoG are on
   by default. The initial camera is focused on this roll module. **Move Camera**
   is selected in the RViz toolbar so mouse drags change the view immediately.
4. Read the annotations: **Link origin** is the link's `[0,0,0]`; **Mesh origin 0**
   is the STL origin positioned by `<visual><origin>`; **Inertial / CoG axes** sit
   at the CoG positioned/oriented by `<inertial><origin>`. Leader lines connect
   the labels to their actual locations. Arrow colors are X red, Y green, Z blue.
5. In the inspector's **Mass / frames / audit** tab, read the **declared** row:
   `.28288 kg` and local CoG `[.02057, 0, -.03110] m`. Hover truncated cells for
   their full value, or scroll the table horizontally. The STL origin is separately
   offset by `[.027, 0, 0] m`. Coordinates in the fixed frame also include the
   upstream joints.
6. For another STL, select the URDF link containing it, then repeat step 2. The
   link supplies the mass/inertia; a bare STL does not contain these properties.

Reference, candidate, ghost and joint overlays start off to keep this view readable.
Enable them under **Display overlays** for comparisons or assembly work. With one
selected mass, its CoG and combined CoG coincide and share one label.

### Rotate, pan and zoom with the mouse

Use the mouse **inside RViz's 3D scene**, with **Move Camera** selected in the top
toolbar. Click **Move Camera** again whenever you want to leave another tool.
Keep **Views → Type** set to **Orbit (rviz)**.

| Mouse action in the 3D scene | Result |
| --- | --- |
| Hold left button and drag | Rotate the camera around the module |
| Hold middle button / wheel and drag | Pan the camera sideways/up/down |
| Hold Shift + left button and drag | Pan, useful without a middle button |
| Scroll the wheel | Zoom in/out |
| Hold right button and drag | Zoom in/out |
| Click **Focus Camera**, then click the STL | Set the clicked surface as the orbit center |

Camera motion does not move the modeled module or change its mass/CoG. To change
the modeled configuration, use the inspector's **Joint positions** or **Preview
origins** controls. After updating this configuration, close/relaunch the viewer
to load the camera toolbar. An already-open RViz can also load
`config/visualization/module_cog_viewer.rviz` through **File → Open Config**;
check its fixed frame/topic afterwards against the inspector window.

If RViz is blank, its **Global Options → Fixed Frame** must be
`cog_viz_inspect_roll/world`; the MarkerArray display's **Marker Topic** must be
`/inspect_roll/viewer/markers`. The launch sets both. In a pure preview, a global
warning about absent TF can appear because no TF is broadcast; the MarkerArray
display should still report **Status: Ok** and show the scene.

### Graphics fallback on this workstation

The reported GLX crash was reproduced: the loaded NVIDIA kernel module reports
`580.173.02` while the NVML user-space library reports `580.178`. Native `glxinfo -B`
also failed. Mesa llvmpipe succeeded with **both** environment selections below.
The launch now applies them only to its RViz process by default:

```text
__GLX_VENDOR_LIBRARY_NAME=mesa
LIBGL_ALWAYS_SOFTWARE=1
```

The normal launch command above therefore works without changing shell/system
graphics settings. `LIBGL_ALWAYS_SOFTWARE=1` alone did not fix this GLVND/NVIDIA
configuration. To explicitly use hardware graphics after the driver stack is
working, add `software_rendering:=false`. No drivers were installed or modified.

## Alternative source-only startup

Use this only if you deliberately want to start the controls and RViz separately.
Use an already configured ROS1 environment and running ROS master. From this
package folder:

```bash
python3 scripts/visualization/module_cog_viewer.py __ns:=/inspect_roll __name:=viewer _preset:=roll_yaw _scene_id:=inspect_roll
```

In another terminal in the same package folder and ROS environment, run:

```bash
env __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 rviz -d config/visualization/module_cog_viewer.rviz -f cog_viz_inspect_roll/world cog_viewer_markers:=/inspect_roll/viewer/markers
```

The Marker Topic is `/inspect_roll/viewer/markers` and Fixed Frame is
`cog_viz_inspect_roll/world`, matching the explicit scene ID above. No TF anchor is
needed: every preview marker is already in that frame. Imports and resources are
resolved relative to the script/package, so the script can also be called by its
absolute path from another working directory.

For normal use, prefer the one-command launch, which configures graphics, topic
and fixed frame automatically:

```bash
roslaunch gimbalrotor module_cog_viewer.launch instance:=inspect_roll preset:=roll_yaw
```

`rviz:=false` runs just the controls. Use a distinct `instance` for each launch.
The default instance uses roslaunch's anonymous naming. Private marker/status
topics, marker namespaces, and frame IDs are separate for each instance. When
switching preview/live mode in an existing window, update RViz's Fixed Frame to
the window's new value. RViz camera focus can be moved using its ordinary Focus
Camera tool; selecting/isolating a link does not automatically move the camera.

Supported `preset` values:

| Preset | Model arguments / observed local connectivity |
| --- | --- |
| `roll_yaw` | roll-yaw enabled; connected base → yaw pipe → roll assembly → hand |
| `pitch_only` | pitch-only enabled; pitch pipe and hand currently disconnected |
| `pitch_roll` | roll enabled; local commented joints leave disconnected modules |
| `pitch_yaw` | yaw enabled; local commented joints leave disconnected modules |
| `basic` | perching variants disabled; base, saw and rotor modules |

All mutually exclusive flags are set explicitly. The wrapper under
`robots/beetle/` is expanded, not the macro definition under `urdf/beetle/`.
Invalid models are rejected before replacing the current generation. Disconnected
components can be inspected separately; selecting one anchors that component's
root at identity in preview. Other components have unavailable poses. There is
no guessed mounting joint, and cross-component chain attachment is refused.

To load an expanded local URDF, use the source field/Browse button or:

```bash
python3 scripts/visualization/module_cog_viewer.py _model_source:=/absolute/path/model.urdf _preset:=roll_yaw
```

The preset also selects the reference **variant identity**. Choose it correctly
for files/parameters: a shared link name cannot establish which physical assembly
was loaded. Mesh URI, scale and hash provide additional record checks. Arbitrary
custom variants without records can still use declared mode.

## Inspect and assemble

The searchable tree follows the actual link/joint hierarchy. Its selection is the
inspected module. Its **Geometry** checkboxes control visibility; its **Mass**
checkboxes control aggregate membership. Selecting a row changes neither.

1. Select `roll_joint_asm_link`. Press **Start assembly from selection**, then
   **Isolate geometry**. The selected mass set now contains just that link.
2. In Display overlays enable meshes, axes, ghost, labels, declared and reference.
   Enable candidate to inspect the explicit mesh-follow assumption. Link, selected
   mesh, saved CAD and URDF inertial axes are separately named. Red spheres are
   declared; green means verified source/frame/calibration; amber means assumptions
   remain. Larger spheres are complete selected-set aggregates.
3. Press **Add child**, select `roll_hand_link via joint_roll`. Geometry and mass
   are added using that actual joint. **Add parent** similarly adds `yaw_pipe_link`
   through `joint_yaw` when the roll assembly is selected.
4. In Joint positions move the slider or numeric field for `joint_roll` or
   `joint_yaw`. URDF limits apply. Continuous joints have a ±pi **viewing range**,
   not a claimed physical limit. Mimic joints derive from the source joint and
   are read-only. Out-of-limit mimic configurations are unavailable, not clamped.
5. Uncheck the hand's Geometry checkbox: mass and aggregate stay unchanged.
   Uncheck its Mass checkbox: the aggregate becomes the roll assembly alone.
6. **Add chain to…** finds an undirected path through actual model joints.
   The action lists connector links. With “Include connector masses” checked,
   all path bodies enter membership. Otherwise connectors only position/show
   the modules and only the target is added to membership. Existing membership
   stays explicit in the checkboxes. Preset groups are sets, so adding overlapping
   groups or a group plus a member never counts a body twice.

“All declared URDF masses” explicitly selects every inertial link, including small
helper masses. The audit also always computes this separate all-declared result
and lists differences from the selected set. No mass is excluded for being small,
or because a source comment calls a link virtual. Selecting a massless link for
a physical assembly makes that assembly incomplete; it is not a zero-mass body.

## Preview edits and reference calibration

Preview origins provides absolute numeric XYZ/RPY editors and reset buttons:

* A **visual-only override** changes the current mesh placement in its link.
  Declared CoG, saved CAD reference CoG and reference ghost stay fixed locally.
  A mesh-placement diagnostic changes even if declared/reference CoG discrepancy
  does not. The enabled candidate follows the changed mesh.
* A **joint-origin override** changes the child/subtree attachment. Its meshes,
  declared/reference/candidate CoGs all move with the child. Joint positions rotate
  around the joint's own axis after the origin transform.
* Display separation (“explosion”), opacity, axes, labels and camera are rendering
  settings. They do not feed any calculation. Aggregate markers remain at their
  physical calculation coordinates, so an exploded scene deliberately separates
  drawings from the aggregate. Set separation to zero for physical comparison.

The CAD import/calibration tab accepts original mass and CoG in kg/g and m/mm,
CAD-to-normalized-mesh XYZ/RPY, saved CAD-to-link XYZ/RPY, provenance and three
independent trust states. Enter the original coordinates, not a previously
corrected URDF CoG. Applying those fields creates an explicit viewer record
revision; it does not infer truth from agreement with URDF. Original inputs/units
are recorded. Changing original properties through this form discards any old
tensor, since its basis/distribution may no longer be justified.
Leave “CAD → mesh relationship available” unchecked when that relationship is
unknown; it stays missing rather than being replaced by identity.

**Accept current mesh placement as saved reference** explicitly freezes the
candidate placement into the session's saved calibration, labeled assumed. This
requires an unambiguous whole-body visual and enabled candidate. It never upgrades
source/frame trust. **Save viewer reference YAML** writes an independent file in
the viewer export folder. **Import reference YAML** validates a registry before
replacing it and establishes the reset-to-original imported baseline. **Reset
references** restores that baseline. Model reload never imports/reseeds CAD values.

For detailed data, import the version-1 schema shown in
`config/visualization/beetle_mass_reference.yaml`:

* One record per `(variant, link)` and unique physical `instance_id` within variant.
  Independent copies may share an STL, but must have distinct instance IDs/links.
* Exact mesh URI, named visual or resolved index, scale, mesh SHA256, original
  visual fingerprint, whole-record visual coverage, and registration model SHA256.
  An unrelated model hash change does not invalidate an otherwise valid record.
  A changed visual-origin fingerprint warns to review index binding but retains
  the saved calibration. Changed mesh/scale/hash makes that mapping unavailable.
* Immutable source mass/CoG, CAD revision/note, optional CoG tensor and its axes,
  source trust; separate `cad_to_mesh` and `calibrated_cad_to_link` trust.
  SI field suffixes are mandatory; registry `units`, if present, must be SI.
  A separately verified direct CAD-to-link calibration can establish a reference
  even with `cad_to_mesh: null`; in that case mesh candidate/ghost are unavailable.
  The mesh-frame relationship is not an extra prerequisite for that direct reference.
* Optional `constituents`, `overlaps_with` (record IDs), `composition_status`.
  Known overlaps suppress totals. Unknown composition stays **unverified** even
  when all requested numerical inputs are present. Populate these fields for base
  CAD assemblies that might already contain separately modeled modules.

An optional tensor must use symmetric matrix entries about its CoG, in kg*m²,
with `off_diagonal_convention: tensor_entries`. Convert CAD products-of-inertia
signs explicitly before import; the viewer does not guess them. Positive moments,
symmetry and triangle inequalities are checked with tolerance
`max(1e-12, max(abs(principal moments))*1e-8)`. Tensor orientation is transformed
using `inertia_axes_rpy_in_cad`; translation alone never shifts a CoG tensor.
No STL centroid, density estimator, bounds or convex-hull claim is made here.

## Coordinate conventions and logic review

All internal quantities use metres, kilograms, radians and kg*m². Column-vector
`T_A_B` maps B coordinates into A. URDF RPY is `Rz(yaw) Ry(pitch) Rx(roll)`.

| Source | Local CoG in link L | Meaning |
| --- | --- | --- |
| Declared | inertial `origin.xyz` | Current URDF declaration |
| Saved reference | `T_L_A_saved * c_A` | Independent CAD coordinates and saved placement |
| Mesh candidate | `T_L_M_current * T_M_A * c_A` | Explicit whole-module placement assumption |

Each local CoG is transformed by `T_F_L` **exactly once**. Inertial RPY rotates
the inertia axes/tensor; it does not rotate inertial XYZ a second time. Preview
uses `T_F_parent * T_parent_joint_origin * T_joint_motion(q)`. Live uses TF directly,
with no extra FK. Legacy markers retain local coordinates and let RViz apply TF.
The mounting yaw remains the model's `3.142`, not an assumed exact pi.

Mesh scale `.001` converts STL millimetres into metres. Already-SI CAD CoGs are
never scaled again. Scaling geometry physically is a different operation: changed,
negative or nonuniform scales are flagged against the recorded scale; they do
not produce invented masses/inertias. Multiple visuals are one link mass, not
multiple bodies. Multi-visual/partial-body mesh following is refused without
component mass data; declared/reference results remain available.

Totals use `sum(m*c)/sum(m)` for unique selected links, separately per source.
Any missing mass, reference, pose, unsupported joint subtree or known overlap
suppresses the complete total. A known-subset mass/CoG, included/missing/excluded
lists and completeness status are explicit in the audit/JSON. Different source
masses or membership are displayed, so a difference between their aggregates is
not described as solely a coordinate-offset error.

The conditional roll reference is `[.04757, 0, -.03110]` m, derived once from the
saved original `[.02057, 0, -.03110]` and assumed `.027` placement. With hand added,
the synthetic reference fixture is `.59094 kg`, with CoG in the roll assembly
frame `[.119223377669476, -.0122610945273632, -.0460197502284496]` m at zero roll.
These are regression fixtures under stated assumptions, **not physical validation**.
The private scene normally uses the component root, so its displayed coordinates
also contain all actual upstream joint transforms.

## Read-only live observation

```bash
roslaunch gimbalrotor module_cog_viewer.launch instance:=observe_roll pose_source:=live preset:=roll_yaw robot_description_param:=/gimbalrotor/robot_description tf_prefix:=gimbalrotor reference_frame:=gimbalrotor/base_link
```

Live loads the selected description parameter and uses external TF. The frame can
instead be `gimbalrotor/fc` or `world`; those frames are not interchangeable. Prefixes
and leading slashes are normalized once. Joint-position/origin/visual editors are
disabled in live mode. Reset preview overrides before switching an edited preview
to live; no servo signs/scales/offsets are applied to model joint coordinates.

All TF requests use one time: `ROS now - ~latency` (default .1 s), without blocking
TF waits. `~max_pose_age` (default .5 s) bounds allowable latency; exact-time TF
queries reject missing/stale dynamic chains rather than using independent latest
poses. Static transforms are valid across time, including zero/old stored stamps.
Zero-stamped dynamic data and data more than .05 s in the future are rejected and
reported. A paused clock is labeled paused; wall-time UI/status updates continue.
A backward clock jump recreates the private TF listener/buffer and requires fresh
consistent poses. Global `use_sim_time` is never changed.

**Copy live snapshot to private preview** infers model joint values from a complete
component TF snapshot and retains its root placement in a private preview frame.
This succeeds only if all component poses agree with model FK within 1 mm / .01 rad;
unsupported joints, missing poses, out-of-limit/mismatched mimic configurations
are refused. Continuous turns cannot be recovered from TF alone and are labeled
ambiguous. Set RViz Fixed Frame to the newly printed private frame, then experiment.
This creates no messages on the robot's state/TF topics.

The description hash is checked periodically in live mode. A changed external
parameter requires explicit model reload. Loading XML from disk updates the viewer
generation, not a robot_state_publisher running elsewhere. Model/TF calibration
agreement remains explicitly unverified; without joint feedback the observer
cannot identify every fixed-origin/model mismatch. TF observation is not a
measurement of mass properties or proof that an edited C++ source was executed.

## Headless and legacy modes

```bash
roslaunch gimbalrotor cog_markers.launch instance:=audit_roll preset:=roll_yaw show_candidate:=true
python3 scripts/visualization/cog_markers.py __ns:=/audit_roll __name:=markers _only_links:=roll_joint_asm_link,roll_hand_link _show_candidate:=true
```

Headless publishes `~markers` and `~status` without importing Qt. A live headless
scene uses the same `pose_source`, `robot_description_param`, `tf_prefix` and
`reference_frame` parameters as the GUI. Configure `~rate` (1–60 Hz, default 25),
`~model_source`, `~reference_file`, `~only_links` (comma list or list), `~latency`,
`~max_pose_age`, `~show_candidate`, `~marker_topic` as needed.
`~discrepancy_threshold_mm` controls the comparison threshold (default 1 mm).
Change its private
`~reload_token` to request an explicit transactional model reload, e.g.
`rosparam set /audit_roll/markers/reload_token 1`. This edits only a viewer control
parameter; the viewer itself never writes parameters. Invalid reload retains
the previous model/registry and reports the error.

The old entry point remains compatible:

```bash
python3 scripts/urdf/urdf_inertia_markers.py _robot_description_param:=/gimbalrotor/robot_description _tf_prefix:=gimbalrotor _only_links:=saw
```

Its default parameter remains `/robot_description`, default output remains
`/urdf/inertia_markers`, and `~only_links` still filters links. It displays **only
declared** local CoGs, with frame-locked markers and correctly rotated inertial
axes. It does not produce misleading link-local aggregate coordinates. Use the new
comparison entry point for independent sources and common-frame aggregates.
Every entry point deletes obsolete/closed markers only by its own namespace/ID;
no DELETEALL is sent. A forced process kill cannot perform clean shutdown deletion.

`~status` is `std_msgs/String` containing version-1 JSON: instance, model source/hash,
generation, variant, pose source/note, fixed frame, snapshot timestamp, enabled
overlays, rows with local/fixed CoGs/masses/provenance/registration diagnostics,
discrepancy threshold/results, independent selected aggregates, all-declared
aggregate and membership differences. `complete` means all requested numerical
inputs were present without known overlap. Composition/calibration status remains
separate. Subset fields never imply a complete assembly.

## Save / reload / export

The Save button creates `exports/cog_viewer/<instance>/` under this package. It
writes `session.yaml`, `reference.yaml`, `report.json`, `report.csv` and
`candidate_origins.xml`. Session YAML embeds the expanded model, original source,
variant, membership, display selections, joint values, absolute preview overrides,
root pose and independent reference records. Load validates everything before
replacing the session and always starts as an isolated preview. It does not
implicitly reconnect to a live robot. Imports may be read from other paths, but
export writes are confined to this viewer folder, including symlink checks.

Origin suggestions are explicitly partial, with original/suggested values and
provenance; no complete mass/tensor block is fabricated or applied. Review mass
composition, source axes and calibration before applying anything manually to a
production model. Optional aggregate inertia, image capture and geometric mass
estimation are not implemented. Normal installed entry points/helpers/resources
are included in CMake rules; export requires a writable project/package folder.

## Offline checks and user-run acceptance

```bash
python3 -B scripts/visualization/tests/test_cog_tools.py
```

The offline checks cover immutable offsets/reload, rotation order/quaternion norms,
rotated-origin revolute/prismatic FK and ancestors, weighted roll/hand fixtures,
display/reference/candidate separation, unit conversion/tensor orientation,
membership/overlap/missing data, disconnected models, mimic cycles/limits,
variant/scale/multi-visual mismatches, transactional session/report exports, and
marker ownership/locking/timestamps. Local Xacro expansion validates all five
presets if dependencies exist. Offline message construction and a fake TF buffer
exercise coherent-time, incomplete, paused and backward-time paths without
initializing a ROS node. Optional dependencies are reported skipped if absent.

For opt-in real desktop testing, with the same sourced ROS environment and X11
display (the local test uses the already-installed `wmctrl` and RViz Python bindings):

```bash
python3 -B scripts/visualization/tests/test_runtime.py
```

This starts its own ROS master on an unused localhost port, opens/closes its own
test windows, and writes results/logs/screenshots under
`exports/cog_viewer/runtime_validation/`. It does not connect to the robot's master.
It verifies the exact documented launch (with a clean parent graphics environment),
the real Qt selection/attachment/edit controls, all five presets, invalid reload
retention, actual RViz rendering and display status, exports/restore, real static
and dynamic TF, missing/stale/future TF, and the built headless/legacy entry points.
The visualization package was also built with `catkin build gimbalrotor --no-deps`.
These runtime checks passed on this workstation on 2026-11-04. Real robot TF,
hardware operation, and paused/backward simulated clock behavior on a running ROS
graph remain outside that runtime check; the clock paths have offline fixtures.

Use this checklist to repeat the full workflow manually:

1. Launch the standalone viewer with `roll_yaw`; RViz should display its private scene.
2. Select `roll_joint_asm_link`, start a new assembly, isolate it: only its mesh is
   visible and selected mass is `.28288 kg`.
3. Enable mesh/link/CAD/inertial axes, ghost and declared/reference/candidate CoGs.
   The example reference/candidate X is `.04757`, declared X `.02057`, all in the
   roll link frame; assumptions must be amber and identified numerically.
4. Add `roll_hand_link` through `joint_roll`: selected mass becomes `.59094 kg`.
5. Move the roll slider: hand geometry/CoGs and aggregate move around actual +X;
   zero and +pi/2 match the documented synthetic fixture after frame conversion.
6. Hide the hand geometry: aggregate membership/mass stay unchanged.
7. Exclude the hand mass: aggregate becomes the roll assembly alone.
8. Override roll visual X from `.027` to `.037`: mesh and candidate move +10 mm;
   saved reference/ghost and declared local CoG remain fixed. Reset the visual.
9. Override a preview joint origin: its child/subtree and all attached sources move
   together. Reset all overrides.
10. Load each other preset: show missing joints/disconnected roots and reject
    cross-component attachments; do not guess a physical pose.
11. Switch to live using the selected description/TF prefix/frame. Check one
    snapshot timestamp, disabled sliders, stale/missing status, fc vs base offset,
    pause/backward clock behavior, reload retention and independent instances.
12. Save the session/report under this package, reload it as preview, and compare
    membership, joint positions, visual overrides and saved reference placements.

If RViz is blank, first check Marker Topic and Fixed Frame against the control
window/status, then check geometry checkboxes and pose diagnostics. Missing meshes
reject model reload rather than replacing the last valid model. A reference from
another variant/mesh/scale is intentionally unavailable. A missing CAD record
does not prevent declared-mode inspection. No physical CoG is established by
these software checks; CAD materials/composition and frame calibrations still need
independent verification.
