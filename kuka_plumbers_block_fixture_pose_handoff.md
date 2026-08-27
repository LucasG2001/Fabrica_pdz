# KUKA plumbers_block fixture pose + Gazebo placement — reference (2026-08-26)

## Ask
User wanted to know how the plumbers_block fixture's pose/coordinate frame is defined, whether
the KUKA bases sit at z=0, and — combining both — how to position the fixture on the table in
Gazebo's world frame.

## Status: documentation only, no code changed

## 1. Three fixture-pose conventions in this repo (they are NOT interchangeable)

| # | File | Reference point of the pose | Consumer |
|---|------|------------------------------|----------|
| 1 | `planning/run_fixture_gen.py` (`generate_fixture`, lines 241–320) | Fabrica's own absolute board/world frame (cm) | Fabrica native sim/render |
| 2 | `planning/utils/fixture_pose_to_graspplanning.py` | Relative to a chosen base part's own final table pose | Grasp_Planning handoff |
| 3 | `planning/utils/generate_plumbers_block_gz_assets.py` | Fabrica board frame rotated into the real dual-KUKA Gazebo world | gz-sim / real rig |

### 1a. Fabrica board frame (source of truth)
`generate_fixture()` builds the fixture as an axis-aligned box:
```python
box_min = np.array([-box_extent[0] / 2, min_fixture_y, 0])
box_max = np.array([box_extent[0] / 2, min_fixture_y + box_extent[1], board_height_max])
```
(`planning/run_fixture_gen.py:275-276`), where `min_fixture_y = get_fixture_min_y('kuka') = 4*dx =
10cm` (`planning/robot/workcell.py:154-161`, `get_board_dx()=2.5` at line 4-5).

There is no dedicated "fixture-center" variable — the frame is just Fabrica's shared
world/board frame:
- **X = 0**: horizontal center of the fixture footprint (box is X-symmetric).
- **Y = 0**: NOT on the fixture. `min_fixture_y` (10cm for kuka) is a positive offset inside
  `generate_fixture()`'s own box-construction formula, but that box lives in a "relative to each
  part's final pose" packing frame (`generate_pickup_pose`, line 189) that gets composed with the
  parts' actual (negative-Y) final assembly poses before export — so the **exported**
  `fixture.obj` ends up entirely on the **negative**-Y side, further from the origin than the
  final assembly. Verified 2026-08-26 against the actual `logs/plumbers_block_sim/fixture/
  fixture.obj` on disk (cross-checked against `validation.json`'s `"fixture"` footprint, which
  matches exactly): footprint is X ∈ [-12.5, 12.5] cm, **Y ∈ [-62.7, -42.7] cm** (center Y ≈
  -52.7cm), vs. the final assembly's own center around Y ≈ -15cm. Don't rely on the box-formula
  sign alone — always check the actual exported mesh/validation.json for a given run.
- **Z = 0**: bottom face of the fixture = table surface (fixture sits flush z=0 up to
  `board_height_max`, which was 4.5cm for this run's fixture.obj — bbox Z ∈ [0, 4.5] cm).

The exported `logs/.../fixture/fixture.obj` mesh is baked in this **absolute** frame — no
re-centering to a local origin (confirmed by the "no extra transform needed" comment in
`generate_plumbers_block_gz_assets.py:11-13`).

Per-part `pickup.json` poses (`run_fixture_gen.py:439-443`) are built with an **unrecorded**
uniform offset (`part_translation = box_center - part_center`, line 283) baked in to align parts
with the box — this is why convention #2 below can't reuse this frame directly.

### 1b. Grasp_Planning conversion (different convention, on purpose)
`fixture_pose_to_graspplanning.py` docstring (lines 1-33) + code (lines 89-98) derive each part's
pose relative to a **base part's own final table pose**, not fixture geometry, specifically
because `part_translation` above is never persisted to disk and can't be reconstructed:
```python
T_local_i = T_base_pickup_inv @ T_pickup_i
T_new_pre_insertion = T_base_final @ T_local_i
```
Don't mix this frame with 1a or 1c — it's intentionally anchored elsewhere.

### 1c. Gazebo / real dual-KUKA rig frame
See section 2 below — this is the one relevant to "how do I place it on the table."

## 2. KUKA base elevation above z=0

`planning/robot/workcell.py:8-21`, `get_kuka_mount_block_height()` returns **2.5 cm**, used as the
Z component of both `get_move_arm_pos('kuka')` and `get_hold_arm_pos('kuka')`
(lines 44, 91): `(±16.8*dx, 10, 2.5)` cm, i.e. **(±42, 10, 2.5) cm**.

- Every other arm (xarm7, panda, ur5e) uses z=0 (bolted flush to the tabletop).
- KUKA's real iiwa7 rig sits on a ~2.5cm riser block, so its `base_link` origin is 2.5cm above the
  true table surface. This is baked into the arm position's Z (not into the ground plane), so
  every existing z=0-relative check (`ground_col_manager`, `inverse_kinematics_above_ground`'s
  `ground_z=0` default) keeps working unmodified.
- Cross-checked against user-provided real-world measurement (comment,
  `workcell.py:60-63`): real bases at (0,±0.42,0) m in a frame centered between them, table
  surface at z=-0.025m in that frame — i.e. base is 2.5cm above table. Matches
  `get_kuka_mount_block_height()` exactly.
- `z=0` means the table/ground plane everywhere in this repo (same plane as
  `ground_col_manager`'s ground and every non-KUKA arm's base) — there is no floor-vs-table
  ambiguity here.
- The actual URDF/xacro/SDF `<origin>`/`<pose>` tags for the real rig (`lbr_dual_arm.xacro`,
  `lbr_one_base_joint` / `lbr_two_base_joint`) live in the external `Grasp_Planning` /
  `lbr_fri_ros2_stack` repos, referenced only in comments — not checked into Fabrica.
  `assets/kuka/kuka.urdf` here only has the arm's internal joint chain, no base-mount geometry.

## 3. How to place the fixture on the table in Gazebo (the actual answer)

`generate_plumbers_block_gz_assets.py:271-303` is the authoritative transform. Key facts:

- **Axis remap**: Fabrica separates the two arms along **X** (`±42cm`) with the assembly offset
  along **Y**. The real rig (`lbr_dual_arm.xacro`) separates them along **Y** instead, both at
  X=0. So Fabrica's raw (x,y) can't be spawned as-is — it would land next to one arm rather than
  out in front of the pair.
- **Fix — rotate +90° about Z, about the arm-pair midpoint**:
  ```python
  BOARD_MIDPOINT_CM = np.array([0.0, get_move_arm_pos('kuka')[1], 0.0])  # (0, 10, 0) cm
  BOARD_TO_GAZEBO_ROT = Rotation.from_euler('z', 90, degrees=True)
  ```
  (lines 280-281). Verified against both real base joints: move arm board-frame (42,10) →
  gazebo (0, 0.42)m matches `lbr_two_base_joint`; hold arm (-42,10) → gazebo (0,-0.42)m matches
  `lbr_one_base_joint`. Computed live from `workcell.py` (not hardcoded) so it can't silently
  drift out of sync the way it did on 2026-08-24 when Y changed 8*dx=20 → 10.
- **For the fixture specifically** (static mesh, already absolute in board frame): bake the
  transform directly into the mesh vertices rather than spawning with a pose —
  `board_transform_matrix()` (lines 294-303):
  ```python
  R = BOARD_TO_GAZEBO_ROT.as_matrix()
  M = np.eye(4)
  M[:3, :3] = CM_TO_M * R           # cm -> m, plus rotation
  M[:3, 3]  = -CM_TO_M * (R @ BOARD_MIDPOINT_CM)   # recenter on arm-pair midpoint
  ```
  Then spawn the fixture model at **identity pose** (line 381:
  `poses['plumbers_block_fixture'] = (zeros, identity_quat, zeros)`) — position is already
  baked into its mesh.
- **For dynamic parts** (screws, base, top, pipe — spawned via a pose, not baked mesh):
  `pose_from_pickup()` (lines 284-291) applies the same rotation to each part's
  `pickup.json` (x,y,z,rx,ry,rz) entry:
  ```python
  pos_cm_rel = np.array([x, y, z]) - BOARD_MIDPOINT_CM
  pos_m = BOARD_TO_GAZEBO_ROT.apply(pos_cm_rel) * CM_TO_M
  orientation = BOARD_TO_GAZEBO_ROT * Rotation.from_euler('xyz', [rx, ry, rz])
  ```

### Worked example: this run's fixture bounding-box center
Computed directly from `logs/plumbers_block_sim/fixture/fixture.obj` (trimesh bounds) run through
the actual `board_transform_matrix()`:
- Board frame (cm): bbox `[-12.5, -62.7, 0]` to `[12.5, -42.7, 4.5]` → center `(0, -52.7, 2.25)`.
- Gazebo world frame (m): center **`(0.627, 0.0, 0.0225)`** — i.e. ~62.7cm out along Gazebo +X
  from the arm-pair midpoint, centered on Y, 2.25cm above the table (half the 4.5cm fixture
  height).

### Recipe: placing a new/different fixture on the table in Gazebo
1. Get its pose/mesh in Fabrica's board frame (absolute cm, as produced by `run_fixture_gen.py`).
2. Subtract `BOARD_MIDPOINT_CM = (0, get_move_arm_pos('kuka')[1], 0)` cm — currently `(0, 10, 0)`.
3. Rotate +90° about Z (`BOARD_TO_GAZEBO_ROT`).
4. Convert cm → m (`CM_TO_M = 0.01`).
5. If the fixture is static and its mesh already carries an absolute board-frame position, bake
   steps 2-4 into the mesh vertices (`board_transform_matrix()`) and spawn at identity; if it's
   dynamic/spawned via pose, apply steps 2-4 to the pose directly (`pose_from_pickup()`).
6. Z: the fixture's own Z is already relative to table z=0 from step 1 (see section 1a) — no
   extra KUKA-mount-height correction is needed for the fixture itself, since
   `get_kuka_mount_block_height()` only offsets the **arm base**, not the table/fixture frame.
   The KUKA elevation only matters if you're separately reasoning about arm-to-table clearance
   (e.g. FK/IK checks), not for fixture placement.

## Known unresolved (unrelated to pose math, don't re-diagnose from this doc)
`screw_b` settles ~5cm off after physics settling despite identical collision/mass/friction setup
to `screw_a` — suspected fixture-mesh asymmetry near that slot, not a frame/transform bug. See
`generate_plumbers_block_gz_assets.py:41-54` and
[[kuka_pdz_plumbers_block_status_20260825]] (memory) / `kuka_pdz_gazebo_collision_tuning_handoff.md`
for what was already tried.
