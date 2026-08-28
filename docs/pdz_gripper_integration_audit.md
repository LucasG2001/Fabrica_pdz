# PDZ gripper — repo integration audit + fixture-gen Franka-hardcoding review (2026-08-28)

## Ask

1. A new `pdz_gripper_description` URDF package is sitting in `~/Downloads`. Is it implemented in
   Fabrica for path planning etc.?
2. Analyze how fixture generation is affected by the PDZ gripper — specifically whether values
   were hardcoded for the Franka/Panda gripper.

## Status: audit only, no code changed

---

## 1. Is the new `~/Downloads/pdz_gripper_description` wired into Fabrica?

**The gripper *type* is fully wired into the planning stack. The new URDF in `~/Downloads` is
not — nothing in the repo has been regenerated from it.**

### 1a. What already exists (keyed on `gripper_type.startswith('pdz')`)

Variants in use: `pdz`, `pdz-14` (14 mm pads), `pdz-mech` (mechanism only, no D405).

| Area | Location | Status |
|---|---|---|
| Grasp geometry: basis dirs, grasp-base offset, open-ratio, finger states, pad math | `planning/robot/geometry.py:157-224, 288-294, 351-356, 377-390, 413-421, 429-459, 482-493, 578-583, 653-654` | implemented |
| Grasp sampling / feasibility / retract | `planning/robot/util_grasp.py` (via `get_gripper_basis_directions`), `planning/run_grasp_gen.py` | implemented |
| RedMax sim string | `planning/robot/sim_string.py:80-113` `get_pdz_gripper_string` | implemented |
| Motion planning / IK | `planning/robot/util_arm.py:137-140` (pdz deliberately falls through to the generic basis-direction heuristic) | implemented |
| Fixture generation | `planning/run_fixture_gen.py` (dispatches on `grasps['gripper']`) | implemented **but see §2** |
| Blender render | `rendering/render_traj_blender.py:101` | implemented, **exact-match bug** — see §3 |

### 1b. Planning consumes flat OBJs, not the URDF

Nothing in `planning/`, `rendering/`, or `simulation/` reads the URDF/xacro. The planning code
loads hand-converted meshes from `assets/pdz/` and `assets/pdz_14/`
(`base.obj`, `finger_left.obj`, `finger_right.obj`, `d405.obj`) plus hardcoded constants in
`geometry.py`:

- `PDZ_JAW_TRAVEL = 3.2` cm (per-finger stroke; URDF joint upper limit `0.032`)
- `PDZ_BARE_FINGER_GAP = 2.8` cm (URDF README: `gap = 0.028 - 2t + 2q`)
- `get_pdz_pad_thickness()` → `0.8` / `1.4` cm
- `get_pdz_grasp_base_offset() = 14.85` cm (pad tip at 15.05 cm, grip 2 mm back)
- `get_pdz_basis_directions()` → approach `[0,0,-1]`, closing/l2r `[1,0,0]`

### 1c. The `assets/pdz_gripper_description` copy in the repo is stale AND locally hacked

- It is **`PDZ_Gripper_Slim(1)`**, not the Downloads **`Slim(3)`**. Confirmed by collision-mesh
  bounds:
  - repo `assets/pdz_gripper_description/meshes/collision/base.stl`: Y ∈ [−117.9, +34.0] mm
  - Downloads `base.stl`: Y ∈ [−82.9, +34.0] mm
  - `assets/pdz/collision/base.obj`: Y ∈ [−119, +34] mm → **generated from the Slim(1) copy.**
- The repo copy carries local Gazebo edits absent from the stock Downloads package:
  placeholder `<inertial>` blocks, `<gazebo>` pad friction `mu1/mu2 = 30`, finger joint
  `effort = 5000` (see `kuka_pdz_gripper_friction_handoff.md`). Stock Downloads has `effort = 100`
  and no inertials.
- No Python references `pdz_gripper_description` — it is only consumed by a ROS 2 workspace build.

---

## 2. Fixture generation — Franka/Panda hardcoding (confirmed)

`planning/run_fixture_gen.py` takes `gripper_type` from `grasps['gripper']` and dispatches
mesh loading/transform correctly for pdz. **The defect is in `generate_individual_pose_info()`**,
the function that decides how each part is laid into the printed fixture pocket:

```python
# planning/run_fixture_gen.py:53-54 (hold arm) and 74-75 (move arm)
gripper_l2r_dir = R.from_quat(grasp.quat[[1, 2, 3, 0]]).apply([0, -1, 0])   # Panda -l2r axis
gripper_b2f_dir = R.from_quat(grasp.quat[[1, 2, 3, 0]]).apply([0, 0,  1])   # Panda -approach axis
```

The literals `[0,-1,0]` / `[0,0,1]` are the **Panda hand-frame axes** (negated `l2r` and
negated `approach` from `get_panda_basis_directions()`). They are applied to a
**gripper-native** `grasp.quat` — `util_grasp.get_gripper_pos_quat()` builds the quat from
`get_gripper_basis_directions(gripper_type)` — with **no `gripper_type` branch here**.

| Gripper | l2r basis | `[0,-1,0]` picks | Result |
|---|---|---|---|
| panda | `[0,1,0]` | real closing axis | correct (by coincidence) |
| kuka | `[0,1,0]` | real closing axis | correct (by coincidence) |
| robotiq-140 | `[0,1,0]` | real closing axis | correct (by coincidence) |
| **pdz** | `[1,0,0]` | **transverse axis (toward D405)** | **part planted ~90° yawed** |
| robotiq-85 | `[-1,0,0]` | transverse axis | also wrong |

`[0,0,1]` (back-to-front) is fine for all of the above — panda/kuka/pdz share the `[0,0,-1]`
approach basis. (Same idiom at `run_grasp_gen.py:146` `compute_retract_grasp` is therefore
safe: it only uses the `[0,0,1]` direction.)

### Downstream effect for a PDZ run

- `pose_info[*]['extent_x' / 'extent_y']` effectively swapped → wrong 2D bin-packing
  (`run_bin_packing`, `generate_pickup_pose`) and wrong fixture footprint.
- The gripper-clearance relief milled into the mold — `MOLD_EDGE_OFFSET_GRIPPER` subtraction of
  `gripper_hull_pickup` at `run_fixture_gen.py:310-314` — is cut on the wrong side of the pocket.
- Exported `fixture/pickup.json` part poses are mis-yawed, so at real pickup the fingers can
  foul the fixture walls.
- Note: the gripper hull in `generate_pickup_meshes()` is itself built correctly (it goes through
  `transform_gripper_meshes(gripper_type, ...)`); part and gripper stay mutually consistent. The
  error is purely in how the part+gripper pair is oriented on the print bed (approach axis not
  vertical for pdz).

### Fix — APPLIED 2026-08-28

`git diff planning/run_fixture_gen.py`:

- `generate_individual_pose_info(...)` gained a `gripper_type` parameter; `run_fixture_gen()`
  passes `grasps['gripper']`.
- The two Panda literals are now derived from the gripper's own basis:
  ```python
  base_basis, l2r_basis = get_gripper_basis_directions(gripper_type)
  gripper_l2r_basis = -np.asarray(l2r_basis, dtype=float)   # was [0,-1,0]
  gripper_b2f_basis = -np.asarray(base_basis, dtype=float)  # was [0, 0, 1]
  ```
- Verified numerically: **byte-identical output for panda / kuka / robotiq-140**
  (l2r basis `[0,1,0]` → `-basis` = `[0,-1,0]`), and the l2r direction is now the true closing
  axis for **pdz / pdz-14** (`+X`) and **robotiq-85** (`-X`). `b2f` was already correct for all.

The remaining constants below are **not** touched by this fix and still need per-gripper tuning
before a real PDZ fixture print — see §2a.

### 2a. Tuning the remaining hardcoded fixture parameters for the PDZ gripper

| Constant | Location | What it controls | PDZ tuning |
|---|---|---|---|
| `MOLD_EDGE_OFFSET_GRIPPER = [0.8, 0.8, 0.4]` cm | `run_fixture_gen.py:32`, used at `:312` as `get_buffered_meshes(gripper_hull_pickup, np.array(MOLD_EDGE_OFFSET_GRIPPER) / 2)` | Clearance inflation of the gripper convex hull before it is subtracted from the mold — i.e. the gap between the descending gripper (fingers + base + D405) and the fixture-relief walls at pickup. Vertex-normal buffer, so the listed value is applied **per side** after the `/2` → net wall gap ≈ the listed number. | Set `[X, Y]` from the real KUKA+PDZ **XY pose repeatability at the fixture station** (a few mm on the rig) **+ printed-fixture / part-seating slop** (~2–3 mm). `0.8 cm` each is plausible but should be rig-measured, not inherited. Set `Z` from how far the **pad tips dip below `board_height_max`** at pickup (PDZ pads reach 150.5 mm from the flange and grip 2 mm inside the tip; the pads sit *in* the pocket) plus 1–2 mm — `0.4 cm` is likely **too small** for PDZ and should be checked against the rendered `fixture.png` / a slice at `board_height_max`. Cleanest: promote to `get_mold_edge_offset_gripper(gripper_type)` alongside the other per-gripper functions in `geometry.py`. |
| `open_ratio - 0.05` (tight) / `+ 0.15` (loose) | `run_fixture_gen.py:226-227` (hold) and `234-235` (move) | Builds the swept clearance hull between a slightly-closed and a wider-open finger pose, so the mold relief accommodates the open→close motion at pickup. Values are in **open-ratio units [0,1]**, converted to cm by each gripper's `get_*_finger_states` / `get_*_meshes_transforms`. | Panda: `Δ=0.15` → `4 cm × 0.15 = 0.6 cm` extra stroke **per finger** (1.2 cm wider span); `Δ=0.05` → 0.2 cm per finger tighter. PDZ stroke is `PDZ_JAW_TRAVEL = 3.2 cm`, so the same literals give only `0.48` / `0.16 cm` per jaw — **20 % less** physical clearance (`3.2/4 = 0.8`). **Decision 2026-08-28: left unchanged** — the ~1 mm difference is inside the mold's other tolerances and the sweep is not the binding clearance term. Guidance if it ever needs tuning: pick the clearance in mm/jaw and convert with `Δ_open_ratio = Δ_mm / (10 · stroke_cm)` (Panda's literals = 2 mm / 6 mm), and clamp `open_ratio + Δ` to `1.0` (grasp-gen allows `open_ratio` up to 0.95, so `+0.15` models the fingers ~10 % past their hard stop). |
| `DELTA_BUFFER_SIZE = 2.0` cm | `run_fixture_gen.py:39`, added to `extent_x` when a part–gripper collision is found in the packing loop (`:422`) | Extra spacing given to a part whose pickup gripper hull overlaps a neighbour. | Generic; 2 cm is a reasonable bump. Only revisit if PDZ fixtures pack too loose/tight in practice — it interacts with `PART_GAP`. |
| `MOLD_EDGE_OFFSET_PART = [0.05, 0.05, 0.0]`, `PART_BOUNDARY_OFFSET = 0.2`, `PART_GAP = 2.0` | `run_fixture_gen.py:31, 33, 34` | Part-side mold/packing offsets. | Gripper-agnostic — leave as is. |
| `board_quat` panda-vs-else | `planning/utils/render_fixture.py:34` | Render-only board orientation. | **arm**-keyed, not gripper-keyed; correct for pdz (runs on the kuka arm). No change. |

**Also feeds the mold relief (asset, not a constant):** `gripper_hull_pickup` is built from
`load_gripper_meshes('pdz'/'pdz-14', ...)` → `assets/pdz*/collision/{base,finger_*,d405}.obj`.
Those OBJs are still the **Slim 1** geometry (no heatsinks; D405 box at the old `Rx(-35°)`
pose). Until they are regenerated from Slim 3 (§1c, §5), the relief is cut for the wrong base
envelope and wrong camera position regardless of the constants above.

---

## 3. What the Downloads (Slim 3) URDF changes vs. what Fabrica assumes

| Change (Slim1 → Slim3) | New value | Effect on Fabrica |
|---|---|---|
| **`pdz_gripper_tcp` Z**: `0.1505 → 0.1355 m` (reframed "fingertips" → "pad vertical midpoint") | −1.5 cm | **Planning: none** — Fabrica ignores the URDF TCP and uses `get_pdz_grasp_base_offset() = 14.85`. Pad geometry is unchanged (both `left_pad_8mm.stl`: tip at 150.5 mm), so 14.85 stays correct. **Real-robot/MoveIt/Grasp_Planning handoffs that key off `pdz_gripper_tcp` shift 1.5 cm along approach.** |
| **D405 pose**: `Rx(-35°) → Rx(-30°)`; `camera_joint` xyz + rpy (`-55° → -60°`) changed | ≈3 cm + 5° | `assets/pdz*/collision/d405.obj` is a flange-frame box baked at the **old** pose → now wrong in grasp-gen collision and in the fixture-mold subtraction (`get_gripper_hand_names` includes `pdz_d405`). |
| **4 heatsinks added** (2 camera, 2 motor); tensioner + camera bracket revised | hull vol +4 %, base Z-max +1.8 mm | Extra collision volume on `pdz_base` **missing** from `assets/pdz/*/base.obj`. The Slim(1) base is conservatively *larger* in −Y (−119 vs −83 mm) so not strictly unsafe, but the shape is wrong. |
| Finger joint `effort` `100` (stock) vs `5000` (repo hack) | — | Gazebo dynamics only; irrelevant to planning. |
| Finger stroke `0.032`, bare gap `0.028`, pad `0.005–0.014` | unchanged | Fabrica's `PDZ_JAW_TRAVEL=3.2`, `PDZ_BARE_FINGER_GAP=2.8`, pad `0.8/1.4` still match. |
| New `COORDINATE_FRAME_AUDIT.md` | — | Confirms canonical flange convention: +X opening axis, +Y transverse (D405 on −Y), +Z out of flange — matches `get_pdz_basis_directions()`. |

---

## 4. Minor issue found

`rendering/render_traj_blender.py:101` uses `gripper_type in ('kuka', 'pdz')` (exact match) —
`pdz-14` and `pdz-mech` fall through to `raise NotImplementedError`, unlike the
`startswith('pdz')` used everywhere else.

---

## 5. Recommendations (to actually adopt the new URDF)

1. Sync `~/Downloads/pdz_gripper_description` → `assets/pdz_gripper_description`, re-applying the
   local Gazebo edits (inertials, pad friction, `effort`) — or move those into a separate overlay
   so the base package stays vendor-clean.
2. Regenerate `assets/pdz/` and `assets/pdz_14/` OBJs from Slim(3): `base` **with heatsinks**,
   `d405` box at the **new** camera transform.
3. Re-verify `planning/robot/geometry.py`: `get_pdz_grasp_base_offset()` stays `14.85` (pad tip
   unchanged); refresh the D405 box; add a note that Fabrica's grasp frame is now 1.5 cm out from
   the URDF's `pdz_gripper_tcp` (decide whether to realign for real-robot-stack consistency).
4. ~~Fix `generate_individual_pose_info()` per §2.~~ **DONE 2026-08-28** — see §2 "Fix — APPLIED".
   Still open: tune `MOLD_EDGE_OFFSET_GRIPPER` and the `open_ratio ± 0.05/0.15` deltas for PDZ
   per §2a, and regenerate `assets/pdz*` from Slim 3 (items 1–3).
5. Fix the `render_traj_blender.py:101` exact match → `startswith('pdz')`.

## Files referenced

- `planning/run_fixture_gen.py` — `generate_individual_pose_info` (42-94), `generate_pickup_meshes`
  (202-238), `generate_fixture` (241-320)
- `planning/robot/geometry.py` — PDZ block 157-224, mesh loaders 482-493
- `planning/robot/util_grasp.py:160-165` — `get_gripper_pos_quat` (quat is gripper-native)
- `planning/robot/sim_string.py:80-113` — `get_pdz_gripper_string`
- `planning/robot/util_arm.py:137-140` — pdz IK note
- `rendering/render_traj_blender.py:101`
- `assets/pdz/`, `assets/pdz_14/`, `assets/pdz_gripper_description/` (Slim 1 + Gazebo hacks)
- `~/Downloads/pdz_gripper_description/` (Slim 3, not yet imported)
- Related: `kuka_plumbers_block_fixture_pose_handoff.md`, `kuka_pdz_gripper_friction_handoff.md`,
  `docs/kuka_pdz_gripper_grasp_quickstart.md`
