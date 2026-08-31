# Handoff — `plumbers_block_sim` fixture-Y short-term fix + collision-tolerant render (2026-08-31)

Branch: `worktree-fixture-min-y-shortfix` (worktree, **not committed to `main`**).
Predecessors: `docs/fixture_pickup_unreachable_handoff.md`,
`docs/fixture_generation_handoff.md`, memories `fabrica_fixture_pickup_unreachable`,
`fabrica_fixture_pose_conventions`.

---

## 0. TL;DR

- The on-disk `plumbers_block_sim` fixture had regenerated to Fabrica board-frame
  **y ≈ +20 cm** (behind the KUKA arm bases) → `run_motion_plan.py` aborted at step 0.
- Correct location is **y ≈ −52 cm** (real-world ≈ `(0.627, 0.0, ~−0.02) m`), matching the
  still-on-disk `validation.json` and the 2026-08-21 `motion.pkl`.
- Getting there peeled back **four stacked problems**. Three now have real (short-term)
  fixes; the fourth is worked around only for visualisation.
- A **collision-tolerant** `motion.pkl` was produced and **rendered headlessly**.
  Render saved to `logs/plumbers_block_sim/render_collision_tolerant/`.

There is still **no collision-free `motion.pkl`** for this asset. Problem 4 needs a
targeted `plan_path_switch` change (or the proper fixture-gen rework).

---

## 1. Root-cause correction to the previous handoff

`docs/fixture_pickup_unreachable_handoff.md` §4 attributes the −52 → +20 fixture swing to
commit `253a3f4` (the `generate_individual_pose_info` gripper-basis literal → basis swap,
a ~90° pdz rotation). **That is disproven.** Probes this session:

| run | code | grasps.pkl | fixture-y result |
|---|---|---|---|
| current code, `gripper='pdz'` | working tree | on-disk (2026-08-21) | y ∈ [13.3, 32.1] |
| current code, `gripper='panda'` (= pre-`253a3f4` literals) | working tree | on-disk | y ∈ [13.4, 28.9] |
| **exact `a72a768` code** (hard-coded old literals) | pre-`253a3f4` | on-disk | y ∈ [13.1, 25.4], pickup yaws identical to the committed `a72a768` `pickup.json` |

So the **old** code also produces y ≈ +20 from today's inputs. `generate_pickup_pose`, the
global-pose export block, `get_assembly_center('kuka')` and `get_fixture_min_y('kuka')` are
byte-identical in git back to `8acda0d` (2026-08-16); `get_fixture_min_y('kuka')` has only
ever been `+10`. The committed `a72a768` `pickup.json` (y ≈ −52) **cannot be reproduced by
any code in git history with the current `grasps.pkl`** (same orientations, X matches, Y
65–78 cm more negative).

**Actual cause:** the good −52 fixture (2026-08-21) was generated under **local uncommitted
edits to `workcell.py`** (a ~62–72 cm more-negative board-Y origin — `get_fixture_min_y`
and/or `get_assembly_center`) that were later discarded. The 2026-08-29 regen ran against
the committed tree (`get_fixture_min_y('kuka') = +10`) and produced the +20 layout,
overwriting `fixture.obj` / `pickup.json`; `validation.json` was **not** re-run and still
describes the −52 fixture.

Structurally: `generate_pickup_pose` sets board-y = `rect_y + h/2 − center_y + min_fixture_y`;
`center_y` (AABB-centre-y of the final-assembled mesh rotated about the world origin)
≈ `(rot_mat · [0,−15,0]).y`, which **cancels** the `(rot_mat · t_final).y` term from
`T_rel @ T_final`. Net `global_y ≈ rect_y + h/2 + min_fixture_y ∈ [min_fixture_y,
min_fixture_y + bin_y]` — the sign of the assembly centre never carries through, so
`min_fixture_y` is the only knob.

---

## 2. The four stacked problems and what was done

### Problem 1 — fixture on the wrong side of the workcell  *(FIXED, patch #1)*

`get_fixture_min_y('kuka')` returned `4*dx = +10` (verbatim from Panda). For Panda/xarm/ur5e
the arm bases are at large +y and the assembly at y = −15, so a bin packed into
`[+10, +10+bin_y]` sits in front of the arms. The **KUKA bases are at `(±42, 10, 2.5)`
yawed −90°** (facing −y), so `[+10, +32]` is on/behind them → pickup IK for the hold arm
(base part 2) and move arm (part 3) is genuinely unreachable at every regularization.

**Fix:** `get_fixture_min_y('kuka')` → **−52.0**. `min_fixture_y` is the near (min-y) edge
of both the packing bin and the carved fixture box, and enters the global pickup pose as a
pure additive y offset (verified: each value shifts every part by exactly Δ). At −52 the
footprint is x ∈ [−12.5, 12.5], y ∈ [−52, −32]; parts land y ∈ [−49, −30] — beyond the
assembly (y = −15), in front of both arms.

Value choice: `−62.7` reproduces `validation.json`'s footprint exactly but leaves the
farthest parts (~y = −60, ~69 cm from the base) at the edge of iiwa7 reach → move-arm
pickup IK still fails mid-plan. `−52` pulls the layout ~11 cm closer; **all 5 pickup IK then
solve**.

### Problem 2 — KUKA `arm_box` z-envelope too tight  *(FIXED, patch #2; independent pre-existing bug)*

`motion_plan_arm.get_fns` builds a keep-out shell from `get_hold_arm_box` / `get_move_arm_box`
whose z half-extents (`+80` up, `−0` down) are Panda-inherited. The taller iiwa7 on its
2.5 cm riser (`get_kuka_mount_block_height`, baked into `get_*_arm_pos` z) does **not** fit at
its own `rest_q`: the *buffered* arm collision meshes span z ∈ [+2.3, +82.9] cm vs the box's
[+2.5, +82.5] — ~2 mm over the floor, ~4 mm over the ceiling. `collision_fn` folds that
box-shell hit into an **"arm and ground"** collision (probe: fires even with an empty scene),
so *every* transport path that starts or ends at `rest_q` fails. The committed pipeline never
saw this because it aborts earlier at pickup IK.

**Fix:** for `arm_type == 'kuka'` only, widen the box z-band — floor → `0.0`, ceiling
`+10 cm` (`_KUKA_ARM_BOX_Z0` / `_KUKA_ARM_BOX_DZ`). x/y unchanged. Low risk: it only enlarges
the allowed workspace shell. Proper fix: re-derive `get_*_arm_box` from the real iiwa7
workspace.

### Problem 3 — pickup poses never vetted for collision-free IK  *(FIXED, patch #3)*

`get_pickup_arm_q` used plain `motion_planner.inverse_kinematics` — no collision check. With
the fixture in a reachable region, the returned wrist config can still intersect the fixture
pocket wall or a neighbouring part, which then blows up downstream in
`plan_path` / `plan_path_switch`.

**Fix:** when `collision_still_meshes` is supplied, `get_pickup_arm_q` calls
`motion_planner.inverse_kinematics_collision_free(...)` (re-solves with resampled seeds until
the config clears the fixture + the *other* still parts + the other arm). Call sites in
`run_motion_plan.py` build the scene per step:
- hold pickup (step 0): fixture + all parts at pickup pose except `part_hold`; other arm =
  move at `rest_q`.
- move pickup (step k): fixture + parts at current state (`placed_parts` at final, rest at
  pickup) except `part_move`; other arm = hold at `grasp_hold.arm_q`.

**Result (probe):** all 5 pickup IK calls succeed **first try, ~0.1 s each**, collision-free.

### Problem 4 — `plan_path_switch` collides the gripper with its own grasp target  *(NOT FIXED; worked around)*

The move-arm "switch" transports the gripper toward `pickup_q_move` at a *reduced*
`open_ratio_transport = min(open_ratio_start, open_ratio_next)` (0.30–0.58 vs
`OPEN_RATIO_REST` 0.50; tight-grasp parts 1 & 4 have `grasp.open_ratio ≈ 0.20`). The
switch's collision scene is `part_meshes_curr + fixture`, which **includes `part_move`**,
and with the 0.5 cm motion-planner buffer the open pads overlap it. `plan_path_switch`'s
retract sub-routine then re-solves IK to back out, and in the dense −52 layout that
re-solved config genuinely (unbuffered) touches a pocket wall / neighbour, tripping
`assert not collision_fn_unbuffered(q_goal_active)` (`motion_plan_arm.py:482`).

The grasp itself is fine (`pickup_q_move` vs the full scene is unbuffered collision-free,
probe-verified). The trigger is: grasp certified by `run_grasp_gen` only against final-pose
parts + a crude +y blocking box → transplanted into a rotated, tightly-packed −52 layout
with real pockets → driven at a non-grasp open ratio → flagged by a bigger buffer →
retract-IK flails.

**Proper fix (recommended, ~1 targeted change):** exclude the active part from
`plan_path_switch`'s collision scene, exactly as `get_pickup_arm_q` (patch #3) already does —
the arm is deliberately going to grasp it. Alternative: widen the mould's gripper clearance
in `run_fixture_gen` (`MOLD_EDGE_OFFSET_GRIPPER` / the swept-gripper open-ratio range in
`generate_pickup_meshes`), but that enlarges every pocket and risks merging adjacent ones.
Changing the `RETRACT_OPEN_RATIO` / `OPEN_RATIO_REST` constants does **not** fix it (it
trades the gripper-vs-part collision for a gripper-vs-pocket-wall collision, and is global).

**Workaround used for the render (patch #4):** env flag `MOTION_PLAN_ALLOW_COLLISION=1`.
A failed / asserting path segment is replaced by a straight joint-space interpolation
between `q_start` and `q_goal`. The resulting `motion.pkl` is **not collision-free** — it is
only for visualising the plan. With all three real patches applied, exactly **one** segment
(a hold-arm regrasp switch) needed the fallback; everything else planned normally.

#### The one collision-ignoring fallback — exact location (probe-verified)

- **When:** the hold-arm `switch` **after assembly step 2** (forward sequence
  `[(3,2),(1,2),(0,2),(4,2)]` — so parts 3, 1, 0 are already on the base; only part 4
  remains). The hold grasp on the **base part 2** changes: `hold grasp per step =
  [2/938, 2/938, 2/938, 2/1772]` → one regrasp, from grasp id `938` to `1772`, both on
  part 2, both `open_ratio ≈ 0.438`.
- **Bodies in contact:** the hold **pdz gripper `pdz_left_finger`** ↔ **part 2 (the base)**.
  At the grasp width (`open_ratio 0.438`) the pad touches the base (min_dist 0.000 cm,
  unbuffered collision). At the wider transport width (0.537) there is 0.32 cm real
  clearance → only a *buffered* (0.5 cm) collision there. All other parts stay clear
  (min_dist: part 0 2.64, part 3 1.31, part 4 1.67, part 1 8.1 cm).
- **Why it fails:** identical to Problem 4 but on the hold side — `plan_path_switch`'s
  collision scene includes part 2 (now at its final/assembled pose), which is exactly the
  part the gripper is regrasping. `collision_fn` cannot tell "grasping" from "colliding",
  so the retract sub-routine tries to back out of the intended grasp contact, fails, and
  `assert not collision_fn_unbuffered(q_goal_active)` fires. The same "exclude the active
  part from the switch collision scene" fix covers both the move-arm and this hold-arm case.
- **Severity:** low. Both configs grasp the same part in nearly the same place, so the
  straight-line fallback is a small wrist re-orientation of the holding arm, not a sweep
  through anything. ("One fallback" = one path *segment* that could not be planned
  collision-free; it is not a guarantee the other segments have zero sub-cm buffered grazes
  at the pickup instants.)

---

## 3. Files changed (worktree `worktree-fixture-min-y-shortfix`)

| File | Change |
|---|---|
| `planning/robot/workcell.py` | `get_fixture_min_y('kuka')` `+10` → `-52.0` (patch #1). `get_move_arm_box` / `get_hold_arm_box` widen kuka z-band via `_KUKA_ARM_BOX_Z0` / `_KUKA_ARM_BOX_DZ` (patch #2). |
| `planning/run_motion_plan.py` | `get_pickup_arm_q` gains `collision_still_meshes` / `collision_open_ratio` / other-arm args → `inverse_kinematics_collision_free` (patch #3). Both pickup call sites build the still-mesh scene (`placed_parts` set). `ALLOW_COLLISION_FALLBACK` (env `MOTION_PLAN_ALLOW_COLLISION`) + `_linear_full_path` helper wrap the three planner calls + pickup-IK None-checks (patch #4). |
| `planning/utils/render_traj_trimesh_headless.py` | **New.** Headless trimesh renderer driven by `traj.npy` (fixed camera; redmax's `render_motion_plan.py` camera constants are metre-scale for this cm-scale scene and it auto-orbits). |
| `logs/plumbers_block_sim/fixture/{fixture.obj, pickup.json, fixture.png, ...}` | Regenerated at `min_fixture_y = -52`. |
| `logs/plumbers_block_sim/render_collision_tolerant/` | **New.** Render outputs (see §5). |

`grasps.pkl` / `precedence.pkl` / `tree_opt.pkl` were **copied in** from the main checkout
(they are `.gitignore`d `*.pkl`); they are unchanged.

---

## 4. Reproduce

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate fabrica   # py3.10
cd <this worktree>
# networkx must be 2.6.3 (the env's ~/.local has 3.4.2 which shadows it):
export PYTHONNOUSERSITE=1

# 1. regenerate the fixture at min_fixture_y = -52
python planning/run_fixture_gen.py --assembly-dir assets/fabrica/plumbers_block \
    --log-dir logs/plumbers_block_sim --optimized --markers none

# 2a. real motion plan (still FAILS at problem 4 -- a move-arm switch):
python planning/run_motion_plan.py --assembly-dir assets/fabrica/plumbers_block \
    --log-dir logs/plumbers_block_sim --optimized

# 2b. collision-tolerant motion plan (writes motion.pkl; NOT collision-free):
MOTION_PLAN_ALLOW_COLLISION=1 python planning/run_motion_plan.py \
    --assembly-dir assets/fabrica/plumbers_block --log-dir logs/plumbers_block_sim --optimized

# 3. render (writes traj.npy as a side effect, then PNG frames):
python rendering/render_motion_plan.py --assembly-dir assets/fabrica/plumbers_block \
    --log-dir logs/plumbers_block_sim --record-path /tmp/discard.mp4   # for traj.npy only
python planning/utils/render_traj_trimesh_headless.py \
    --log-dir logs/plumbers_block_sim --out /tmp/frames --step 28
ffmpeg -y -framerate 20 -i /tmp/frames/%04d.png -c:v libx264 -pix_fmt yuv420p plan_view.mp4
```

Gotchas: planning task — do **not** strip miniconda from PATH. `MOTION_PLAN_ALLOW_COLLISION`
unset ⇒ original abort-on-failure behaviour. If `run_motion_plan` hangs for >5 min it is
RRT thrash on a hard segment, not the collision-aware IK (that is ~0.1 s/call).

---

## 5. Where the render is saved

`logs/plumbers_block_sim/render_collision_tolerant/` (in this worktree):

| file | notes | in git? |
|---|---|---|
| `plan_view.mp4` | ~12 s, static 3/4 view, 145 frames @ 20 fps | no (`*.mp4` git-ignored) |
| `plan_view.gif` | same, ~7 MB | no (`*.gif` git-ignored) |
| `plan_montage.png` | 9 evenly-spaced keyframes | **yes** |
| `run_motion_plan.log` | the collision-tolerant run log (1 fallback line) | no (`*.log` git-ignored) |

The mp4 + montage were also delivered into the originating conversation via file cards.
The mp4/gif are reproducible from §4; only `plan_montage.png` and the code survive a
`git clean`. **The worktree itself is deleted when its session ends** — commit/push the
branch (or copy the dir out) to keep the non-git render files.

What the render shows: both KUKA arms at rest over the H-shaped grey fixture (parts
orange = 2 base, blue = 0, green = 3, yellow = 1, red = 4 in their pockets) → hold arm
grabs the orange base → move arm brings parts 3 → 1 → 0 → 4 onto it → sub-assembly grows →
both arms retract with the finished part on the fixture.

---

## 6. Status / next steps

- [x] Fixture back on the correct (−y) side — `min_fixture_y('kuka') = -52`.
- [x] KUKA `arm_box` z-envelope widened (independent pre-existing bug).
- [x] Pickup IK made collision-aware — all 5 solve collision-free.
- [x] Collision-tolerant `motion.pkl` + headless render.
- [ ] **Problem 4**: exclude the active part from `plan_path_switch`'s collision scene →
      then a real (collision-free) `motion.pkl` should be reachable. Do this before any
      hardware run.
- [ ] Longer term (per `fixture_pickup_unreachable_handoff.md` option 1): decouple the
      board layout from the assembled-part transform in `run_fixture_gen`, add an
      IK + collision reachability gate, and re-derive `get_*_arm_box('kuka')` from the real
      workspace so patches #1 and #2 can be retired.
- [ ] Alternative unblock (option 3): restore the committed `a72a768` `fixture.obj` /
      `pickup.json` and check the 2026-08-21 `motion.pkl` against it.
- Nothing here is committed to `main`. Do not `git push` to `main`, force-push, or merge.
