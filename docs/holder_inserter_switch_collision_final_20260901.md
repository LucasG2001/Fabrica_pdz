# plumbers_block ArUco-fixture motion plan — final state (2026-09-01)

Branch `worktree-holder-inserter-collision-handoff` (**not on `main`**), tip `083abca` + this
doc. Full investigation trail: `docs/holder_inserter_switch_collision_handoff.md` (§0–§10).
Predecessor: `docs/fixture_min_y_shortfix_handoff.md` (branch `worktree-fixture-min-y-shortfix`).

---

## 0. TL;DR

Starting point: the plan regenerated from the new ArUco fixture showed a holder↔inserter
arm collision after the `pb_pipe` (part 0) insertion, and `run_motion_plan` either hard-aborted
or hung, depending on `min_fixture_y`.

End state: **the plan builds end-to-end, collision-free except one segment**, which is a
gripper-vs-its-own-grasp-target proximity flag on the holder's approach to the base part —
not a real inter-body collision.

- Plan: `logs/plumbers_block_y10_min475/{motion.pkl, commands.pkl}` (base y=10,
  `--markers aruco`, `min_fixture_y = -47.5`). Planning time ~4.5 min.
- Residual: 1 of ~24 arm segments flags `collision: True`. Identified as the holder's
  **open gripper within 0.5 cm of part 2 (the base) over the last ~19 % of its first
  approach transport** — i.e. the jaws closing in around the part they are about to grasp.
  Fixture / other arm / ground / other parts: all clear. Not fixed by choice (it is the
  expected geometry of a pickup; see §4).

---

## 1. What the original collision was

The reported holder↔inserter collision was **a motion-plan fallback artifact, not fixture
geometry**:

1. The ArUco markers never touch pickup poses (`run_fixture_gen.py` asserts it;
   `validation.json` byte-identical with/without `--markers`).
2. `get_fixture_min_y('kuka')` was `+10` (Panda-inherited) → fixture at/behind the KUKA
   bases → the shortfix moved it to `-52`, which shifted every pickup pose ~62 cm in Y and
   forced a full re-plan.
3. The committed "Official release" schedule parks the **inserter arm at the assembly
   between handoffs**; `plan_path_switch` then plans the holder regrasp against it. No
   collision-free path exists there in the dense layout → `AssertionError`.
4. `MOTION_PLAN_ALLOW_COLLISION=1` (a visualisation flag added on the shortfix branch)
   swallowed the assert and substituted a straight-line joint interpolation → the holder
   arm sweeping through the parked inserter arm in the render.

Details + evidence: main handoff §1–§2.

---

## 2. Everything that changed (vs `origin/main`)

### 2a. From the shortfix branch (`1ff1131`, earlier session), merged in as prerequisites
| File | Change |
|---|---|
| `planning/robot/workcell.py` | `get_fixture_min_y('kuka')` `+10` → `-52` (then `-47.5`, §2c); KUKA `arm_box` z-band widened (`_KUKA_ARM_BOX_Z0=0`, `_KUKA_ARM_BOX_DZ=+10`). |
| `planning/run_motion_plan.py` | collision-aware pickup IK (`inverse_kinematics_collision_free`); `MOTION_PLAN_ALLOW_COLLISION` env flag + straight-line fallback. |
| `planning/utils/render_traj_trimesh_headless.py` | new headless renderer. |

### 2b. This session — schedule fixes in `planning/run_motion_plan.py` (commit `9b09614`)
- **A. Exclude the approached part from `plan_path_switch`'s collision scene.** The `switch`
  command now carries the approached part id in its `active_part` slot; the handler drops it
  from `part_meshes`. Kills the hard "goal is in collision" aborts on the move switches.
- **B. Retract the inserter arm to `rest_q` after every insert (kuka-gated), before the
  holder switch.** Was: last step only. Kills the "start is in collision" RRT hang on
  `move part-0 switch` and the holder↔inserter collision on the regrasp.

### 2c. This session — tuning pass (commit `5f83700`)
| Change | File | From → To |
|---|---|---|
| Pocket gripper relief | `run_fixture_gen.py` | `MOLD_EDGE_OFFSET_GRIPPER [1.2,1.2,0.9]` → `[1.6,1.6,1.1]` |
| Swept-gripper open-ratio | `run_fixture_gen.py` | loose delta `+0.15` → `+0.25` (hold + move) |
| Fixture position | `workcell.py` | `get_fixture_min_y('kuka') -52` → **`-47.5`** |
| Retract back-off | `config.py` | `RETRACT_DELTA_FAR 5.0` → `9.0` (`RETRACT_DELTA_NEAR` unchanged) |
| Planner retry-on-graze | `motion_plan_arm.py` | `plan_path_with_grasp`: up to 3 attempts (RRT 1000/2500/4000, smooth 120/180/240 s), keeps the first collision-free path; attempt 0 keeps the original budget |

### 2d. Vendored (your uncommitted main-checkout WIP, copied in for reproducibility)
`planning/run_fixture_gen.py` (the `--markers aruco` feature), `planning/utils/fixture_markers.py`,
`assets/aruco/dict_4x4_50.json`.

### Untouched by this session
`motion_plan_arm.py` planner core (only the retry loop added), the grasp/sequence pipeline,
arm base positions/orientations (`get_*_arm_pos`, `get_*_arm_euler`), `main`.

---

## 3. Fixture at `min_fixture_y = -47.5`

- Footprint x ∈ [-12.5, 12.5], y ∈ [-47.5, -27.5], z ∈ [0, 4.5] cm.
- **Slab screw holes on a clean 5 cm Y grid** (`_slab_hole_lattice` = `min_fixture_y + {2.5,
  7.5, 12.5, 17.5}` for a 20 cm footprint): 8 countersunk M-holes, the ±X side holes at
  **y = -45 and y = -30**, mid holes at y = -40 / -35.
- 6 ArUco markers (DICT_4X4_50, 2.32 cm), ids 0–5.
- Part 3 (farthest pickup) now ~55 cm from the base (was ~62 at `-52`).
- Byte-identical pickup poses to `--markers none`.

`min_fixture_y` regime map (base y=10, all with A+B applied):
| value | outcome |
|---|---|
| -60 | move-arm pickup IK fails at step 2 — no plan |
| -55 | pickup IK solves; `move part-0 switch` start-in-collision, RRT hangs |
| -52 | builds; 8 buffer-margin grazes |
| **-47.5** | builds; **1 buffer-margin graze** (this doc) |

---

## 4. Final result — `logs/plumbers_block_y10_min475`

`PYEXIT=0`, `motion.pkl` (345 KB) + `commands.pkl` written, ~4.5 min.

**7 of the 8 grazes from the `-52` run are gone.** Per segment:

| segment | at -52 | at -47.5 + tuning |
|---|---|---|
| `hold` → base-pickup approach | graze | **graze (still)** — see below |
| `hold` base pickup → assembly | graze | clean (retry attempt 2) |
| `move part-3 switch` | graze | clean (1st try) |
| `move part-3` pickup → assembly | graze | clean (1st try) |
| `move → rest` after part 1 | graze | clean (retry attempt 2) |
| `hold` base-regrasp switch (the §1 segment) | graze / abort | **clean (retry attempt 2)** |
| `move → rest` after part 4 | graze | clean (retry attempt 2) |
| `hold → rest` final retract | graze | clean (1st try) |

Also clean (were `collision: True` at `-52`): `move part-1 transport`, `move part-0 switch`,
`move part-0 transport`. The retry loop cleared 6 segments on attempt 2; the part-3 pair
cleared first try from the wider pockets + closer fixture.

### The one residual graze — identified

Replaying the `hold arm transport` path from `motion.pkl` through `collision_fn` (verbose),
scene split mesh-by-mesh:

- **43 / 220 waypoints**, all in the **last ~19 %** of the path (path-fraction 0.81 → 1.0).
- Category `gripper and part`; isolating the scene: **only part 2 (the base)**. Fixture 0,
  other 4 parts 0, other arm 0, ground 0, self 0.
- The `gripper and part` check uses the **unbuffered gripper** vs the **0.5 cm-buffered
  part**. The holder gripper is at `open_ratio = 0.5375` (retract-open, wider than the 0.4375
  grasp). So: the open jaws pass within 5 mm of the base part as they descend around it.

This is the **expected geometry of a pickup** — the open gripper is meant to close around
the part — and it is the same class as fix A (exclude the reached-for part from the scene),
which was applied to `switch` tasks only. The holder's first move to the base pickup is a
`transport`, so it was not covered.

**One-line fix if wanted** (deferred): give the step-0 `hold arm transport` command the
approached part in `active_part` and drop it from `part_meshes` for a plain reach — a new
`'approach'` task or a branch in the transport handler. Not applied.

### Still not built
A synchronised dual-arm **unbuffered** `motion.pkl` validator (the repo has none;
`debug_movehold_collision_headless.py` checks `grasps.pkl` candidates only). Everything above
uses the planner's own buffered `collision_fn`; the residual is a within-0.5 cm proximity,
not a confirmed interpenetration.

---

## 5. Reproduce

```bash
# env: py3.10 fabrica, networkx must resolve to 2.6.3
export PYTHONNOUSERSITE=1
PY=~/miniconda3/envs/fabrica/bin/python
cd <this worktree>

D=logs/plumbers_block_y10_min475
mkdir -p $D
cp <main-checkout>/logs/plumbers_block_sim/{precedence,grasps,tree_opt,contact}.pkl $D/
cp <main-checkout>/logs/plumbers_block_sim/stats.json $D/

# 1. fixture (ArUco), min_fixture_y = -47.5 comes from workcell.py
$PY planning/run_fixture_gen.py --assembly-dir assets/fabrica/plumbers_block \
    --log-dir $D --optimized --markers aruco

# 2. motion plan (strict -- do NOT set MOTION_PLAN_ALLOW_COLLISION)
$PY planning/run_motion_plan.py --assembly-dir assets/fabrica/plumbers_block \
    --log-dir $D --optimized --verbose
```

Expected: `PYEXIT=0`; one `[plan_path_with_grasp] ... attempts: 3, in_collision: True` on the
first `hold arm transport`; every other segment `in_collision: False`.

---

## 6. Render

See `logs/plumbers_block_y10_min475/render/` (produced 2026-09-01):

```bash
# traj.npy (redmax replay; its own mp4 auto-orbits and is metre-scaled -- ignore it)
$PY rendering/render_motion_plan.py --assembly-dir assets/fabrica/plumbers_block \
    --log-dir $D --record-path $D/render/redmax_orbit.mp4

# fixed-camera headless frames + stitch
$PY planning/utils/render_traj_trimesh_headless.py --log-dir $D --out $D/render/frames --step 20
ffmpeg -y -framerate 20 -i $D/render/frames/%04d.png -c:v libx264 -pix_fmt yuv420p $D/render/plan_view.mp4
```

Render files (`*.mp4` / `*.png` under `logs/` are git-ignored — commit/copy out to keep them):
| file | notes |
|---|---|
| `render/plan_view.mp4` | fixed 3/4 view, the deliverable |
| `render/frames/*.png` | source frames |
| `render/redmax_orbit.mp4` | redmax replay, auto-orbits (reference only) |

---

## 7. Status

- [x] Original holder↔inserter collision root-caused (schedule + fallback, not fixture).
- [x] Fixes A (switch scene) + B (inserter retract) — the holder regrasp switch is clean.
- [x] Tuning pass: 8 grazes → 1.
- [x] Residual graze identified: holder open gripper vs base part, final approach — expected
      pickup geometry, deferred.
- [x] Plan rendered.
- [ ] Optional: extend fix A to the step-0 hold approach transport → 0 grazes.
- [ ] Optional: build the unbuffered `motion.pkl` validator.
- [ ] Nothing is on `main`. Do not push to `main`, force-push, or merge.
