# Handoff — holder↔inserter arm collision in the regenerated `plumbers_block_sim` motion plan (2026-09-01)

Branch: `worktree-holder-inserter-collision-handoff` (worktree, **not committed to `main`**).
Predecessors / related:
- `docs/fixture_generation_handoff.md` (on `main`)
- `docs/fixture_min_y_shortfix_handoff.md`, `docs/fixture_pickup_unreachable_handoff.md`
  (branch `worktree-fixture-min-y-shortfix` / `plumbers-block-shortfix-renders`, **not on `main`**)
- memories: `fabrica_fixture_pickup_unreachable`, `fabrica_fixture_pose_conventions`,
  `fabrica_kuka_ik_and_grasp_gaps`

This doc answers three questions raised after running the motion plan generated from the
new (ArUco-marker) fixture:

1. The collision seen in sim is **holder arm vs inserter arm, after the `pb_pipe` (part 0)
   insertion** — not at any pickup. Why?
2. Why did `get_fixture_min_y('kuka')` have to change at all — *"can't we just face +Y to be
   consistent with Panda?"*
3. Why was the motion plan *"changed"* from returning the inserter arm home between
   handoffs to leaving it parked until the last step?

---

## 0. TL;DR

- **The ArUco markers are not involved.** `run_fixture_gen.py` asserts `pickup.json` is
  byte-identical with/without `--markers`, and `validation.json` is byte-identical between
  the old and new runs. Markers only recess into the slab layer.
- **The collision is a motion-plan fallback artifact, not geometry.** The current
  (committed) `run_motion_plan.py` leaves the inserter ("move") arm parked at the assembly
  during the holder's regrasp `switch` on every non-final step. In the regenerated
  `−52 cm` layout, `plan_path_switch` for the holder regrasp after part 0 cannot find a
  collision-free path around that parked inserter arm, so it **asserts** (start/goal in
  collision). The `MOTION_PLAN_ALLOW_COLLISION=1` flag (added on the shortfix branch, for
  visualisation only) then swallows the assert and substitutes a **straight-line joint
  interpolation** between `q_start` and `q_goal` — that interpolation sweeps the holder arm
  through the inserter arm. See `render_collision_tolerant/run_motion_plan.log`:
  `hold switch planner raised AssertionError(); using direct interpolation`.
- **`min_fixture_y` had to change** because `4*dx = +10 cm` is a Panda-inherited constant
  that puts the fixture at/behind the KUKA arm bases. **Facing +Y is not the fix** — both
  arms already share Panda's yaw (`-π/2`, arm-type-independent code); flipping to +Y would
  move the *assembly* behind the arms and break the Grasp_Planning grasp cache.
- **The "return-to-home between handoffs" behaviour was never changed.** The committed
  scheduler has parked the inserter arm on all but the last step since the January 2026
  "Official release" (`215a30f`). The 2026-08-21 reference `commands.pkl` you are comparing
  against was produced by an older/pre-release planner run under later-discarded local
  edits; `commands.pkl` is git-ignored, so it is an untracked artifact from a different
  code state.
- **Proper fix for the collision:** exclude the active part from `plan_path_switch`'s
  collision scene (same as the shortfix's patch #3 already does for pickup IK), **and/or**
  re-introduce an inserter-arm retract before every holder `switch`, not just the last.
  There is still **no collision-free `motion.pkl`** for this asset.

---

## 1. What actually collides

Assembly order (both runs): base **2** held by the hold arm, then insert **3 → 1 → 0 → 4**.
Part **0** is the `pb_pipe`.

Walk the regenerated `logs/plumbers_block_sim_newfixture/commands.pkl` (36 commands):

| cmd | motion / task | what happens |
|----:|---|---|
| 20 | `move arm transport` pid=0 | inserter carries part 0 (`pb_pipe`) toward the assembly |
| 21 | `move arm assembly` pid=0 | part 0 seated — **inserter arm now at `q=[-0.745, 0.89, -0.109, -1.533, -0.235, 1.003, -0.67]`, right at the assembly** |
| 22 | `hold gripper open` | holder releases the base |
| 23 | **`hold arm switch`** | holder regrasps the base to reorient it for the next insertion — planned with the inserter arm **still at its cmd-21 pose** passed in as `arm_q_other` |
| 24 | `hold gripper close` | holder re-grips |

`run_motion_plan.py` → `plan_path_switch()` → internally `plan_path()` →
`assert not collision_fn(q_start_active)` / `assert not collision_fn(q_goal_active)`
(`motion_plan_arm.py`, ~`:482` / `:497-498`). `collision_fn` includes the other arm via
`arm_q_other`. The holder's regrasp goal (`grasp_hold_next.arm_q`, from the regrasp
sequence — unchanged between runs) overlaps the inserter arm parked at the assembly, so the
assert fires **with an empty message** → `AssertionError()`.

The shortfix runner catches it (only because `MOTION_PLAN_ALLOW_COLLISION=1`) and replaces
the whole segment with `_linear_full_path(q_start, q_goal)` — 40 samples of straight
joint-space interpolation, **collision checking disabled**. That is the holder arm passing
through the inserter arm that you see in `render_collision_tolerant/plan_view.mp4`.

### Why the old (2026-08-21) plan didn't show this

The 2026-08-21 `commands.pkl` has, between every insertion:
`move arm transport → rest_q` (indices 13 / 19 / 25) **before** the holder `switch`, plus
`[0,0,1]` z-up orientation constraints on the switch moves. With the inserter arm parked at
`rest_q` (out of the shared volume), the holder regrasp never comes near it. The current
committed scheduler removed both — see §5.

---

## 2. Root-cause chain

Five independent links, all required:

1. **`pickup.json` regenerated in a different Y frame.** The on-disk fixture had been
   regenerated (2026-08-29) against the committed tree, landing at board-frame
   **y ≈ +20 cm**; the shortfix then re-pinned `get_fixture_min_y('kuka')` to `−52`, so the
   current `logs/plumbers_block_sim_newfixture/fixture/pickup.json` sits at **y ≈ −43…−49**
   — a uniform **−62 cm** shift from the 2026-08-21 values (`pickup.json.disk-0829` in the
   same dir preserves the +20-frame copy). `run_motion_plan.py:155-157` consumes
   `pickup.json` verbatim — no frame transform — so every pickup, transport-start and
   handoff config is re-solved 62 cm away from the reference plan.

2. **Full re-plan.** New pickup poses ⇒ new IK everywhere, including the inter-arm `switch`
   maneuvers. `motion_plan` time dropped 280 s → 96 s (fewer waypoints, unconstrained
   switches, early fallback).

3. **Committed scheduler parks the inserter arm.** `run_motion_plan.py` only emits
   `move arm → rest_q` on the final step (`if step == len(sequence) - 1`, ~`:258`); on
   intermediate steps the inserter arm stays where its last `assembly` command left it.
   Same for the switch orientation constraint: emitted as `[None, None]` (~`:237`, `:249`),
   not z-up. (Stock since `215a30f` — see §5.)

4. **`plan_path_switch` can't route the holder around the parked inserter arm** in the
   dense `−52` layout for the part-0 → part-4 regrasp, and asserts on start/goal-in-
   collision rather than returning `None`.

5. **`MOTION_PLAN_ALLOW_COLLISION=1` straight-lines the segment.** Added on the shortfix
   branch purely so a `motion.pkl` could be rendered. It converts link 4's `AssertionError`
   into a collision-ignoring interpolation. Without the flag, `run_motion_plan.py` simply
   aborts here with `Failed to plan path for hold arm in task switch`.

The user's instinct is correct: **the collision is not a fixture property.** The fixture
change (link 1) forced the re-plan that exposed a scheduler weakness (links 3–4) that a
visualisation hack (link 5) then rendered as an arm-through-arm sweep.

---

## 3. Why `get_fixture_min_y('kuka')` had to change

### `min_fixture_y` is the only knob on fixture Y

In `run_fixture_gen.py`, `generate_pickup_pose` sets each part's board-frame y to
`rect_y + h/2 − center_y + min_fixture_y`, where `center_y` is the AABB-centre-y of the
final-assembled mesh rotated about the world origin. That `center_y` term algebraically
**cancels** the `(rot_mat · t_final).y` contribution from `T_rel @ T_final`, so the net is
`global_y ≈ rect_y + h/2 + min_fixture_y ∈ [min_fixture_y, min_fixture_y + bin_y]`. The
assembly-centre never carries through — `min_fixture_y` is the *only* thing that sets where
the fixture (and every pickup pose) lands in Y. There is no IK/collision reachability gate;
a bad value is not caught until `run_motion_plan` fails at step 0.

### `4*dx = +10` is Panda-inherited and does not fit KUKA

`get_fixture_min_y` returns `4*dx` for both `panda` and `kuka` (`6*dx` for xarm7/ur5e). With
`bin_y ≈ 20`, the fixture footprint is `y ∈ [+10, +30]`. Relevant workcell geometry
(`planning/robot/workcell.py`):

| | Panda | KUKA |
|---|---|---|
| inserter (`move`) base | `(18·dx, 8·dx, 0) = (45, 20, 0)` | `(16.8·dx, 10, 2.5) = (42, 10, 2.5)` |
| holder (`hold`) base | `(−45, 20, 0)` | `(−42, 10, 2.5)` |
| base yaw (`get_*_arm_euler`) | `−π/2` | `−π/2` (**identical, shared code**) |
| base riser | none (base z = 0) | **+2.5 cm** (`get_kuka_mount_block_height`, baked into base z) |
| assembly centre | `(0, −15, 0)` | `(0, −15, 0)` |

So for KUKA vs Panda, with the *same* `+10` fixture band:

- **Base Y is 10 cm smaller** (10 vs 20 — a deliberate 2026-08-24 change to match the real
  measured rig `KUKA_BASE_Y`, pinned by an exact grasp-flange numeric replay). Panda's base
  at +20 sits in the *middle* of `[+10, +30]` — half the fixture is in front of it. KUKA's
  base at +10 sits at the *near edge* — the **entire** fixture is at or behind the base in
  Y.
- **The base is 2.5 cm higher**, so every pickup requires reaching that much further *down*.
- Reaching down-and-behind from an elevated base folds the elbow/forearm toward the ground
  plane. Per the shortfix's controlled A/B (`run_grasp_arm_gen.py check_grasp_feasible`),
  **arm–ground collision rejections roughly triple** at the KUKA base placement. Raw
  `inverse_kinematics_above_ground()` on the same targets still succeeds — so it is **not a
  kinematic reach wall** (iiwa7 reach ≈ 800 mm); it is the *collision-checked* IK
  (`inverse_kinematics_collision_free`) exhausting its resampled seeds. That is what
  *"unreachable at every regularization"* means: sweeping the nullspace regularization
  parameter (0.1 … 1.0) changes nothing because the blocker is collision, not the IK
  objective. Compounded by Problem 2 in the shortfix doc (the KUKA `arm_box` keep-out
  z-band is itself Panda-sized and ~2–4 mm too tight at `rest_q`).

### The fix (shortfix patch #1)

`get_fixture_min_y('kuka') → −52.0`. Footprint becomes `y ∈ [−52, −32]`; parts land
`y ∈ [−49, −30]` — past the assembly (y = −15), **in front of** both −Y-facing arms,
matching the still-on-disk `validation.json` and the 2026-08-21 `motion.pkl`. `−62.7` would
reproduce `validation.json` exactly but leaves the farthest parts (~y = −60, ~69 cm from
the base) at the edge of reach → move-arm pickup IK fails mid-plan; `−52` pulls it ~11 cm
in so all 5 pickup IK solve. Explicitly short-term.

---

## 4. "Can't we just face +Y, to be consistent with Panda?"

**The premise is off:** the arms already face −Y with Panda's exact yaw. `get_move_arm_euler()`
and `get_hold_arm_euler()` are **not arm-type-dependent** — they unconditionally return
`np.array([0, 0, -np.pi/2])`. Panda, xarm7, ur5e and KUKA all mount at yaw `−π/2`. There is
nothing to "make consistent"; the yaw is already shared. What differs is base **position**
(§3), not orientation.

**Why −Y is the right facing.** The two bases sit side-by-side separated in X (±42) and
both look down the −Y axis at a workspace shared between them: the assembly at `(0, −15)`
and — once correctly placed — the fixture at `y ≈ −52`. Facing +Y aims both arms *away*
from the assembly.

**Flipping to +Y does not fix anything — it relocates the problem:**

- It would bring the *old* `+20` fixture into comfortable reach, but push the **assembly**
  (`y = −15`, ~25 cm behind bases at `y = +10`) and the correctly-located `−52` fixture out
  *behind* the arms. The insertion sequence — the contact-rich phase you are actually
  debugging — happens at the assembly. You would trade a pickup-reach problem for a
  worse insertion-reach problem.
- It breaks the **Grasp_Planning handoff**. The `--use-graspplanning` path consumes a grasp
  cache computed against `holder = (0, −0.42, 0)` / `inserter = (0, +0.42, 0)` in this base
  frame; `KUKA_BASE_Y = 10` is pinned by an exact grasp-flange-pose replay (was off by
  exactly 10 cm at the old `Y = 20`). A yaw flip invalidates every cached grasp's `arm_q`
  and the `grasp_retarget` output.
- It does not touch the actual collision in §1 (parked inserter arm during a holder
  regrasp), which is a scheduler issue, not an orientation issue.

**Correct axis to fix: the fixture location, not the arm.** `min_fixture_y = −52` puts the
fixture on the same (−Y) side as the assembly, both in front of the −Y-facing arms — which
is what the real rig does and what the 2026-08-21 reference plan assumed. The deeper
structural fix (see §6) is to decouple the board layout from the assembled-part transform
and add a reachability gate, so `get_fixture_min_y` no longer has to be hand-tuned per arm.

---

## 5. Why the "return-to-home between handoffs" was *not* changed

It wasn't. This is stock upstream Fabrica behaviour, unchanged since the initial public
release.

`git log --all -- planning/run_motion_plan.py` → only `215a30f "Official release"`
(2026-01-11), `c77c303` (a branch baseline checkpoint), and `1ff1131` (the shortfix, on a
branch). `git show 215a30f:planning/run_motion_plan.py` already has, verbatim:

```python
# hold (switch/rest)
if step < len(sequence) - 1:
    ...
    commands.append(['hold', 'arm', (grasp_hold_next.arm_q, [None, None], ...), None, 'switch'])
    ...
else:
    commands.append(['hold', 'arm', (rest_q_hold, [None, None]), None, 'transport'])

# move (rest)
commands.append(['move', 'gripper', open_ratio_retract_move, None, 'open'])
if step == len(sequence) - 1:
    commands.append(['move', 'arm', (rest_q_move, [None, None]), None, 'transport'])
```

i.e. the inserter arm returns to `rest_q` **only on the final step**; on every intermediate
step it stays where its `assembly` command left it, and the holder `switch` is planned
against it via `plan_path_switch`'s `arm_q_other`. The KUKA port (`8acda0d`,
`4395f7e`, …) added a `kuka` branch to the *position/parameter* helpers in `workcell.py`
but never touched this command schedule.

The 2026-08-21 `commands.pkl` that *does* insert `move arm → rest_q` between handoffs (and
constrains switches to z-up) was generated by an **older / pre-release research motion
planner**, run under the same later-discarded local `workcell.py` edits that produced the
good `−52` fixture (see `fixture_min_y_shortfix_handoff.md` §1). `commands.pkl` / `*.pkl`
are `.gitignore`d (`a72a768`), so that file is an untracked artifact from a code state that
no longer exists in the repo — not a baseline the current pipeline regressed from.

**So there is no "someone deleted the go-home step" regression.** The committed scheduler is
simply tighter: it works *iff* `plan_path_switch` can find a collision-free holder path
around the parked inserter arm. Pre-`−52` that held (or the pipeline aborted earlier at
pickup IK and nobody saw it); post-`−52` it fails for the part-0 regrasp.

---

## 6. What actually fixes the collision

Two complementary options; do at least the first before any hardware run.

### A. Exclude the active part from `plan_path_switch`'s collision scene *(recommended, ~1 change)*

`plan_path_switch` is called with `part_meshes = list(part_meshes_curr.values()) + [fixture_mesh]`,
which **includes the part the arm is about to grasp**, and it transports at a reduced
`open_ratio` — so the open pads clip that part under the 0.5 cm planner buffer, the retract
sub-routine re-solves IK to back out, and in the dense `−52` layout that re-solved config
trips `assert not collision_fn_unbuffered(...)`. `get_pickup_arm_q` (shortfix patch #3)
already excludes the active part for exactly this reason; mirror it in `plan_path_switch`'s
call site (and/or in `plan_path_switch` itself). This is "Problem 4" in
`fixture_min_y_shortfix_handoff.md` — still open.

### B. Retract the inserter arm before every holder `switch`, not just the last

Re-introduce `['move', 'arm', (rest_q_move, [None, None]), None, 'transport']` (or a lighter
"lift clear of the assembly" waypoint) before the holder `switch` on intermediate steps, so
the holder regrasp is never planned against an inserter arm sitting in the shared volume.
This is what the 2026-08-21 reference plan did. Costs cycle time; buys a much easier
`plan_path_switch` and removes the whole class of holder↔inserter overlaps. Consider gating
it on `arm_type == 'kuka'` to avoid perturbing the validated Panda path.

### C. Structural (per `fixture_pickup_unreachable_handoff.md` option 1)

Decouple the board layout from the assembled-part transform in `run_fixture_gen`, add an
IK + collision reachability gate on the generated pickup poses, and re-derive
`get_*_arm_box('kuka')` from the real iiwa7 workspace. Then `get_fixture_min_y('kuka')` and
the `arm_box` z-band widening (shortfix patches #1, #2) can be retired.

### Non-fixes

- Facing the arms +Y (§4).
- Changing `RETRACT_OPEN_RATIO` / `OPEN_RATIO_REST` — trades gripper-vs-part for
  gripper-vs-pocket-wall, and is global.
- Widening `MOLD_EDGE_OFFSET_GRIPPER` in `run_fixture_gen` — enlarges every pocket, risks
  merging adjacent ones.
- Leaving `MOTION_PLAN_ALLOW_COLLISION=1` on for anything but a render.

---

## 7. Evidence / where to look

| item | path |
|---|---|
| fallback log line (the smoking gun) | `logs/plumbers_block_sim_newfixture/render_collision_tolerant/run_motion_plan.log` |
| regenerated plan (36 cmds, `[None,None]` switches, no interstitial home moves) | `logs/plumbers_block_sim_newfixture/commands.pkl` |
| reference plan (39 cmds, z-up switches, home moves at 13/19/25) | `logs/plumbers_block_sim/commands.pkl` (untracked, 2026-08-21) |
| pickup Y shift (−62 cm uniform) | `logs/plumbers_block_sim_newfixture/fixture/pickup.json` vs `…/pickup.json.disk-0829` |
| fixture Y frames | `fixture.obj` bounds: reference `Y∈[10,30]`, new `Y∈[−52,−32]` |
| markers don't touch poses | `run_fixture_gen.py` `assert json.dumps(pose_pickup…) == pickup_before, 'markers mutated pickup poses'` (~`:766-770`); `validation.json` byte-identical between runs |
| committed scheduler | `git show 215a30f:planning/run_motion_plan.py` — `if step == len(sequence) - 1` |
| the assert that fires | `planning/robot/motion_plan_arm.py` `assert not collision_fn_unbuffered(q_goal_active)` (~`:482`), `assert not collision_fn_new(...)` (~`:497-498`) |
| shortfix patches (branch, not on `main`) | `git show 1ff1131` ; `docs/fixture_min_y_shortfix_handoff.md` on `worktree-fixture-min-y-shortfix` |
| workcell geometry | `planning/robot/workcell.py` `get_move_arm_pos` / `get_hold_arm_pos` / `get_move_arm_euler` / `get_assembly_center` / `get_fixture_min_y` |

---

## 8. What the 2026-08-21 reference plan actually used

The fixture side is recoverable from tracked artifacts; the arm base pose is not recorded
and is a dated inference.

### Fixture configuration — recorded

`git show a72a768:logs/plumbers_block_sim/fixture/fixture.obj` and the still-on-disk
`logs/plumbers_block_sim/validation.json` both describe the Aug-21 fixture:

- **`min_fixture_y = −62.7`** (the board `box_min.y`, `run_fixture_gen.py:320`). Footprint
  **x ∈ [−12.5, 12.5], y ∈ [−62.7, −42.7], z ∈ [0, 4.5]** (20 × 20 × 4.5 cm), centred in X,
  ~48 cm in front of the arm bases, ~28–48 cm beyond the assembly at y = −15. Current
  committed KUKA value is `+10`; the shortfix uses `−52` (pulled ~11 cm toward the arms).
- **Part pickup poses** — `git show a72a768:logs/plumbers_block_sim/fixture/pickup.json`:
  y ∈ [−58.7, −46.7], x ∈ [−2.4, 12.4]; parts 2/3/1 at **yaw ≈ π**, part 0 at ≈ π/2,
  part 4 at ≈ 1.95. Those orientations **predate commit `253a3f4`** (the ~90° pdz
  gripper-basis swap), so no current code reproduces them — the new plan's pickups are at
  yaw ≈ π/2.
- `--optimized` (`tree_opt.pkl`), order 2 (base) / 3 / 1 / 0 / 4, no `--markers` (didn't
  exist). `stats.json`: `fixture_gen 0.86 s`, `motion_plan 280.08 s`.

### Base pose — inferred, not recorded

No artifact stores it. `commands.pkl` is dated 2026-08-21, so committed `workcell.py` then
was `931de5c` (2026-08-18):

- `get_move_arm_pos('kuka') = (18·dx, 8·dx, riser) = (45, 20, 2.5)`
- `get_hold_arm_pos('kuka') = (−45, 20, 2.5)`
- both yaw `−π/2`; **±45 cm** X-separation (Panda's), **Y = 20**, on the 2.5 cm riser.

The 2026-08-24 retune (`53e1213` → `a72a768`) narrowed X-separation to **±42** (`16.8·dx`)
"to match the real rig" and later dropped **Y to 10** — current `main` is `(±42, 10, 2.5)`.
The shortfix reconstruction attributes the discarded local edits only to
`get_fixture_min_y` / `get_assembly_center` (board Y origin), not the arm bases, so the
Aug-21 plan most plausibly ran on `931de5c`'s `(±45, 20, 2.5)` bases.

### Caveat

Per `fixture_min_y_shortfix_handoff.md` §1, the Aug-21 fixture layout **cannot be
reproduced from any committed code + the current `grasps.pkl`** — it needed local
uncommitted `workcell.py` edits (a ~62–72 cm more-negative board-Y origin) that were later
discarded. `validation.json` + `a72a768:…/pickup.json` + `a72a768:…/fixture.obj` are the
surviving ground truth for the fixture; the base pose is a dated inference.

---

## 9. Experiment — push the ArUco fixture out + fix the switch schedule (2026-09-01)

Branch commit `9b09614`. Goal: a collision-free `plumbers_block` plan with the **ArUco**
fixture, KUKA bases at **y = 10** (re-verified: every grasp in `grasps.pkl` has
`grasp.arm_pos = (±42, 10, 2.5)` — hold and move, all 5 parts), fixture pushed as far from
the robots as reach allows.

### 9.1 `get_fixture_min_y('kuka')` sweep (base y=10, `--markers aruco`)

| value | fixture footprint y | outcome |
|---|---|---|
| **−60** | [−60, −40] | move-arm **pickup IK fails at step 2** (part 0 ≈ 67 cm from the base) — before any path planning. No plan. |
| **−55** | [−55, −35] | all pickup IK solves; `move part-3 transport` → `collision: True`; `move part-0 switch` **start config buffered-in-collision, RRT never escapes** → 2000 s timeout. No plan. |
| **−52** | [−52, −32] | the value the plan **builds** at. (Same as the shortfix.) |

So `min_fixture_y ≤ −55` is past the reach/collision wall for this grasp set + arm
placement; −52 is the practical maximum. "65 cm total from a base at y=10" (−55) does not
build.

### 9.2 Two schedule fixes in `run_motion_plan.py` (both required)

**A — `plan_path_switch` collision scene excludes the approached part.** The switch drives
the open, reduced-`open_ratio` gripper right onto the part it is about to grasp; keeping
that part in `plan_path_switch`'s `part_meshes` makes its own goal / retract-IK checks fail
on the *intended* contact (`assert not collision_fn_unbuffered(...)` → the hard
`AssertionError` aborts seen in §1). The `switch` command now carries the approached part
id in its `active_part` slot (was `None`) and the handler drops it from the scene —
mirroring the collision-aware pickup IK (`get_pickup_arm_q`). Neighbours + fixture + the
other arm stay in.

**B — inserter retract between handoffs (`arm_type == 'kuka'` only).** The committed
schedule retracts the move arm to `rest_q` **only on the last step**, leaving it parked at
its assembly pose. `plan_path_switch` then plans the holder regrasp against it
(holder↔inserter collision — the original report), and the next move switch starts from a
config grazing the growing sub-assembly (`start is in collision` → RRT hang). Fix: emit
`move arm → rest_q` after **every** insert, *before* the holder switch — the 2026-08-21
reference plan's schedule. kuka-gated so the validated Panda path is untouched.

### 9.3 Result at −52 + A + B

Plan **builds end-to-end** — `PYEXIT=0`, `motion.pkl` + `commands.pkl` written, **no hard
aborts**, and the `move part-0 switch` that hung indefinitely with either fix alone now
plans clean.

**Not fully collision-free.** 8 of ~24 arm segments still flag `collision: True` — the path
comes within the 0.5 cm motion-planner safety buffer at some waypoint (buffered checker;
not necessarily geometric interpenetration, and the planner returns the path without
retry):

| # | segment | note |
|---|---|---|
| 1 | `hold` → base-part pickup approach | tight against the fixture pocket wall |
| 2 | `hold` base pickup → assembly (carrying base) | " |
| 3 | `move part-3 switch` (approach) | part 3 = farthest part, arm fully extended |
| 4 | `move part-3` pickup → assembly (carrying part 3) | " |
| 5 | `move → rest` retract after part 1 (new, fix B) | backing out past the sub-assembly |
| 6 | **`hold` base-regrasp switch after part 0** | the §1 segment — no longer a hard abort (fix A), still grazes |
| 7 | `move → rest` retract after part 4 (new, fix B) | " |
| 8 | `hold → rest` final retract | " |

Parts 1, 0, 4 handoffs are all clean with margin. The residual tight segments cluster on
the base part (inherently near the fixture wall), part 3 (reach-limited), and the
holder regrasp + retracts.

### 9.4 Levers not yet tried (to close the last 8)

- Widen `MOLD_EDGE_OFFSET_GRIPPER` (or the swept-gripper open-ratio range) in
  `run_fixture_gen.py` → more pocket clearance for segments 1–4.
- Raise `RETRACT_DELTA_FAR` / the retract deltas so the retract segments clear the
  sub-assembly by more than the buffer.
- Regenerate `grasps.pkl` with clearance-aware candidates for the base and part 3.
- Relax the 0.5 cm motion-planner buffer only where it is provably over-conservative.
- A real synchronised dual-arm **unbuffered** validator on `motion.pkl` to confirm which of
  the 8 (if any) are true interpenetrations vs. sub-buffer grazes — the repo has no such
  validator today (`debug_movehold_collision_headless.py` checks `grasps.pkl` candidates,
  not `motion.pkl`).

---

## 10. Tuning pass — 8 grazes → 1 (2026-09-01, commit `5f83700`)

Applied all of §9.4 plus a fixture reposition. `logs/plumbers_block_y10_min475`, base y=10,
`--markers aruco`.

| Change | File | From → To |
|---|---|---|
| Pocket gripper relief | `run_fixture_gen.py` | `MOLD_EDGE_OFFSET_GRIPPER [1.2,1.2,0.9]` → `[1.6,1.6,1.1]` |
| Swept-gripper open-ratio | `run_fixture_gen.py` | loose delta `+0.15` → `+0.25` (hold + move) |
| Fixture position | `workcell.py` | `get_fixture_min_y('kuka') -52` → **`-47.5`** |
| Retract back-off | `config.py` | `RETRACT_DELTA_FAR 5.0` → `9.0` |
| Planner retry-on-graze | `motion_plan_arm.py` | `plan_path_with_grasp`: up to 3 attempts (RRT 1000/2500/4000, smooth 120/180/240 s), keep first collision-free |

`-47.5` also lands the slab screw-hole Y lattice (`min_fixture_y + {2.5,7.5,12.5,17.5}`,
20 cm footprint) on a clean 5 cm grid — **side holes at y = -45 / -30** — and pulls part 3
(the farthest pickup) from ~62 cm to ~55 cm from the base.

**Result: `PYEXIT=0`, `motion.pkl` written, ~4.5 min. 7 of the 8 grazes gone.**

| segment | before | after |
|---|---|---|
| `hold` → base-pickup approach | graze | **graze — `in_collision:True` across all 3 retries** |
| `hold` base pickup → assembly | graze | clean (retry attempt 2) |
| `move part-3 switch` | graze | clean (first try — wider pockets + closer fixture) |
| `move part-3` pickup → assembly | graze | clean (first try) |
| `move → rest` after part 1 | graze | clean (retry attempt 2) |
| `hold` base-regrasp switch (§1 segment) | graze / hard-abort | **clean (retry attempt 2)** |
| `move → rest` after part 4 | graze | clean (retry attempt 2) |
| `hold → rest` final retract | graze | clean (first try) |

Also cleared (were `collision: True` at `-52`): `move part-1 transport`, `move part-0 switch`
+ transport. The retry loop did the heavy lifting — 6 segments succeeded on attempt 2.

### The one remaining graze

`hold arm transport`, `rest_q → pickup_q_hold` (holder's first move, to grasp the base part
in the fixture). `in_collision: True` on all 3 fresh RRT trees → the corridor is structurally
within 0.5 cm of something. **The endpoints are clean** — `pickup_q_hold` itself is
buffer-clear (the transport *out* of the same pocket is clean), so it is a mid-path graze,
most likely the descent past the fixture rim or the move arm parked at `rest_q`. Options:
- Insert an explicit intermediate waypoint (approach the pocket from directly above).
- Widen just the base-part pocket further (it is the largest part).
- Run the unbuffered `motion.pkl` validator (still TODO) to confirm it is a sub-5 mm graze
  that is acceptable for sim / soft-contact hardware rather than a real overlap.

---

## 11. Status / next steps

- [x] Confirmed the sim collision is holder↔inserter during the part-0 (`pb_pipe`) holder
      regrasp `switch`, produced by the `MOTION_PLAN_ALLOW_COLLISION` straight-line
      fallback — not the ArUco fixture, not a pickup collision.
- [x] Confirmed `min_fixture_y` change is a real requirement (Panda-inherited `+10` puts
      the fixture at/behind the KUKA bases), and that "face +Y" is a non-fix.
- [x] Confirmed the "parked inserter arm between handoffs" schedule is stock since
      `215a30f`, not a regression.
- [ ] Apply fix **A** (exclude active part from `plan_path_switch` scene). Re-run
      `run_motion_plan.py` **without** `MOTION_PLAN_ALLOW_COLLISION` and confirm a
      collision-free `motion.pkl`.
- [ ] If A alone is insufficient, add fix **B** (inserter retract before intermediate
      holder switches), `kuka`-gated.
- [ ] Longer term: fix **C** (decouple board layout + reachability gate), then retire the
      `min_fixture_y` / `arm_box` shortfix patches.
- [ ] None of the shortfix work is on `main`. Do not push to `main`, force-push, or merge.
