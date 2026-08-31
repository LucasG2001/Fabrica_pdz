# Handoff for the coding agent — deploy the new `plumbers_block` plan to the hardware executor repo (2026-08-31)

**Target repo:** the KUKA hardware executor (`lbr_dual_arm_pdz_bringup` /
`hardware_plan_executor_node`, a shim over
`plan_executor_node.py --controller-topology combined --robot-name lbr_dual_arm`).
It reads **only `motion.pkl`** on the real rig.

**Source of truth for this change:** `Fabrica/docs/fixture_min_y_shortfix_handoff.md`
(branch `plumbers-block-shortfix-renders`). Read that first for the "why".

---

## 0. What changed on the planning side

The `plumbers_block_sim` fixture had regenerated to the **wrong side of the workcell**
(Fabrica board-frame y ≈ +20 cm, behind the KUKA arm bases). It has been moved back to the
**correct location, Fabrica y ≈ −52 cm**, which in the real dual-KUKA frame is:

- **fixture bbox centre ≈ `(0.62, 0.00, ~0.00) m`**, resting on the table
  (table surface `z = −0.025 m`; fixture is 4.5 cm tall, so bottom at −0.025, top at +0.020).
- footprint (real frame, after the board→world map): x‑span ≈ 25 cm, y‑span ≈ 20 cm,
  long axis along **world +x**, near edge ≈ x = 0.52 m, far edge ≈ x = 0.72 m.
- Derivation: Fabrica board frame → real: rotate **+90° about Z about the arm‑pair midpoint
  `(0, 10, 0) cm`**, then cm→m. See
  `Fabrica/planning/utils/generate_plumbers_block_gz_assets.py` (authoritative) and memory
  `fabrica_fixture_pose_conventions`. The Fabrica bounds are
  `x ∈ [−12.5, 12.5], y ∈ [−52, −32], z ∈ [0, 4.5] cm`; run that script (or its transform)
  to get exact world pose + per‑part poses rather than hand‑transcribing.

Deliverables produced on the Fabrica branch:
- `logs/plumbers_block_sim/fixture/fixture.obj` — new fixture mesh at y = −52 (**reprint this**).
- `logs/plumbers_block_sim/fixture/pickup.json` — new global pickup poses (Fabrica frame).
- `logs/plumbers_block_sim/motion.pkl` — **collision‑TOLERANT** plan (see §2). Not for hardware yet.
- `logs/plumbers_block_sim/render_collision_tolerant/` — mp4 / gif / montage of the plan.

---

## 1. Your tasks (hardware executor repo)

Do these now; they do not depend on the collision‑free plan landing.

1. **Update the fixture reference pose** everywhere the executor/bringup encodes it:
   - `grep -rn` the repo for the old fixture pose / any hard‑coded `0.5`‑ish or `-0.5`‑ish
     x/y fixture offset, the fixture spawn in launch/URDF/SDF, the MoveIt planning‑scene
     collision object, and any static TF (`world`→`fixture` / `table`→`fixture`).
   - Set it to the §0 pose. Prefer deriving it from one place (a single `fixture_pose`
     param) rather than repeating the numbers.
2. **Swap the fixture mesh** if the executor loads one for collision/visualisation:
   replace the old `fixture.obj`/STL with the new
   `Fabrica/logs/plumbers_block_sim/fixture/fixture.obj` (convert to your mesh format;
   scale is **cm in the Fabrica file** — check your loader's expected units, the sim uses cm).
3. **Point the executor at the new `motion.pkl`** — confirm which path/param
   `hardware_plan_executor_node` / `plan_executor_node.py` reads and that it will pick up the
   regenerated file (don't rely on a stale copy).
4. **Verify the pdz gripper command mapping.** The new plan's grasp widths are
   `open_ratio ≈ {0.44, 0.20, 0.48, 0.20, 0.44}` for parts `{3,1,0,4}` + base `2`, and it
   contains a **hold‑arm regrasp of the base** (grasp `2/938` → `2/1772`) mid‑sequence.
   Make sure the executor's `open_ratio → gripper setpoint` path handles a mid‑plan hold‑arm
   open/close (not just one grab at the start). Cross‑check against memory
   `kuka_gripper_package_landscape` (servo_gripper vs servo_gripper_julien — confirm which is
   live) and `kuka_hardware_plan_executor`.
5. **Add a safety gate:** the executor should refuse (or require an explicit
   `--allow-collision-plan` flag) to run a `motion.pkl` that is not collision‑verified. The
   Fabrica side can stamp a flag in the pickle / a sidecar file; if not, at minimum log a
   loud warning and default to dry‑run. See §2 for why.
6. **Dry‑run first:** play the new `motion.pkl` back in RViz against the updated fixture pose
   and confirm the arms reach the fixture and the assembly area with no gross clipping,
   **before** any FRI / real‑hardware execution. Compare against
   `Fabrica/logs/plumbers_block_sim/render_collision_tolerant/plan_view.mp4`.

---

## 2. Blocker — do NOT run the current `motion.pkl` on hardware

The `motion.pkl` currently on the Fabrica branch was produced with
`MOTION_PLAN_ALLOW_COLLISION=1`: one path segment (the hold‑arm regrasp of the base part 2)
could not be planned collision‑free and was replaced by a **straight joint‑space
interpolation**. The plan is fine for visualisation / RViz playback but is **not
collision‑verified**.

**Prerequisite before a hardware run (Fabrica side, not this repo):**
- Apply the "Problem 4" fix — exclude the active/target part from `plan_path_switch`'s
  collision scene in `Fabrica/planning/robot/motion_plan_arm.py` (same rationale as the
  collision‑aware pickup IK already added). This covers both the move‑arm switch and the
  hold‑arm regrasp.
- Re‑run `run_fixture_gen.py` (already at `min_fixture_y = -52`) then `run_motion_plan.py`
  **without** `MOTION_PLAN_ALLOW_COLLISION` — it must exit 0 and write a `motion.pkl` with
  no fallback lines in the log.
- Re‑run `validation.json` for the new fixture (the on‑disk one still describes the old
  y ≈ −52.7 layout and is stale).

Once that `motion.pkl` exists, re‑do §1.3 and §1.6, then proceed to hardware.

---

## 3. Known prior issues to re‑check on the real rig

From memory `kuka_pdz_plumbers_block_status_20260825` (may or may not still apply after the
fixture move):
- real run previously stalled at **RTF ≈ 0.028** with fixture‑mesh collision — watch sim/
  control rate when the new fixture mesh is loaded.
- **`screw_b` (a part) landed ~5 cm off** its target — verify part seating after the first
  successful run; may be a grasp/insertion issue independent of the fixture pose.
- pad friction / finger effort (`kuka_pdz_gripper_friction_status_20260825`) was applied but
  never load‑tested live.

---

## 4. Acceptance

- [ ] Executor fixture pose = §0 pose, derived from one param, matches
      `generate_plumbers_block_gz_assets.py` output.
- [ ] New fixture mesh loaded (correct units).
- [ ] Executor reads the regenerated `motion.pkl`; gripper mapping handles the mid‑plan
      hold‑arm regrasp.
- [ ] Safety gate rejects a non‑collision‑verified `motion.pkl` by default.
- [ ] RViz dry‑run of the collision‑free `motion.pkl` against the new fixture pose looks
      clean.
- [ ] Fixture reprinted and physically mounted at the §0 pose (holes / ArUco aligned).
