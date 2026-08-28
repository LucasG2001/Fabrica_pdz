# Fixture generation — parameters & paths handoff (2026-08-28)

Reference for `planning/run_fixture_gen.py`: what it reads, what it writes, and every
tunable that shapes the printed pickup fixture (the "mold"). All lengths are in
**centimetres** (Fabrica's planning convention).

---

## 1. What it does

Given a solved assembly (`precedence` graph + `grasps` + assembly `tree`), it:

1. Replays the assembly sequence and, for each part, computes a **pickup orientation**
   (grasp-aligned: finger-closing axis → fixture +X, approach axis → −Z)
   — `generate_individual_pose_info` (`run_fixture_gen.py:42`).
2. **2D bin-packs** the part footprints onto a board — `generate_pickup_pose` /
   `run_bin_packing` (`:141`, `:156`).
3. Sweeps the gripper open→close hull at each pickup pose and, if it collides with a
   packed neighbour, **buffers that part and repacks** — `check_part_gripper_collision`
   (`:335`), loop at `:417`.
4. **Carves the mold**: subtracts the swept part volume and the swept gripper hull from a
   board box, clips to the part area, adds a solid bottom slab — `generate_fixture` (`:253`).
5. Adds 4 **countersunk mounting pads** — `add_countersunk_pads_to_fixture` (`:348`).
6. Exports mesh + per-part global pickup poses + a preview PNG.

**The assembly plan does not depend on any fixture parameter.** Sequence and grasps come
straight from `precedence.pkl` / `tree(_opt).pkl` / `grasps.pkl`; `--optimized` just
replays the stored tree (no seed sampling). Changing `PART_GAP`, `DELTA_BUFFER_SIZE`,
`MOLD_EDGE_OFFSET_*`, bin sizes, etc. only changes the board geometry and the per-part
**pickup layout poses** (`fixture/pickup.json`), never the order or the grasps.

---

## 2. How to run

```bash
# conda env 'fabrica' (Python 3.10). Do NOT strip miniconda from PATH here — this is a
# planning task, not a ROS one.
source ~/miniconda3/etc/profile.d/conda.sh && conda activate fabrica

python planning/run_fixture_gen.py \
  --assembly-dir assets/<ASSEMBLY_DIR>/<ASSEMBLY> \
  --log-dir      logs/<EXP_NAME>/<ASSEMBLY> \
  --optimized                 # use tree_opt.pkl (SequenceOptimizer); omit to sample tree.pkl
  # --seed 0                   # only used without --optimized
  # --render                   # pop up trimesh viewer + matplotlib packing plot (needs display)
```

Also invoked as the 5th stage of `planning/run_planning.sh` (`EXP_NAME ASSEMBLY [SETUP] [ASSEMBLY_DIR]`).

CLI args — `run_fixture_gen.py:473-482`:

| Arg | Required | Default | Effect |
|---|---|---|---|
| `--assembly-dir` | yes | — | Where per-part `<id>.obj` + `final` transforms live. |
| `--log-dir` | yes | — | Reads inputs from here, writes `fixture/` here. |
| `--optimized` | no | `False` | `True`: `tree_opt.pkl` via `SequenceOptimizer`. `False`: `tree.pkl` via `SequencePlanner.sample_sequence(seed=…)`. |
| `--seed` | no | `0` | Sequence sampling seed (ignored with `--optimized`). |
| `--render` | no | `False` | Interactive viewers on; otherwise pyglet runs headless. |

---

## 3. Input files (read from `--log-dir`)

| File | Needed when | Contents |
|---|---|---|
| `precedence.pkl` | always | `networkx.DiGraph`, disassembly precedence + `contact_points` per edge. |
| `grasps.pkl` | always | dict; `['arm']`, `['gripper']`, per-part grasp candidates. |
| `tree_opt.pkl` | `--optimized` | `networkx.DiGraph`, optimized assembly tree (subset-chain nodes, `move/hold_part` + grasp ids on edges). |
| `tree.pkl` | no `--optimized` | assembly search tree for sampling. |
| `stats.json` | always | updated in place with a `fixture_gen` timing entry. |
| `assets/<…>/<part>.obj`, `pos_quat` (`transform='final'`) | always | final assembled part meshes/poses. |

### ⚠ networkx pickle-version gotcha

`logs/plumbers_block_sim/{precedence,tree_opt}.pkl` were pickled with **networkx ≥ 3.2**
(they carry `__networkx_cache__` + cached view objects). The repo pins
`networkx==2.6.3` (`requirements.txt`, `environment.yml`), under which they fail to load:

```
AttributeError: 'DiGraph' object has no attribute '_adj'
```

They were converted in place to clean 2.6.3 pickles on 2026-08-28 (data-identical: 5
nodes / 4 edges, `contact_points` arrays and `move/hold/grasp-id` edge attrs preserved).
Originals kept at `logs/plumbers_block_sim/_nx3_backup/`. If a fresh copy of an
nx-3.x-pickled graph shows up again, re-run the same neutralise-views → rebuild-DiGraph →
re-dump conversion. Graphs from the other log dirs (`plumbers_block_run`, `franka3/…`)
load fine as-is.

---

## 4. Output files (written to `<log-dir>/fixture/`)

| Path | Written at | Contents |
|---|---|---|
| `<log-dir>/fixture/fixture.obj` | `run_fixture_gen.py:461` | Watertight mold mesh: board + carved part/gripper reliefs + solid bottom slab + 4 countersunk pads. Origin at world (0,0,0); +Z up; board spans `y ∈ [min_fixture_y, …]`. |
| `<log-dir>/fixture/pickup.json` | `:459` | `{part_id: [x, y, z, roll, pitch, yaw]}` — **global** pickup pose of each part on the fixture (relative-to-final pose composed with the part's final pose, then shifted by `part_translation`). Consumed downstream by motion planning / pose-handoff utils. |
| `<log-dir>/fixture/fixture.png` | `:462` | Preview render of `fixture.obj` **with parts seated in it** (trimesh scene, headless). Not fixture-only. |
| `<log-dir>/stats.json` (`fixture_gen` key) | `:469` | `{"fixture_gen": {"time": <seconds>}}` merged into existing stats. |

`fixture/` is created with `exist_ok=True` (`:458`) and files are overwritten each run.
Large artifacts under `logs/` are git-ignored (see commit `a72a768`), so a regenerated `fixture.obj` and the
converted graph pickles won't show in `git status`; `pickup.json`, `fixture.png`,
`stats.json` are small and *do* show.

### Viewing the fixture alone

`fixture.png` always composites the parts in. For a fixture-only image:

```python
import pyglet; pyglet.options["headless"] = True
import trimesh
s = trimesh.Scene([trimesh.load("logs/<EXP>/<ASM>/fixture/fixture.obj", force="mesh")])
open("fixture_only.png", "wb").write(s.save_image(resolution=(1500, 1150), visible=False))
```

(Setting `pyglet.options["headless"]` *before* other imports is what makes the offscreen
render non-black in this env.) Or open `fixture.obj` in MeshLab / Blender / `f3d`.

---

## 5. Tunable parameters — `run_fixture_gen.py:26-39`

Current values reflect the 2026-08-28 PDZ tuning pass.

| Constant | Current | Units | Used at | Controls | Notes / effect of increasing |
|---|---|---|---|---|---|
| `DX` | `2.5` | cm | everywhere | Board grid unit (`get_board_dx()`, `workcell.py:4`). Bin sizes, pad placement, box rounding are all multiples of it. | Global scale knob; don't change casually — also affects arm/workcell placement. |
| `BOTTOM_THICKNESS` | `0.5` | cm | `:202`, `:263`, `:290`, `:303` | Solid slab under the mold; also the z-datum the part is dropped onto in the pocket. | Thicker floor, part sits higher. |
| `EDGE_THICKNESS` | `3.0` | cm | — | **Defined but unused** in current code. | No effect. |
| `MIN_MOLD_DEPTH` | `1.0` | cm | `:263` | Starting mold depth searched upward until each part's COM is over solid pocket wall. | Deeper minimum pocket. |
| `MOLD_EDGE_OFFSET_PART` | `[0.05, 0.05, 0.0]` | cm | `:309` | Per-side clearance inflation of the **part** swept hull before it's subtracted from the board (vertex-normal buffer, value is halved then applied per side → net ≈ listed). | Looser part pocket. Gripper-agnostic — leave as is. |
| `MOLD_EDGE_OFFSET_GRIPPER` | `[1.2, 1.2, 0.9]` | cm | `:324` | Per-side clearance inflation of the **gripper** convex hull before it's subtracted → the gap between the descending gripper (fingers + base + camera) and the relief walls at pickup. Halved then per-side → net wall gap ≈ listed number. | Wider gripper relief. `[X,Y]` should come from real KUKA+PDZ XY pose repeatability at the fixture station + fixture/part seating slop; `Z` from how far the PDZ pads dip below `board_height_max` at pickup. Was `[0.8, 0.8, 0.4]` (Panda-inherited); bumped for PDZ. |
| `PART_BOUNDARY_OFFSET` | `0.2` | cm | `:315-316` | Outward margin of the per-part keep-box that clips the carved board back to "just the part area". | Keeps slightly more board around each pocket. Gripper-agnostic. |
| `PART_GAP` | `2.5` | cm | `:145`, `:173` | Added to each part footprint (X and Y) before 2D bin-packing → min spacing between packed parts. | Parts spaced further apart; larger board; may push single→double bin. Was `2.0`. |
| `MAX_BIN_SIZE_SINGLE` | `[8*DX, 10*DX]` = `[20, 25]` | cm | `:162-164` | Preferred board envelope (one print). | — |
| `MAX_BIN_SIZE_DOUBLE` | `[8*DX, 20*DX]` = `[20, 50]` | cm | `:158`, `:166`, `:351` | Fallback board envelope (two prints) + caps pad Y span. | If packing exceeds this the run aborts with "Bin size exceeds maximum size". |
| `MAX_BIN_SIZE_BLOCKING` | `[12*DX, 20*DX]` = `[30, 50]` | cm | — | **Defined but unused** in current code. | No effect. |
| `DELTA_BIN_SIZE` | `1*DX` = `2.5` | cm | `:172-178` | Step size of the bin-size search that minimises board area. | Coarser/finer area optimisation. |
| `DELTA_BUFFER_SIZE` | `2.5` | cm | `:434` | Amount added to a part's `extent_x` each time its pickup gripper hull collides with a packed neighbour, before repacking. | Faster convergence of the buffer loop but looser packing. Interacts with `PART_GAP`. Was `2.0`. |

### Swept-gripper-hull open-ratio deltas — `run_fixture_gen.py:238-239, 246-247`

`grasp.open_ratio - 0.05` (tight) and `+ 0.15` (loose) build the finger-motion clearance
hull carved into the mold. Units are open-ratio [0,1]; each gripper converts to cm by its
own stroke. **Decision 2026-08-28: left unchanged for PDZ** — with PDZ's 3.2 cm jaw
travel the literals give ~20 % less physical clearance than on Panda, but that ~1 mm is
inside the mold's other tolerances and the sweep isn't the binding term. If ever tuned:
pick clearance in mm/jaw and convert `Δ = Δ_mm / (10 · stroke_cm)`; clamp `open_ratio + Δ`
to `1.0`.

---

## 6. Derived / workcell parameters (not in `run_fixture_gen.py`)

| Value | Source | For `kuka` | Meaning |
|---|---|---|---|
| `get_board_dx()` | `planning/robot/workcell.py:4` | `2.5` cm | `DX`. |
| `get_assembly_center(arm)` | `workcell.py:140` | `[0, -6*DX, 0]` = `[0, -15, 0]` | Added to final part meshes/poses before layout. |
| `get_fixture_min_y(arm)` | `workcell.py:154` | `4*DX` = `10` cm | `min_fixture_y`: the board's near (−Y) edge; parts are laid at `y ≥ min_fixture_y`. |
| `get_kuka_mount_block_height()` | `workcell.py` | `2.5` cm | KUKA base riser; not used by fixture-gen directly but part of the same cm world. |
| `board_height_max` | computed, `generate_fixture` `:256-272` | per-assembly | Top of the mold: raised (in 1 cm steps from `BOTTOM_THICKNESS + MIN_MOLD_DEPTH`) until every part's COM projects onto solid pocket wall. Everything above it is not carved. |
| `part_translation` | computed, `:292-296` | per-assembly | Recentres parts over the final compact board box; also applied to `pickup.json` poses and returned. |

### Countersunk mounting pads — `planning/utils/fixture_countersunk.py:4-8`

| Constant | Value (cm) | Meaning |
|---|---|---|
| `COUNTERSUNK_DIAMETER` | `1.05` | Countersink cone top diameter. |
| `HOLE_DIAMETER` | `0.6` | Through-hole diameter. |
| `PAD_DIAMETER` | `2.5` | Pad disc diameter. |
| `PAD_HEIGHT` | `0.5` | Pad disc thickness. |

Four pads are unioned onto the mold at the board corners
(`x = ±(bin_x/2 + DX/2)`, `y` clamped to `min_fixture_y + DX/2 … min_fixture_y +
MAX_BIN_SIZE_DOUBLE[1]//2 + DX/2`), `add_countersunk_pads_to_fixture` (`:348`).

---

## 7. Downstream dependency

`fixture/pickup.json` is consumed by `planning/run_motion_plan.py`,
`planning/utils/fixture_pose_to_graspplanning.py`, and `planning/utils/render_fixture.py`.
Regenerating the fixture changes the pickup layout, so any existing `motion.pkl` /
`commands.pkl` / `traj.npy` in the same log dir are stale w.r.t. the new layout — re-run
motion planning if the pickup approach must match.

Fixture-frame conventions (native vs graspplanning vs gazebo, and the KUKA 2.5 cm table
offset) are covered separately in the `fabrica_fixture_pose_conventions` memory.

---

## 8. Last regeneration (2026-08-28)

- Target: `logs/plumbers_block_sim` (`--optimized`), assembly `assets/fabrica/plumbers_block`, `kuka` / `pdz`.
- Params bumped: `PART_GAP` 2.0→2.5, `DELTA_BUFFER_SIZE` 2.0→2.5, `MOLD_EDGE_OFFSET_GRIPPER` `[0.8,0.8,0.4]`→`[1.2,1.2,0.9]`.
- Plan unchanged — replayed assembly order `[3, 1, 0, 4]` with part `2` as base, byte-identical to the pre-existing `commands.pkl`.
- Outputs refreshed: `fixture.obj` (bbox 25 × 20 × 4.5 cm), `fixture.png`, `pickup.json`, `stats.json` (`fixture_gen: 0.54 s`).
- `precedence.pkl` / `tree_opt.pkl` converted from nx-3.x to nx-2.6.3 (see §3); backups in `_nx3_backup/`.
