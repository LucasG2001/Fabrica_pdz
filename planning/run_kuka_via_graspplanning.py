'''
Orchestrator for the "KUKA plumbers_block grasping via Grasp_Planning" plan
(see kuka_grasp_via_graspplanning_plan.md).

Given a Fabrica fixture's pickup.json (per-part rest poses in a physically valid,
collision-free tray layout -- Franka's, reused for KUKA per the plan's fallback), this:

  1. converts those poses into a Grasp_Planning pre_insertion_poses.json variant and a
     mirrored asset-root directory (planning/utils/fixture_pose_to_graspplanning.py),
     leaving the original Grasp_Planning assembly untouched;
  2. writes throwaway Grasp_Planning YAML configs pointing at that asset root and at
     the requested assembly world placement (x=0.65, y=0.0 by default);
  3. shells out to Grasp_Planning's own scripts/build_dual_grasp_pairs.py (stage 1-3
     grasp/pair generation) and scripts/run_dual_assembly_benchmark.py (the
     precedence-ordered sequential execution driver) with those configs;
  4. best-effort parses each case's plan.json for accepted IK joints and persists them
     via planning/utils/graspplanning_ik_cache.py, so a later run can build on top of
     a partial/interrupted one instead of re-solving from scratch.

Every Grasp_Planning invocation's raw stdout/stderr is preserved under
logs/kuka_via_graspplanning/<assembly>/ for a human (or a review subagent) to inspect
in full, since some of this pipeline's real dependencies (Isaac, MoveIt/ROS2 workspace
paths) are unverified on this machine -- see the plan doc's "Risks" section.
'''

import argparse
import glob
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time

PROJECT_BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.append(PROJECT_BASE_DIR)

from planning.utils.fixture_pose_to_graspplanning import (
    compute_fixture_pre_insertion_poses, build_fixture_asset_root,
)
from planning.utils import graspplanning_ik_cache

try:
    import yaml
except ImportError:
    yaml = None


def log(msg):
    print(f'[run_kuka_via_graspplanning] {msg}', flush=True)


def find_fixture_pickup_json(fabrica_root, assembly, preferred_arm_dirs):
    '''
    Prefers a KUKA-specific fixture if one exists on disk; falls back to franka's,
    per the plan's documented fallback (no KUKA fixture has ever been generated for
    this assembly as of 2026-08-18 -- see plan doc section 2).
    '''
    for arm_dir in preferred_arm_dirs:
        candidate = os.path.join(fabrica_root, 'logs', arm_dir, assembly, 'fixture', 'pickup.json')
        if os.path.exists(candidate):
            return candidate, arm_dir
    raise FileNotFoundError(
        f'No fixture pickup.json found under logs/{{{",".join(preferred_arm_dirs)}}}/{assembly}/fixture/')


def run_logged(command, cwd, log_path):
    log(f'running: {" ".join(command)}')
    log(f'  cwd={cwd}')
    log(f'  log={log_path}')
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    started = time.time()
    with open(log_path, 'w') as fp:
        fp.write(f'$ {" ".join(command)}\n')
        fp.flush()
        proc = subprocess.run(command, cwd=cwd, stdout=fp, stderr=subprocess.STDOUT)
    elapsed = time.time() - started
    log(f'  exit={proc.returncode} elapsed={elapsed:.1f}s')
    return proc.returncode


def write_stage123_config(template_path, asset_root_abs, output_root_abs, assembly):
    with open(template_path, 'r') as fp:
        cfg = yaml.safe_load(fp)
    cfg['assembly']['name'] = assembly
    cfg['assembly']['asset_root'] = asset_root_abs
    cfg.setdefault('artifacts', {})['output_root'] = output_root_abs
    # stage1 cache is pose-keyed (see plan doc section 2) so it's safe to keep shared,
    # but route it under the fixture output root to keep this run's cache separate
    # and inspectable.
    cfg.setdefault('planning', {})['stage1_cache_dir'] = os.path.join(output_root_abs, 'stage1_cache')
    return cfg


def write_benchmark_config(template_path, artifact_root_abs, output_dir_abs, assembly, x, y, z):
    with open(template_path, 'r') as fp:
        cfg = yaml.safe_load(fp)
    cfg['benchmark']['assembly'] = assembly
    cfg['benchmark']['artifact_root'] = artifact_root_abs
    cfg['benchmark']['output_dir'] = output_dir_abs
    cfg.setdefault('base', {})
    cfg['base']['position_world_m'] = [x, y, z]
    return cfg


def extract_ik_from_plan_json(plan_json_path):
    '''
    Best-effort: this pipeline's exact plan.json joint-field names are unverified on
    this machine (no case has been run through it yet). Searches recursively for any
    key that looks like a joint-solution field and returns {found_key: value, ...}
    plus the raw parsed content for traceability, rather than assuming one exact
    schema and silently dropping data if it's wrong.
    '''
    with open(plan_json_path, 'r') as fp:
        data = json.load(fp)

    joint_like_keys = {}

    def walk(obj, path=''):
        if isinstance(obj, dict):
            for k, v in obj.items():
                key_lower = k.lower()
                if any(tok in key_lower for tok in ('joint', 'arm_q', 'ik_solution')) and isinstance(v, list):
                    joint_like_keys[f'{path}/{k}'] = v
                walk(v, f'{path}/{k}')
        elif isinstance(obj, list):
            for i, item in enumerate(obj):
                walk(item, f'{path}[{i}]')

    walk(data)
    return joint_like_keys, data


def cache_ik_from_benchmark_output(benchmark_output_dir, fabrica_log_dir, assembly):
    cases_glob = os.path.join(benchmark_output_dir, 'cases', '*', 'plan.json')
    plan_files = sorted(glob.glob(cases_glob))
    if not plan_files:
        log(f'no case plan.json files found under {cases_glob} -- nothing to cache yet')
        return 0

    cached = 0
    for plan_path in plan_files:
        case_id = os.path.basename(os.path.dirname(plan_path))
        try:
            joint_fields, raw = extract_ik_from_plan_json(plan_path)
        except Exception as exc:  # noqa: BLE001 - best-effort extraction, log and continue
            log(f'  {case_id}: failed to parse plan.json ({exc}), skipping')
            continue

        if not joint_fields:
            log(f'  {case_id}: no joint-like fields found in plan.json (schema unrecognized) '
                f'-- caching raw plan.json for manual inspection instead')
            joint_fields = {}

        graspplanning_ik_cache.save_entry(
            fabrica_log_dir, assembly,
            step_id=case_id, part_id=case_id, role='benchmark_case',
            arm_q=next(iter(joint_fields.values()), []),
            meta={'plan_json_path': plan_path, 'joint_like_fields': list(joint_fields.keys()), 'raw': raw},
        )
        cached += 1
        log(f'  {case_id}: cached ({len(joint_fields)} joint-like field(s) found)')
    return cached


def _read_jsonl(path):
    if not os.path.exists(path):
        return []
    records = []
    with open(path, 'r') as fp:
        for line in fp:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def _stage1_grasp_id_from_role_id(role_grasp_id, role, part_id):
    '''
    Translate a Grasp_Planning stage-2/3 pair-planning role id (e.g. holder "h0820", inserter
    "i0_1603__sym_object_y_bounds_center_order2_step1") back into the plain stage-1 grasp_id
    ("g0820"/"g1603") that the stage-1 cache's raw_candidates[i]['grasp_id'] actually uses.

    Confirmed by direct inspection this session (dual_grasp_pair_planner.py's
    _inserter_candidate(): grasp_id=f"i{incoming_part_id}_{suffix}" where suffix is the stage-1
    id's digits) that pair-planning role ids are NOT the same strings as stage-1 ids -- a
    transition-symmetry suffix ("__<label>") may also be appended by a later pipeline stage.
    Feeding a raw role id straight into
    graspplanning_import.load_graspplanning_antipodal_pair_by_grasp_id() (which matches stage-1's
    raw_candidates by exact grasp_id) silently raises KeyError, since stage-1 ids are always
    "g####" and role ids never are.
    '''
    prefix = 'h' if role == 'holder' else f'i{part_id}_' if role == 'inserter' else None
    if prefix is None:
        raise ValueError(f'unknown role {role!r} (expected "holder" or "inserter")')
    if not role_grasp_id.startswith(prefix):
        raise ValueError(f'{role} grasp_id {role_grasp_id!r} does not start with expected prefix {prefix!r}')
    digits = role_grasp_id[len(prefix):].split('__', 1)[0]
    return f'g{digits}'


def _lookup_pair_grasp_ids(stage123_output_root, assembly, step_id, pair_id):
    '''
    Reads the stage 2-3 pair-planning artifact (dual_grasp_pairs_<step_id>.json, written by
    build_dual_grasp_pairs.py) and returns (holder_grasp_id, inserter_grasp_id) -- pair-planning
    role ids, NOT yet translated to stage-1 ids; see _stage1_grasp_id_from_role_id().
    '''
    pairs_path = os.path.join(stage123_output_root, assembly, f'dual_grasp_pairs_{step_id}.json')
    with open(pairs_path, 'r') as fp:
        data = json.load(fp)
    for pair in data['retained_pairs']:
        if pair['pair_id'] == pair_id:
            return pair['holder_grasp_id'], pair['inserter_grasp_id']
    raise KeyError(f'pair_id {pair_id!r} not found in {pairs_path}')


def start_shared_moveit(graspplanning_root, log_path, ros_domain_id=0, ik_solver='kdl', timeout_s=90.0):
    '''
    Starts one mock MoveIt stack (the same start_dual_lbr_moveit.sh invocation
    run_dual_assembly_benchmark.py's own managed-stack mode uses internally) that
    find_one_successful_case()'s repeated `--ik-only --reuse-moveit` subprocess calls can all
    share, instead of paying the ~45s per-case MoveIt restart cost. `--reuse-moveit` at the CLI
    level only means "assume a stack is already running, don't start one" -- it never starts one
    itself, so this external launch is required (see kuka_grasp_handoff.md's "Important semantic
    note").
    '''
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    log_fp = open(log_path, 'w')
    command = [
        './start_dual_lbr_moveit.sh', '--mode', 'mock',
        '--ros-domain-id', str(ros_domain_id), '--ik-solver', ik_solver,
    ]
    log(f'starting shared mock MoveIt stack: {" ".join(command)} (cwd={graspplanning_root})')
    process = subprocess.Popen(
        command, cwd=graspplanning_root, stdout=log_fp, stderr=subprocess.STDOUT,
        text=True, start_new_session=True,
    )
    ready_marker = 'You can start planning now!'
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if process.poll() is not None:
            log_fp.close()
            raise RuntimeError(
                f'shared MoveIt stack exited early (rc={process.returncode}); see {log_path}')
        with open(log_path, 'r') as fp:
            if ready_marker in fp.read():
                log('shared mock MoveIt stack is ready.')
                return process, log_fp
        time.sleep(0.5)
    log_fp.close()
    raise RuntimeError(f'shared MoveIt stack did not become ready within {timeout_s}s; see {log_path}')


def stop_shared_moveit(process, log_fp):
    log('stopping shared mock MoveIt stack')
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=10)
    except ProcessLookupError:
        pass
    finally:
        log_fp.close()


def find_one_successful_case(graspplanning_root, benchmark_config, config_path, part_id, output_dir, log_dir,
                              ros_domain_id=0):
    '''
    Loops candidate placement/orientation combos from the benchmark config (the same 12
    placements x 8 orientations = 96 combos/part an exhaustive sweep would use), each time
    invoking `run_dual_assembly_benchmark.py --parts <part_id> --placements <p> --orientations <o>
    --limit-cases 1 --ik-only --reuse-moveit` against the already-running shared MoveIt stack, and
    stops at the first case whose events.jsonl record has status == "success". Resume-safe: first
    checks events.jsonl for an already-successful case for this part from a prior (partial) run.

    Returns the winning events.jsonl record dict (has step_id/pair_id/case_id, among others).
    '''
    events_path = os.path.join(output_dir, 'events.jsonl')

    def _existing_success():
        for record in _read_jsonl(events_path):
            if str(record.get('incoming_part_id')) == str(part_id) and record.get('status') == 'success':
                return record
        return None

    existing = _existing_success()
    if existing is not None:
        log(f'part {part_id}: reusing existing successful case {existing["case_id"]!r} from a prior run')
        return existing

    placement_ids = [p['id'] for p in benchmark_config['placements']]
    orientation_ids = [o['id'] for o in benchmark_config['orientations']]
    log(f'part {part_id}: searching up to {len(placement_ids)}x{len(orientation_ids)} '
        f'placement/orientation combos for the first dual-feasible one')

    for placement_id in placement_ids:
        for orientation_id in orientation_ids:
            command = [
                'python3', 'scripts/run_dual_assembly_benchmark.py',
                '--config', config_path,
                '--parts', str(part_id),
                '--placements', placement_id,
                '--orientations', orientation_id,
                '--limit-cases', '1',
                '--ik-only', '--reuse-moveit',
                '--ros-domain-id', str(ros_domain_id),
            ]
            run_logged(
                command, cwd=graspplanning_root,
                log_path=os.path.join(
                    log_dir, 'targeted_search',
                    f'part_{part_id}_{placement_id}_{orientation_id}.log'),
            )
            record = _existing_success()
            if record is not None:
                log(f'part {part_id}: SUCCESS at placement={placement_id!r} orientation={orientation_id!r}')
                return record

    raise RuntimeError(
        f'part {part_id}: no successful case found across all '
        f'{len(placement_ids)}x{len(orientation_ids)} placement/orientation combos')


def run_targeted_grasp_search(graspplanning_root, config_path, precedence_order, output_json_path, log_dir,
                               ros_domain_id=0, ik_solver='kdl'):
    '''
    Part 2 of nifty-munching-snowflake.md: for each part in `precedence_order`, find one
    dual-feasible grasp_id (fast, via find_one_successful_case()), then persist
    {"<part_id>": {"holder_grasp_id": "g...", "inserter_grasp_id": "g..."}, ...} to
    `output_json_path` for Part 3 (run_grasp_arm_gen.py --graspplanning-grasp-ids-json) to consume.
    Starts one shared mock MoveIt stack for the whole search, tears it down once at the end.
    '''
    with open(config_path, 'r') as fp:
        benchmark_config = yaml.safe_load(fp)
    benchmark = benchmark_config['benchmark']
    output_dir = benchmark['output_dir']
    stage123_output_root = benchmark['artifact_root']
    assembly = benchmark['assembly']

    process, log_fp = start_shared_moveit(
        graspplanning_root, log_path=os.path.join(log_dir, 'shared_moveit_search.log'),
        ros_domain_id=ros_domain_id, ik_solver=ik_solver)
    try:
        targeted_cases = {}
        for part_id in precedence_order:
            record = find_one_successful_case(
                graspplanning_root, benchmark_config, config_path, part_id, output_dir, log_dir,
                ros_domain_id=ros_domain_id)
            step_id = record['step_id']
            pair_id = record['pair_id']
            holder_role_id, inserter_role_id = _lookup_pair_grasp_ids(
                stage123_output_root, assembly, step_id, pair_id)
            holder_grasp_id = _stage1_grasp_id_from_role_id(holder_role_id, 'holder', part_id)
            inserter_grasp_id = _stage1_grasp_id_from_role_id(inserter_role_id, 'inserter', part_id)
            targeted_cases[str(part_id)] = {
                'holder_grasp_id': holder_grasp_id,
                'inserter_grasp_id': inserter_grasp_id,
                'case_id': record['case_id'],
                'pair_id': pair_id,
                'step_id': step_id,
            }
            log(f'part {part_id}: holder_grasp_id={holder_grasp_id} inserter_grasp_id={inserter_grasp_id} '
                f'(from pair_id={pair_id}, roles were {holder_role_id}/{inserter_role_id})')
    finally:
        stop_shared_moveit(process, log_fp)

    os.makedirs(os.path.dirname(output_json_path), exist_ok=True)
    with open(output_json_path, 'w') as fp:
        json.dump(targeted_cases, fp, indent=2)
    log(f'wrote {output_json_path}')
    return targeted_cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assembly', default='plumbers_block')
    parser.add_argument('--fabrica-root', default=PROJECT_BASE_DIR)
    parser.add_argument('--graspplanning-root', default=os.path.expanduser('~/Grasp_Planning'))
    parser.add_argument('--fixture-arm-dirs', nargs='+', default=['kuka2', 'kuka3', 'franka3'],
                         help='Search order for the fixture to reuse; first existing pickup.json wins.')
    parser.add_argument('--base-part-id', default=None,
                         help='Defaults to reading assembly_sequence.json base_part_id from the original run.')
    parser.add_argument('--assembly-x', type=float, default=0.65)
    parser.add_argument('--assembly-y', type=float, default=0.0)
    parser.add_argument('--assembly-z', type=float, default=-0.03)
    parser.add_argument('--skip-stage123', action='store_true',
                         help='Skip build_dual_grasp_pairs.py (reuse a previous run\'s artifacts).')
    parser.add_argument('--skip-benchmark', action='store_true',
                         help='Skip run_dual_assembly_benchmark.py entirely (fixture-conversion + stage1-3 only).')
    parser.add_argument('--stage123-extra-args', default='',
                         help='Extra args (single shell-quoted string) forwarded to build_dual_grasp_pairs.py, '
                              'e.g. --stage123-extra-args "--max-inserter-candidates-per-step 5 --max-pair-checks 50" '
                              'for a fast bounded smoketest instead of the full-scale run.')
    parser.add_argument('--benchmark-extra-args', default='',
                         help='Extra args (single shell-quoted string) forwarded to run_dual_assembly_benchmark.py, '
                              'e.g. --benchmark-extra-args "--ik-only --limit-cases 1"')
    parser.add_argument('--targeted-grasp-search', action='store_true',
                         help='Part 2 of nifty-munching-snowflake.md: instead of the fixture-conversion + '
                              'stage1-3 + full-benchmark flow above, run a fast targeted "first success per '
                              'part" search (find_one_successful_case()) against an already-existing '
                              'fixture-anchored benchmark config, and write targeted_cases.json for Part 3 '
                              '(run_grasp_arm_gen.py --graspplanning-grasp-ids-json) to consume.')
    parser.add_argument('--precedence-order', nargs='+', default=['0', '3', '1', '4'],
                         help='incoming_part_id search order for --targeted-grasp-search (base part excluded '
                              '-- it is never an incoming_part_id, see assembly_sequence.json).')
    parser.add_argument('--benchmark-config-path', default=None,
                         help='Fixture-anchored benchmark config for --targeted-grasp-search. Defaults to '
                              'output/graspplanning_fixture_assets/dual_assembly_benchmark_fixture.yaml under '
                              '--fabrica-root, i.e. the config STEP 3/3 above already writes.')
    parser.add_argument('--targeted-cases-output', default=None,
                         help='Defaults to logs/kuka_via_graspplanning/<assembly>/targeted_cases.json under '
                              '--fabrica-root.')
    parser.add_argument('--search-ros-domain-id', type=int, default=0)
    parser.add_argument('--search-ik-solver', default='kdl', choices=['kdl', 'pick_ik'])
    args = parser.parse_args()

    if yaml is None:
        log('ERROR: PyYAML not importable in this Python. Grasp_Planning itself depends on '
            'PyYAML>=6.0 (pyproject.toml), so this environment cannot run its scripts either. Aborting.')
        return 1

    fabrica_log_dir = os.path.join(args.fabrica_root, 'logs', 'kuka_via_graspplanning')
    fixture_asset_root = os.path.join(args.fabrica_root, 'output', 'graspplanning_fixture_assets')

    if not os.path.isdir(args.graspplanning_root):
        log(f'ERROR: --graspplanning-root {args.graspplanning_root} does not exist.')
        return 1

    if args.targeted_grasp_search:
        benchmark_config_path = args.benchmark_config_path or os.path.join(
            fixture_asset_root, 'dual_assembly_benchmark_fixture.yaml')
        if not os.path.exists(benchmark_config_path):
            log(f'ERROR: --targeted-grasp-search requires an existing fixture-anchored benchmark config at '
                f'{benchmark_config_path} (run this script once without --targeted-grasp-search first to '
                f'generate it via STEP 1-3, or pass --benchmark-config-path explicitly).')
            return 1
        targeted_cases_output = args.targeted_cases_output or os.path.join(
            fabrica_log_dir, args.assembly, 'targeted_cases.json')
        log('=' * 70)
        log('TARGETED GRASP SEARCH (Part 2): first dual-feasible grasp_id per precedence step')
        log('=' * 70)
        run_targeted_grasp_search(
            args.graspplanning_root, benchmark_config_path, args.precedence_order, targeted_cases_output,
            log_dir=os.path.join(fabrica_log_dir, args.assembly),
            ros_domain_id=args.search_ros_domain_id, ik_solver=args.search_ik_solver)
        return 0

    source_assembly_dir = os.path.join(args.graspplanning_root, 'assets', 'obj', 'fabrica', args.assembly)
    output_assembly_dir = os.path.join(fixture_asset_root, args.assembly)

    if not os.path.isdir(source_assembly_dir):
        log(f'ERROR: source assembly dir {source_assembly_dir} does not exist.')
        return 1

    base_part_id = args.base_part_id
    if base_part_id is None:
        seq_path = os.path.join(args.graspplanning_root, 'artifacts', 'dual_grasp_planning',
                                 args.assembly, 'assembly_sequence.json')
        if os.path.exists(seq_path):
            with open(seq_path, 'r') as fp:
                base_part_id = json.load(fp)['base_part_id']
            log(f'base_part_id auto-detected from {seq_path}: {base_part_id!r}')
        else:
            log(f'ERROR: --base-part-id not given and {seq_path} does not exist to auto-detect from.')
            return 1

    log('=' * 70)
    log('STEP 1/3: convert fixture pickup poses -> Grasp_Planning pre_insertion_poses.json')
    log('=' * 70)
    try:
        pickup_json_path, chosen_arm_dir = find_fixture_pickup_json(
            args.fabrica_root, args.assembly, args.fixture_arm_dirs)
    except FileNotFoundError as exc:
        log(f'ERROR: {exc}')
        return 1
    log(f'using fixture: logs/{chosen_arm_dir}/{args.assembly}/fixture/pickup.json')

    orig_pre_insertion = os.path.join(source_assembly_dir, 'pre_insertion_poses.json')
    new_poses = compute_fixture_pre_insertion_poses(pickup_json_path, orig_pre_insertion, base_part_id)
    build_fixture_asset_root(source_assembly_dir, output_assembly_dir, new_poses)
    conv = new_poses['fixture_conversion']
    log(f'converted parts: {conv["converted_parts"]}, skipped: {conv["skipped_parts"]}')
    log(f'fixture asset root ready: {output_assembly_dir}')

    stage123_output_root_abs = os.path.join(args.graspplanning_root, 'artifacts', 'dual_grasp_planning_fixture')
    stage123_output_dir_abs = os.path.join(stage123_output_root_abs, args.assembly)

    if not args.skip_stage123:
        log('=' * 70)
        log('STEP 2/3: regenerate stage 1-3 grasp/pair artifacts from the fixture poses')
        log('=' * 70)
        stage123_template = os.path.join(args.graspplanning_root, 'configs', 'dual_grasp_planning.yaml')
        stage123_cfg = write_stage123_config(
            stage123_template, fixture_asset_root, stage123_output_root_abs, args.assembly)
        stage123_cfg_path = os.path.join(fixture_asset_root, 'dual_grasp_planning_fixture.yaml')
        with open(stage123_cfg_path, 'w') as fp:
            yaml.safe_dump(stage123_cfg, fp)
        log(f'wrote config: {stage123_cfg_path}')

        stage123_cmd = ['python3', 'scripts/build_dual_grasp_pairs.py', '--config', stage123_cfg_path]
        stage123_cmd += shlex.split(args.stage123_extra_args)
        rc = run_logged(
            stage123_cmd,
            cwd=args.graspplanning_root,
            log_path=os.path.join(fabrica_log_dir, args.assembly, 'build_dual_grasp_pairs.log'),
        )
        if rc != 0:
            log(f'ERROR: build_dual_grasp_pairs.py exited {rc}. See log above. Aborting before benchmark.')
            return rc
    else:
        log('STEP 2/3: skipped (--skip-stage123), reusing existing stage1-3 artifacts if present')

    if args.skip_benchmark:
        log('STEP 3/3: skipped (--skip-benchmark)')
        return 0

    log('=' * 70)
    log('STEP 3/3: run the precedence-ordered sequential execution driver')
    log('=' * 70)
    benchmark_template = os.path.join(args.graspplanning_root, 'configs', 'dual_assembly_benchmark.yaml')
    benchmark_output_dir_abs = os.path.join(
        args.graspplanning_root, 'artifacts', 'dual_assembly_benchmark_fixture', args.assembly)
    benchmark_cfg = write_benchmark_config(
        benchmark_template, stage123_output_root_abs, benchmark_output_dir_abs,
        args.assembly, args.assembly_x, args.assembly_y, args.assembly_z)
    benchmark_cfg_path = os.path.join(fixture_asset_root, 'dual_assembly_benchmark_fixture.yaml')
    with open(benchmark_cfg_path, 'w') as fp:
        yaml.safe_dump(benchmark_cfg, fp)
    log(f'wrote config: {benchmark_cfg_path}')
    log(f'assembly world placement: x={args.assembly_x} y={args.assembly_y} z={args.assembly_z}')

    command = ['python3', 'scripts/run_dual_assembly_benchmark.py', '--config', benchmark_cfg_path]
    command += shlex.split(args.benchmark_extra_args)
    rc = run_logged(
        command, cwd=args.graspplanning_root,
        log_path=os.path.join(fabrica_log_dir, args.assembly, 'run_dual_assembly_benchmark.log'),
    )

    log('=' * 70)
    log('caching any IK solutions found in this run\'s case outputs')
    log('=' * 70)
    n_cached = cache_ik_from_benchmark_output(benchmark_output_dir_abs, fabrica_log_dir, args.assembly)
    log(f'cached {n_cached} case(s) to {os.path.join(fabrica_log_dir, args.assembly, "graspplanning_ik_cache.json")}')

    if rc != 0:
        log(f'run_dual_assembly_benchmark.py exited {rc} -- see log for which case(s) failed.')
    return rc


if __name__ == '__main__':
    sys.exit(main())
