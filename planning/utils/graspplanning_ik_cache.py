'''
Durable, incremental IK-solution cache for the KUKA-via-Grasp_Planning pipeline
(see kuka_grasp_via_graspplanning_plan.md section 5).

Grasp_Planning's own IK caching (plan_simple_dual_robot_sim.py's kinematic_cache) is
in-process only -- each run_dual_assembly_benchmark.py case is a fresh subprocess, so
nothing persists across steps or across runs. This module fills that gap on the
Fabrica side: one JSON file per assembly, read-merge-written (never overwritten
wholesale), keyed by (step_id, part_id, role) so a later run can skip re-solving
steps a previous run already accepted.
'''

import json
import os
import time


def _cache_path(log_dir, assembly):
    return os.path.join(log_dir, assembly, 'graspplanning_ik_cache.json')


def load_cache(log_dir, assembly):
    path = _cache_path(log_dir, assembly)
    if not os.path.exists(path):
        return {'assembly': assembly, 'entries': {}}
    with open(path, 'r') as fp:
        return json.load(fp)


def _key(step_id, part_id, role):
    return f'{step_id}::{part_id}::{role}'


def get_entry(cache, step_id, part_id, role):
    return cache['entries'].get(_key(step_id, part_id, role))


def save_entry(log_dir, assembly, step_id, part_id, role, arm_q, meta=None):
    '''
    Read-merge-write: loads the current on-disk cache, adds/overwrites this one
    entry, writes back. Safe to call once per accepted step from a long-running
    orchestrator without losing earlier entries if the process is interrupted
    partway through a multi-part sequence.
    '''
    path = _cache_path(log_dir, assembly)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cache = load_cache(log_dir, assembly)
    cache['entries'][_key(step_id, part_id, role)] = {
        'step_id': step_id,
        'part_id': part_id,
        'role': role,
        'arm_q': list(arm_q),
        'source': 'graspplanning',
        'verified': True,
        'timestamp': time.time(),
        'meta': meta or {},
    }
    tmp_path = path + '.tmp'
    with open(tmp_path, 'w') as fp:
        json.dump(cache, fp, indent=2)
    os.replace(tmp_path, path)
    return cache
