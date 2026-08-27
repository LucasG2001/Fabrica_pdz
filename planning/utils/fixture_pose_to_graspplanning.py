'''
Convert a Fabrica fixture's per-part pickup poses (planning/run_fixture_gen.py output,
e.g. logs/franka3/plumbers_block/fixture/pickup.json) into a Grasp_Planning
pre_insertion_poses.json variant, so parts spawn in the dual-arm scene at their real,
collision-free fixture-tray layout instead of Grasp_Planning's own disassembly-derived
staging pose.

Also builds a mirrored Grasp_Planning asset-root directory (symlinks to the original
assembly's OBJ meshes / precedence_plan.json, only pre_insertion_poses.json replaced)
so the original assembly's artifacts are never touched.

See /home/pdzuser/Fabrica/kuka_grasp_via_graspplanning_plan.md section 2 for the
derivation of the pose math used here.

Math: Fabrica's fixture writer (planning/run_fixture_gen.py:439-443) builds

    pickup_json[part] = Trans(part_translation) @ T_relative[part] @ T_final[part]

where part_translation is an arbitrary uniform fixture-render offset (not recorded to
disk) applied identically to every part. For two parts i, base:

    inv(pickup[base]) @ pickup[i]
        = inv(T_relative[base] @ T_final[base]) @ inv(Trans(part_translation))
          @ Trans(part_translation) @ T_relative[i] @ T_final[i]
        = inv(T_relative[base] @ T_final[base]) @ T_relative[i] @ T_final[i]

so part_translation cancels exactly regardless of its (unrecorded) value. This gives
every part's pickup pose relative to the base part's pickup pose, preserving the real
fixture's collision-free layout. Anchoring that relative layout at the base part's own
true final (table-resting) pose -- already known from the assembly's
pre_insertion_poses.json -- places the whole layout back in Grasp_Planning's native
assembly-asset cm frame, with no dependency on the unrecorded fixture offset.
'''

import argparse
import copy
import json
import os
import sys

import numpy as np

PROJECT_BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(PROJECT_BASE_DIR)

from assets.transform import get_transform_matrix, get_pos_euler_from_transform_matrix


def compute_fixture_pre_insertion_poses(pickup_json_path, pre_insertion_json_path, base_part_id):
    '''
    Returns a new pre_insertion_poses.json-shaped dict where every non-base part's
    pre_insertion_pose_world_cm_euler_xyz / final_to_pre_insertion_transform_cm is
    replaced by a pose derived from the real Fabrica fixture layout, anchored at the
    base part's existing final (table) pose. The base part's own entry is left
    untouched (it starts on the table at its final pose, matching the existing
    "static_base parts have no pre-insertion pose" convention).
    '''
    with open(pickup_json_path, 'r') as fp:
        pickup = json.load(fp)
    with open(pre_insertion_json_path, 'r') as fp:
        orig = json.load(fp)

    if base_part_id not in pickup:
        raise ValueError(f'base part {base_part_id!r} not found in fixture pickup.json (keys: {list(pickup.keys())})')
    if base_part_id not in orig['parts']:
        raise ValueError(f'base part {base_part_id!r} not found in pre_insertion_poses.json parts')

    base_final_vec = orig['parts'][base_part_id]['final_pose_world_cm_euler_xyz']
    T_base_final = get_transform_matrix(base_final_vec)
    T_base_pickup = get_transform_matrix(pickup[base_part_id])
    T_base_pickup_inv = np.linalg.inv(T_base_pickup)

    result = copy.deepcopy(orig)
    result['source'] = f'fixture pickup poses ({pickup_json_path}), anchored at base part {base_part_id} final pose'
    converted_parts = []
    skipped_parts = []

    for part_id, part_entry in result['parts'].items():
        if part_id == base_part_id:
            continue
        if part_id not in pickup:
            skipped_parts.append(part_id)
            continue
        if 'pre_insertion_pose_world_cm_euler_xyz' not in part_entry:
            # e.g. a second static_base-like part with no pre-insertion pose in the original
            skipped_parts.append(part_id)
            continue

        T_pickup_i = get_transform_matrix(pickup[part_id])
        T_local_i = T_base_pickup_inv @ T_pickup_i
        T_new_pre_insertion = T_base_final @ T_local_i

        final_vec = part_entry['final_pose_world_cm_euler_xyz']
        T_final_i = get_transform_matrix(final_vec)
        T_final_to_pre = T_new_pre_insertion @ np.linalg.inv(T_final_i)

        part_entry['pre_insertion_pose_world_cm_euler_xyz'] = get_pos_euler_from_transform_matrix(T_new_pre_insertion).tolist()
        part_entry['final_to_pre_insertion_transform_cm'] = T_final_to_pre.tolist()
        converted_parts.append(part_id)

    result['fixture_conversion'] = {
        'fixture_pickup_json': os.path.abspath(pickup_json_path),
        'base_part_id': base_part_id,
        'converted_parts': converted_parts,
        'skipped_parts': skipped_parts,
    }
    return result


def build_fixture_asset_root(source_assembly_dir, output_assembly_dir, new_pre_insertion_poses):
    '''
    Mirrors source_assembly_dir into output_assembly_dir: every file is symlinked
    except pre_insertion_poses.json, which is written fresh from
    new_pre_insertion_poses. Never modifies source_assembly_dir.
    '''
    os.makedirs(output_assembly_dir, exist_ok=True)
    for name in os.listdir(source_assembly_dir):
        if name == 'pre_insertion_poses.json':
            continue
        src = os.path.abspath(os.path.join(source_assembly_dir, name))
        dst = os.path.join(output_assembly_dir, name)
        if os.path.islink(dst) or os.path.exists(dst):
            os.remove(dst)
        os.symlink(src, dst)

    out_path = os.path.join(output_assembly_dir, 'pre_insertion_poses.json')
    with open(out_path, 'w') as fp:
        json.dump(new_pre_insertion_poses, fp, indent=2)
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture-pickup-json', required=True,
                         help='e.g. logs/franka3/plumbers_block/fixture/pickup.json')
    parser.add_argument('--pre-insertion-poses-json', required=True,
                         help='e.g. ~/Grasp_Planning/assets/obj/fabrica/plumbers_block/pre_insertion_poses.json')
    parser.add_argument('--source-assembly-dir', required=True,
                         help='e.g. ~/Grasp_Planning/assets/obj/fabrica/plumbers_block')
    parser.add_argument('--output-assembly-dir', required=True,
                         help='e.g. output/graspplanning_fixture_assets/plumbers_block')
    parser.add_argument('--base-part-id', required=True)
    args = parser.parse_args()

    new_poses = compute_fixture_pre_insertion_poses(
        args.fixture_pickup_json, args.pre_insertion_poses_json, args.base_part_id)
    out_path = build_fixture_asset_root(args.source_assembly_dir, args.output_assembly_dir, new_poses)

    print(f'[fixture_pose_to_graspplanning] wrote {out_path}')
    print(f'[fixture_pose_to_graspplanning] converted parts: {new_poses["fixture_conversion"]["converted_parts"]}')
    print(f'[fixture_pose_to_graspplanning] skipped parts: {new_poses["fixture_conversion"]["skipped_parts"]}')
    print(f'[fixture_pose_to_graspplanning] asset root ready: {args.output_assembly_dir}')


if __name__ == '__main__':
    main()
