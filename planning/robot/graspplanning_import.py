import os
import glob
import json

import numpy as np


DEFAULT_GRASPPLANNING_CACHE_DIR = os.path.expanduser('~/Grasp_Planning/artifacts/dual_grasp_planning/stage1_cache')


def _load_stage1_cache(assembly_name, part_id, cache_dir):
    cache_dir = cache_dir or DEFAULT_GRASPPLANNING_CACHE_DIR
    pattern = os.path.join(cache_dir, f'obj_fabrica_{assembly_name}_{part_id}_*.json')
    matches = sorted(glob.glob(pattern))
    if len(matches) == 0:
        raise FileNotFoundError(f'no Grasp_Planning stage-1 cache found for pattern: {pattern}')
    with open(matches[-1], 'r') as fp:
        return json.load(fp), matches[-1]


def _correct_frame(pts_cm, fabrica_part_mesh_raw, fabrica_final_transform):
    '''
    Shared frame-correction logic (see load_graspplanning_antipodal_pairs's docstring for why
    this is needed). `pts_cm` is an (N, 2, 3) array of one or more antipodal contact-point pairs,
    in centimeters, in Grasp_Planning's own per-object frame.
    '''
    if fabrica_part_mesh_raw is not None:
        pts_flat = pts_cm.reshape(-1, 3)
        gp_center = (pts_flat.min(axis=0) + pts_flat.max(axis=0)) / 2.0
        fab_bounds = fabrica_part_mesh_raw.bounds
        fab_center = (fab_bounds[0] + fab_bounds[1]) / 2.0
        pts_cm = pts_cm + (fab_center - gp_center)

    if fabrica_final_transform is not None:
        T = np.asarray(fabrica_final_transform, dtype=float)
        pts_cm = np.einsum('ij,nkj->nki', T[:3, :3], pts_cm) + T[:3, 3]

    return pts_cm


def load_graspplanning_antipodal_pair_by_grasp_id(assembly_name, part_id, grasp_id, cache_dir=None,
                                                    fabrica_part_mesh_raw=None, fabrica_final_transform=None):
    '''
    Like load_graspplanning_antipodal_pairs, but returns only the one antipodal pair whose
    stage-1 candidate['grasp_id'] == grasp_id, as a (1, 2, 3) array, plus the raw candidate dict
    (for `jaw_width_m` / rank / score inspection by callers).
    '''
    data, matched_path = _load_stage1_cache(assembly_name, part_id, cache_dir)
    for candidate in data['raw_candidates']:
        if candidate['grasp_id'] == grasp_id:
            pts_cm = (np.array(candidate['contact_points_obj'], dtype=float) * 100.0)[None, :, :]
            pts_cm = _correct_frame(pts_cm, fabrica_part_mesh_raw, fabrica_final_transform)
            return pts_cm, candidate
    raise KeyError(f'grasp_id {grasp_id!r} not found for {assembly_name}/{part_id} in {matched_path}')


def load_graspplanning_antipodal_pairs(assembly_name, part_id, cache_dir=None, dedup_decimals=3,
                                        fabrica_part_mesh_raw=None, fabrica_final_transform=None):
    '''
    Read antipodal contact-point pairs sampled by the sibling ~/Grasp_Planning repo's stage-1
    grasp sampler for a given Fabrica part, as a drop-in replacement for
    planning.robot.util_grasp.compute_antipodal_pairs's return value.

    Grasp_Planning loads the exact same Fabrica .obj files directly (mesh_scale=0.01, i.e. cm->m),
    but its own "object-local" frame is NOT identical to Fabrica's raw mesh frame: Grasp_Planning
    re-centers its own per-object frame origin independently (confirmed via its own
    `source_frame_origin_obj_world` metadata on the per-step candidate files, e.g. ~5.19cm in Z
    for plumbers_block part 0) -- while X/Y happen to already match Fabrica's raw
    `part_meshes[part_id]` (pre-`part_final_transforms`) bounds almost exactly, Z commonly does
    not. Fabrica also applies its own uniform `part_final_transforms[part_id]` (e.g. a constant
    scene-repositioning shift, unrelated to Grasp_Planning's per-object frame) to place a part
    into the coordinate system its IK/collision/ground-plane machinery actually operates in.

    Passing `fabrica_part_mesh_raw` (= `GraspArmGenerator.part_meshes[part_id]`, i.e. the raw,
    pre-final-transform mesh) aligns Grasp_Planning's antipodal points to that mesh's own
    axis-aligned bounding-box center before returning -- empirically this leaves X/Y translation
    near zero (they already matched) and corrects Z. Passing `fabrica_final_transform` (=
    `GraspArmGenerator.part_final_transforms[part_id]`) additionally applies Fabrica's own
    final-placement transform on top, so the returned pairs land directly in the same frame
    `compute_antipodal_pairs`'s own output would have been in. Omitting either leaves that stage
    of the correction out -- only do that if you're deliberately working in Grasp_Planning's raw
    per-object frame for some other purpose.
    '''
    data, _ = _load_stage1_cache(assembly_name, part_id, cache_dir)

    seen = set()
    pairs = []
    for candidate in data['raw_candidates']:
        pts_cm = np.array(candidate['contact_points_obj'], dtype=float) * 100.0
        # many candidates share the same contact pair across different roll angles / scores;
        # dedup here since Fabrica's own generate_gripper_states() will do its own angle sweep
        key = tuple(sorted([tuple(np.round(pts_cm[0], dedup_decimals)), tuple(np.round(pts_cm[1], dedup_decimals))]))
        if key in seen:
            continue
        seen.add(key)
        pairs.append(pts_cm)

    pairs = np.array(pairs)
    return _correct_frame(pairs, fabrica_part_mesh_raw, fabrica_final_transform)
