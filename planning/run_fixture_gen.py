import os
os.environ['OMP_NUM_THREADS'] = '1'
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.append(project_base_dir)

import numpy as np
import pickle
from scipy.spatial.transform import Rotation as R
from scipy.spatial import Delaunay
from rectpack import newPacker
import trimesh
import json
from time import time

from assets.load import load_pos_quat_dict
from assets.transform import get_transform_matrix, get_transform_matrix_quat, mat_to_pos_quat, get_pos_euler_from_transform_matrix
from planning.robot.geometry import load_part_meshes, load_gripper_meshes, transform_gripper_meshes, get_buffered_meshes, get_gripper_basis_directions
from planning.robot.workcell import get_assembly_center, get_board_dx, get_fixture_min_y
from planning.run_seq_plan import SequencePlanner
from planning.run_seq_opt import SequenceOptimizer
from planning.utils.fixture_countersunk import (
    generate_countersunk_pad, generate_countersunk_hole,
    PAD_DIAMETER, COUNTERSUNK_DIAMETER, HOLE_DIAMETER)
from planning.utils.fixture_markers import add_aruco_markers_to_fixture, _section_polygon


# fixture board parameters
DX = get_board_dx()
BOTTOM_THICKNESS = 0.5 # bottom thickness of the fixture without mold
EDGE_THICKNESS = 3.0 # thickness of the fixture edge
MIN_MOLD_DEPTH = 1.0 # minimum depth of the mold
MOLD_EDGE_OFFSET_PART = [0.05, 0.05, 0.0] # offset from part edge to mold edge
MOLD_EDGE_OFFSET_GRIPPER = [1.2, 1.2, 0.9] # offset from gripper edge to mold edge
PART_BOUNDARY_OFFSET = 0.2 # offset from part boundary to part edge
PART_GAP = 2.5 # gap between parts
MAX_BIN_SIZE_SINGLE = [8 * DX, 10 * DX] # maximum size of bin for rect pack (one print)
MAX_BIN_SIZE_DOUBLE = [8 * DX, 20 * DX] # maximum size of bin for rect pack (two prints)
MAX_BIN_SIZE_BLOCKING = [12 * DX, 20 * DX] # maximum size of bin for rect pack (blocking collision check)
DELTA_BIN_SIZE = 1 * DX # delta size of bin for rect pack
DELTA_BUFFER_SIZE = 2.5 # delta size of buffer for part-gripper collision

# --- bottom-plate lightening (only the z <= BOTTOM_THICKNESS slab layer; never touches the
#     mold islands above it, the fixture bounding box, the pickup poses, or the markers) ---
LIGHTEN_BOTTOM = False       # OFF by default: the frame+beds+ribs cut ~15 % of the material
                            # but the extra perimeters / travel moves make it print SLOWER.
                            # True -> lighten the slab (see the FLOOR_* / MIN_CUTOUT_AREA knobs)
FLOOR_FRAME_W = 0.6          # width of the retained perimeter rail, cm (0 -> no rail)
FLOOR_POCKET_FLANGE = 0.5    # slab material kept around each carved pocket, cm
FLOOR_RIB_W = 0.6           # width of the ribs tying pockets / pads / rail together, cm
FLOOR_MARKER_MARGIN = 0.6   # slab bed kept around the ArUco marker footprint, cm
FLOOR_PAD_MARGIN = 0.5      # slab kept around each countersunk pad centre, cm
MIN_CUTOUT_AREA = 4.0      # only open a bottom-slab window whose footprint exceeds this,
                          # cm^2 (smaller cut-outs are left solid; many tiny holes just add
                          # perimeters + travel moves and slow the print). CLI: --min-cutout-area

# --- mounting screw holes ---
MOUNT_STYLE = 'slab'      # 'slab':  countersunk holes drilled straight into the solid bottom
                          #          slab on a SCREW_PITCH grid, clear of the pockets + ArUco
                          #          tiles, BEFORE the lightening (which then keeps a disc of
                          #          slab around each). Envelope unchanged -> body stays small.
                          # 'ears':  holes on tabs projecting from the -X/+X/+Y edges; grows
                          #          the XY envelope outward (bigger print).
                          # 'corners': legacy 4 in-slab pads at the raw board corners.
SCREW_PITCH = 5.0         # mounting hole-to-hole spacing, cm (grid pitch / ear-edge pitch)
HOLE_EDGE_INSET = 1.1     # 'slab': keep a drilled hole centre this far in from the slab edge, cm
HOLE_ISLAND_CLR = 1.0     # 'slab': ... and this far from a carved pocket, cm
HOLE_MARKER_CLR = 0.7     # 'slab': ... and this far from an ArUco marker tile, cm
EAR_STICKOUT = 2.5        # 'ears': ear hole-centre distance past the body edge, cm
EAR_HALF_W = 1.4         # 'ears': half the ear-tab width; also hole-to-tip material, cm
EAR_MARGIN = 1.0         # 'ears': min gap from an ear hole to the end of its edge, cm


def generate_individual_pose_info(part_cfg_final, sequence, grasps_sequence, gripper_type):

    part_meshes_final = part_cfg_final['mesh']
    pose_info = {}
    sequence_forward = sequence[::-1]
    grasps_sequence_forward = grasps_sequence[::-1]

    # Pickup orientation aligns the grasp's finger-closing axis to the fixture's +X and its
    # approach axis to -Z. Both directions must come from THIS gripper's basis, not a fixed
    # literal: grasp.quat is stored gripper-native (util_grasp.get_gripper_pos_quat builds it
    # from get_gripper_basis_directions(gripper_type)), so applying the historical Panda
    # literals [0,-1,0] (-l2r) / [0,0,1] (-approach) only works while the l2r basis is [0,1,0].
    # pdz closes along +X (robotiq-85 along -X), so the literal selected a transverse axis and
    # planted the part ~90 deg off. -basis reproduces the old values exactly for
    # panda / kuka / robotiq-140.
    base_basis, l2r_basis = get_gripper_basis_directions(gripper_type)
    gripper_l2r_basis = -np.asarray(l2r_basis, dtype=float)
    gripper_b2f_basis = -np.asarray(base_basis, dtype=float)

    for i, ((part_move, part_hold), (grasps_move, grasp_hold)) in enumerate(zip(sequence_forward, grasps_sequence_forward)):
        grasp_move_final = grasps_move[0]

        if i == 0: # first step, both arm pick up
            gripper_l2r_dir = R.from_quat(grasp_hold.quat[[1, 2, 3, 0]]).apply(gripper_l2r_basis)
            gripper_b2f_dir = R.from_quat(grasp_hold.quat[[1, 2, 3, 0]]).apply(gripper_b2f_basis)
            target_l2r_dir = np.array([1, 0, 0])
            target_b2f_dir = np.array([0, 0, -1])
            pickup_rot_mat = R.align_vectors([target_l2r_dir, target_b2f_dir], [gripper_l2r_dir, gripper_b2f_dir])[0].as_matrix()

            hold_mesh = part_meshes_final[part_hold].copy()
            pickup_transform_mat = np.eye(4)
            pickup_transform_mat[:3, :3] = pickup_rot_mat
            hold_mesh.apply_transform(pickup_transform_mat)

            pose_info[part_hold] = {
                'extent_x': hold_mesh.extents[0], 
                'extent_y': hold_mesh.extents[1], 
                'center_x': np.min(hold_mesh.vertices[:, 0]) + hold_mesh.extents[0] / 2,
                'center_y': np.min(hold_mesh.vertices[:, 1]) + hold_mesh.extents[1] / 2,
                'min_z': np.min(hold_mesh.vertices[:, 2]),
                'rot_mat': pickup_rot_mat,
            }
        
        # move arm pick up
        gripper_l2r_dir = R.from_quat(grasp_move_final.quat[[1, 2, 3, 0]]).apply(gripper_l2r_basis)
        gripper_b2f_dir = R.from_quat(grasp_move_final.quat[[1, 2, 3, 0]]).apply(gripper_b2f_basis)
        target_l2r_dir = np.array([1, 0, 0])
        target_b2f_dir = np.array([0, 0, -1])
        pickup_rot_mat = R.align_vectors([target_l2r_dir, target_b2f_dir], [gripper_l2r_dir, gripper_b2f_dir])[0].as_matrix()

        move_mesh = part_meshes_final[part_move].copy()
        pickup_transform_mat = np.eye(4)
        pickup_transform_mat[:3, :3] = pickup_rot_mat
        move_mesh.apply_transform(pickup_transform_mat)

        pose_info[part_move] = {
            'extent_x': move_mesh.extents[0], 
            'extent_y': move_mesh.extents[1], 
            'center_x': np.min(move_mesh.vertices[:, 0]) + move_mesh.extents[0] / 2,
            'center_y': np.min(move_mesh.vertices[:, 1]) + move_mesh.extents[1] / 2,
            'min_z': np.min(move_mesh.vertices[:, 2]),
            'rot_mat': pickup_rot_mat,
        }

    return pose_info


def plot_packing(packer):
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    for index, abin in enumerate(packer):
        bw, bh  = abin.width, abin.height

        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        for rect in abin:
            x, y, w, h = rect.x, rect.y, rect.width, rect.height
            plt.axis([0,bw,0,bh])
            # print('rectangle', w,h)
            patch = patches.Rectangle(
                                        (x, y),  # (x,y)
                                        w,          # width
                                        h,          # height
                                        facecolor="#00ffff",
                                        edgecolor="black",
                                        linewidth=3
                                    )
            ax.add_patch(patch)
            rx, ry = patch.get_xy()
            cx = rx + patch.get_width()/2.0
            cy = ry + patch.get_height()/2.0

            ax.annotate(f'w:{w}\nh:{h}', (cx, cy), color='b', weight='bold', 
                        fontsize=4, ha='center', va='center')
        
        plt.show()


def run_bin_packing(pose_info, bin_size):
    packer = newPacker(rotation=False)

    for part_id, part_pose_info in pose_info.items():
        packer.add_rect(part_pose_info['extent_x'] + PART_GAP, part_pose_info['extent_y'] + PART_GAP, part_id)
    packer.add_bin(bin_size[0], bin_size[1])

    packer.pack()
    all_rects = packer.rect_list()
    if len(all_rects) == len(pose_info):
        return packer
    else:
        return None


def generate_pickup_pose(pose_info, min_fixture_y, render=False):

    packer = run_bin_packing(pose_info, MAX_BIN_SIZE_DOUBLE) # try big bin size
    if packer is None:
        return None, None # no feasible bin size
    
    packer = run_bin_packing(pose_info, MAX_BIN_SIZE_SINGLE) # try small bin size
    if packer is not None:
        max_bin_size = MAX_BIN_SIZE_SINGLE
    else:
        max_bin_size = MAX_BIN_SIZE_DOUBLE

    best_packer = packer
    best_bin_size = max_bin_size
    best_bin_area = np.prod(max_bin_size)

    delta_bin_size = DELTA_BIN_SIZE
    min_bin_area = np.sum([(pose_info[part_id]['extent_x'] + PART_GAP) * (pose_info[part_id]['extent_y'] + PART_GAP) for part_id in pose_info.keys()])
    min_bin_x = max(np.ceil(min_bin_area / max_bin_size[1] / delta_bin_size), 4) * delta_bin_size
    min_bin_y = max(np.ceil(min_bin_area / max_bin_size[0] / delta_bin_size), 4) * delta_bin_size

    for bin_x in np.arange(min_bin_x, max_bin_size[0] + 0.5 * delta_bin_size, delta_bin_size):
        for bin_y in np.arange(min_bin_y, max_bin_size[1] + 0.5 * delta_bin_size, delta_bin_size):
            current_area = bin_x * bin_y
            if current_area >= best_bin_area:
                continue

            packer = run_bin_packing(pose_info, [bin_x, bin_y])
            if packer is not None:
                best_packer = packer
                best_bin_size = [bin_x, bin_y]
                best_bin_area = current_area

    packer, bin_size = best_packer, best_bin_size

    if render:
        plot_packing(packer)

    pickup_pose = {}
    for rect in packer[0]:
        part_id = rect.rid
        part_transform = np.eye(4)
        part_transform[:3, :3] = pose_info[part_id]['rot_mat']
        part_transform[:3, 3] = np.array([
            rect.x + rect.width / 2 - pose_info[part_id]['center_x'] - bin_size[0] / 2,
            rect.y + rect.height / 2 - pose_info[part_id]['center_y'] + min_fixture_y,
            BOTTOM_THICKNESS - pose_info[part_id]['min_z']])
        pickup_pose[part_id] = get_pos_euler_from_transform_matrix(part_transform).tolist()

    return pickup_pose, bin_size


def get_swept_mesh(mesh_start, mesh_end):
    mesh_swept = trimesh.points.PointCloud(mesh_start.vertices.tolist() + mesh_end.vertices.tolist())
    mesh_swept = trimesh.convex.convex_hull(np.unique(mesh_swept.vertices.round(decimals=6), axis=0), qhull_options='Qx Qs Qt')
    return mesh_swept


def generate_pickup_meshes(part_cfg_final, sequence, grasps_sequence, gripper_type, pickup_pose):

    part_meshes_final = part_cfg_final['mesh']
    asset_folder = os.path.join(project_base_dir, 'assets')
    gripper_meshes = load_gripper_meshes(gripper_type, asset_folder)

    # pickup part meshes
    part_meshes_pickup = {k: v.copy() for k, v in part_meshes_final.items()}
    for part_id, part_transform in pickup_pose.items():
        part_meshes_pickup[part_id].apply_transform(get_transform_matrix(part_transform))
    
    # pickup gripper meshes
    sequence_forward = sequence[::-1]
    grasps_sequence_forward = grasps_sequence[::-1]
    gripper_meshes_pickup = {}

    for i, ((part_move, part_hold), (grasps_move_t, grasp_hold)) in enumerate(zip(sequence_forward, grasps_sequence_forward)):
        grasp_move_final = grasps_move_t[0]

        if i == 0:
            gripper_final_mat_hold = get_transform_matrix_quat(grasp_hold.pos, grasp_hold.quat)
            part_pickup_mat_hold = get_transform_matrix(pickup_pose[part_hold])
            gripper_pickup_mat_hold = part_pickup_mat_hold @ gripper_final_mat_hold
            gripper_hold_pickup_pos, gripper_hold_pickup_quat = mat_to_pos_quat(gripper_pickup_mat_hold)
            gripper_meshes_hold_tight = transform_gripper_meshes(gripper_type, gripper_meshes, gripper_hold_pickup_pos, gripper_hold_pickup_quat, np.eye(4), grasp_hold.open_ratio - 0.05)
            gripper_meshes_hold_loose = transform_gripper_meshes(gripper_type, gripper_meshes, gripper_hold_pickup_pos, gripper_hold_pickup_quat, np.eye(4), grasp_hold.open_ratio + 0.15)
            gripper_meshes_pickup[part_hold] = trimesh.boolean.union([get_swept_mesh(gripper_meshes_hold_tight[gripper_part], gripper_meshes_hold_loose[gripper_part]) for gripper_part in gripper_meshes_hold_tight.keys()])

        gripper_final_mat_move = get_transform_matrix_quat(grasp_move_final.pos, grasp_move_final.quat)
        part_pickup_mat_move = get_transform_matrix(pickup_pose[part_move])
        gripper_pickup_mat_move = part_pickup_mat_move @ gripper_final_mat_move
        gripper_move_pickup_pos, gripper_move_pickup_quat = mat_to_pos_quat(gripper_pickup_mat_move)
        gripper_meshes_move_tight = transform_gripper_meshes(gripper_type, gripper_meshes, gripper_move_pickup_pos, gripper_move_pickup_quat, np.eye(4), grasp_move_final.open_ratio - 0.05)
        gripper_meshes_move_loose = transform_gripper_meshes(gripper_type, gripper_meshes, gripper_move_pickup_pos, gripper_move_pickup_quat, np.eye(4), grasp_move_final.open_ratio + 0.15)
        gripper_meshes_pickup[part_move] = trimesh.boolean.union([get_swept_mesh(gripper_meshes_move_tight[gripper_part], gripper_meshes_move_loose[gripper_part]) for gripper_part in gripper_meshes_move_tight.keys()])

    return part_meshes_pickup, gripper_meshes_pickup


def generate_fixture(part_meshes_pickup, gripper_meshes_pickup, bin_size, min_fixture_y):

    # determine fixture height and part positions
    board_height_max = 0.0

    for part_id in part_meshes_pickup.keys():

        part_mesh = part_meshes_pickup[part_id]
        part_com = part_mesh.center_mass

        board_height = BOTTOM_THICKNESS + MIN_MOLD_DEPTH
        while True:
            part_sliced = part_mesh.slice_plane([0, 0, board_height], [0, 0, -1], cap=True)
            hole_hull = Delaunay(part_sliced.vertices[:, :2])
            if hole_hull.find_simplex(part_com[:2]) >= 0: # com inside hole hull
                break
            board_height += 1.0

        if board_height > board_height_max:
            board_height_max = board_height

    # verify bin size
    part_meshes_concat = trimesh.util.concatenate(list(part_meshes_pickup.values()))
    part_meshes_vertices_in_fixture = part_meshes_concat.vertices[part_meshes_concat.vertices[:, 2] < board_height_max]
    vertices_min, vertices_max = part_meshes_vertices_in_fixture.min(axis=0), part_meshes_vertices_in_fixture.max(axis=0)
    part_extent = vertices_max - vertices_min
    edge_gap = (np.array(bin_size) - part_extent[:2]) / 2
    assert np.all(edge_gap >= 0), 'Bin size is too small'

    # generate compact fixture mesh
    box_min, box_max = np.zeros(3), np.zeros(3)
    box_units = np.floor(part_extent[:2] / DX) + 1
    box_units = np.ceil(box_units / 2) * 2 # make it even
    box_extent = box_units * DX
    box_min = np.array([-box_extent[0] / 2, min_fixture_y, 0])
    box_max = np.array([box_extent[0] / 2, min_fixture_y + box_extent[1], board_height_max])
    board_mesh = trimesh.creation.box(bounds=[box_min, box_max])
    board_mesh_bottom = trimesh.creation.box(bounds=[box_min, [box_max[0], box_max[1], BOTTOM_THICKNESS]])

    # part translation
    box_center = (box_min + box_max) / 2.0
    part_center = (vertices_min + vertices_max) / 2.0
    part_translation = box_center - part_center
    part_translation[2] = 0.0

    # create convex hull for each part with swept volume
    part_meshes_swept = {}
    for part_id, part_mesh in part_meshes_pickup.items():
        part_mesh_swept_low = part_mesh.slice_plane([0, 0, board_height_max + 0.01], [0, 0, -1], cap=True)
        part_mesh_swept_high = part_mesh_swept_low.copy()
        part_mesh_swept_high.apply_translation([0, 0, board_height_max - BOTTOM_THICKNESS + 0.01])
        part_meshes_swept[part_id] = get_swept_mesh(part_mesh_swept_low, part_mesh_swept_high)

    # subtract parts from board, only keep part area
    part_boxes = []
    for part_id, part_mesh in part_meshes_swept.items():
        part_mesh_buffered = get_buffered_meshes(part_mesh, np.array(MOLD_EDGE_OFFSET_PART) / 2)
        part_mesh_buffered.apply_translation(part_translation)
        board_mesh = trimesh.boolean.difference([board_mesh, part_mesh_buffered])

        part_vertices = part_mesh_buffered.vertices
        part_min, part_max = part_vertices.min(axis=0), part_vertices.max(axis=0)
        part_min -= PART_BOUNDARY_OFFSET
        part_max += PART_BOUNDARY_OFFSET
        part_min[2] = 0.0 - 1e-2
        part_max[2] = part_meshes_pickup[part_id].center_mass[2] + 0.5
        part_box = trimesh.creation.box(bounds=[part_min, part_max])
        part_boxes.append(part_box)

        if gripper_meshes_pickup[part_id].vertices.min(axis=0)[2] < board_height_max:
            gripper_hull_pickup = gripper_meshes_pickup[part_id].slice_plane([0, 0, board_height_max + 0.01], [0, 0, -1], cap=True).convex_hull
            gripper_hull_pickup_buffered = get_buffered_meshes(gripper_hull_pickup, np.array(MOLD_EDGE_OFFSET_GRIPPER) / 2)
            gripper_hull_pickup_buffered.apply_translation(part_translation)
            board_mesh = trimesh.boolean.difference([board_mesh, gripper_hull_pickup_buffered])

    part_boxes = trimesh.boolean.union(part_boxes)
    board_mesh = trimesh.boolean.intersection([board_mesh, part_boxes])
    board_mesh = trimesh.boolean.union([board_mesh, board_mesh_bottom])

    return board_mesh, part_translation


def check_part_gripper_collision(part_meshes_pickup, gripper_meshes_pickup, sequence):

    part_disassembly_sequence = [part_move for part_move, _ in sequence] + [sequence[-1][1]]
    parts_to_buffer = []
    col_manager = trimesh.collision.CollisionManager()
    for part_id in part_disassembly_sequence:
        if col_manager.in_collision_single(gripper_meshes_pickup[part_id]):
            parts_to_buffer.append(part_id)
        col_manager.add_object(part_id, part_meshes_pickup[part_id])

    return parts_to_buffer


def add_countersunk_pads_to_fixture(fixture_mesh, min_fixture_y):
    bin_size = fixture_mesh.extents[:2]
    pad_lower_x, pad_upper_x = -bin_size[0] / 2 - DX / 2, bin_size[0] / 2 + DX / 2
    pad_lower_y, pad_upper_y = min_fixture_y + DX / 2, min(min_fixture_y + bin_size[1] - DX / 2, min_fixture_y + MAX_BIN_SIZE_DOUBLE[1] // 2 + DX / 2)
    pad_centers = [(pad_lower_x, pad_lower_y), (pad_upper_x, pad_lower_y), (pad_lower_x, pad_upper_y), (pad_upper_x, pad_upper_y)]
    pad_meshes = []
    for pad_center in pad_centers:
        pad_mesh = generate_countersunk_pad()
        pad_mesh.apply_translation([pad_center[0], pad_center[1], 0.0])
        pad_meshes.append(pad_mesh)
    fixture_mesh = trimesh.boolean.union([fixture_mesh] + pad_meshes)
    return fixture_mesh, pad_centers


def widen_fixture_rim_for_markers(fixture_mesh, extend=None):
    """Union a plain slab-thick band onto the -X / +X edges so the ArUco perimeter strips
    have room. In 'corners' mode the four countersunk pads widened the footprint by
    ``DX/2 + PAD_DIAMETER/2`` per side as a side effect; 'ears' mode has no pads, so do it
    explicitly here (same amount -> identical marker strip width). No holes, slab layer
    only -> pockets / Z height / plan untouched."""
    if extend is None:
        extend = DX / 2 + PAD_DIAMETER / 2
    b = fixture_mesh.bounds
    bands = [
        trimesh.creation.box(bounds=[[b[0][0] - extend, b[0][1], 0.0],
                                     [b[0][0] + 1e-3, b[1][1], BOTTOM_THICKNESS]]),
        trimesh.creation.box(bounds=[[b[1][0] - 1e-3, b[0][1], 0.0],
                                     [b[1][0] + extend, b[1][1], BOTTOM_THICKNESS]]),
    ]
    return trimesh.boolean.union([fixture_mesh] + bands, engine='manifold', check_volume=False)


def _slab_hole_lattice(bounds):
    """The SCREW_PITCH lattice of candidate hole centres, centred in the slab footprint and
    symmetric, with a half-pitch margin to the edges. Returns (xs, ys)."""
    (x_lo, y_lo), (x_hi, y_hi) = bounds[0][:2], bounds[1][:2]

    def ticks(lo, hi):
        n = int((hi - lo - SCREW_PITCH) // SCREW_PITCH) + 1
        start = 0.5 * (lo + hi) - 0.5 * (n - 1) * SCREW_PITCH
        return [start + k * SCREW_PITCH for k in range(max(n, 1))]

    return ticks(x_lo, x_hi), ticks(y_lo, y_hi)


def slab_hole_corners(fixture_mesh):
    """The four outer lattice points — the reliable clamp positions. Passed to the ArUco
    step as keep-outs before the marker rows are placed."""
    xs, ys = _slab_hole_lattice(fixture_mesh.bounds)
    return [(xs[0], ys[0]), (xs[-1], ys[0]), (xs[0], ys[-1]), (xs[-1], ys[-1])]


def plan_slab_screw_holes(fixture_mesh, markers_meta, slab_top_z):
    """SCREW_PITCH-grid countersunk-hole centres drilled into the solid bottom slab, kept
    ``HOLE_EDGE_INSET`` from the slab edge, ``HOLE_ISLAND_CLR`` from any carved pocket and
    ``HOLE_MARKER_CLR`` from any ArUco tile. The four outer points (corners) are forced in
    even if a clearance is marginally violated; the rest are opportunistic."""
    from shapely.geometry import box as _box, Point
    from shapely.ops import unary_union

    slab = _section_polygon(fixture_mesh, slab_top_z - 0.10)
    island = _section_polygon(fixture_mesh, slab_top_z + 0.15)
    if slab is None:
        return []
    free = slab.buffer(-HOLE_EDGE_INSET)
    if island is not None:
        free = free.difference(island.buffer(HOLE_ISLAND_CLR))
    tiles = [_box(min(c[0] for c in mk['corners']), min(c[1] for c in mk['corners']),
                  max(c[0] for c in mk['corners']), max(c[1] for c in mk['corners']))
             for mk in (markers_meta or {}).get('markers', [])]
    if tiles:
        free = free.difference(unary_union(tiles).buffer(HOLE_MARKER_CLR))

    xs, ys = _slab_hole_lattice(fixture_mesh.bounds)
    corners = {(xs[0], ys[0]), (xs[-1], ys[0]), (xs[0], ys[-1]), (xs[-1], ys[-1])}
    r = COUNTERSUNK_DIAMETER / 2
    keep = []
    for x in xs:
        for y in ys:
            if (x, y) in corners or free.contains(Point(x, y).buffer(r)):
                keep.append((x, y))
    return keep


def drill_slab_screw_holes(fixture_mesh, body_mesh, centers, slab_top_z, verbose=True):
    """Drill countersunk through-holes at ``centers`` into both meshes (identical -> the
    body+markers volume split is preserved). Called after the markers, before the lightening
    (so the lightening keeps a disc of slab around each). Envelope unchanged."""
    if not centers:
        if verbose:
            print('[slab_screws] no grid point cleared the pockets / markers')
        return fixture_mesh, body_mesh
    holes = []
    for x, y in centers:
        h = generate_countersunk_hole(COUNTERSUNK_DIAMETER, HOLE_DIAMETER, slab_top_z)
        h.apply_translation([x, y, 0.0])
        holes.append(h)

    def _drill(mesh):
        for h in holes:
            mesh = trimesh.boolean.difference([mesh, h], engine='manifold', check_volume=False)
        return mesh

    fixture_out = _drill(fixture_mesh)
    body_out = fixture_out if body_mesh is fixture_mesh else _drill(body_mesh)
    if verbose:
        print(f'[slab_screws] {len(centers)} countersunk holes @ {SCREW_PITCH:g} cm pitch: '
              f'{[(round(x, 1), round(y, 1)) for x, y in centers]}')
    return fixture_out, body_out


def _ear_hole_layout(bounds, slab_top_z):
    """Screw-hole centres (native XY) for the mounting ears, plus the ear tabs that carry
    them. Holes sit ``EAR_STICKOUT`` past the -X, +X and +Y body edges (never the -Y,
    robot-facing edge); neighbours on an edge are ``SCREW_PITCH`` apart, centred on the
    edge. Returns ``(hole_centers, tab_meshes)``."""
    (x_lo, y_lo, _), (x_hi, y_hi, _) = bounds[0], bounds[1]

    def _spread(center, half_span):
        n = int((half_span - EAR_MARGIN) // SCREW_PITCH)
        return [center + k * SCREW_PITCH for k in range(-n, n + 1)]

    edges = [  # (axis of the row, fixed coord of the hole, list of the free coord)
        ('y', x_lo - EAR_STICKOUT, _spread(0.5 * (y_lo + y_hi), 0.5 * (y_hi - y_lo))),  # -X
        ('y', x_hi + EAR_STICKOUT, _spread(0.5 * (y_lo + y_hi), 0.5 * (y_hi - y_lo))),  # +X
        ('x', y_hi + EAR_STICKOUT, _spread(0.5 * (x_lo + x_hi), 0.5 * (x_hi - x_lo))),  # +Y
    ]
    hole_centers, tabs = [], []
    for axis, fixed, frees in edges:
        for f in frees:
            hx, hy = (f, fixed) if axis == 'x' else (fixed, f)
            hole_centers.append((hx, hy))
            if axis == 'x':      # +Y ear: tab bridges y_hi -> past the hole
                lo, hi = [hx - EAR_HALF_W, y_hi - 0.3], [hx + EAR_HALF_W, hy + EAR_HALF_W]
            elif fixed < 0:      # -X ear
                lo, hi = [hx - EAR_HALF_W, hy - EAR_HALF_W], [x_lo + 0.3, hy + EAR_HALF_W]
            else:                # +X ear
                lo, hi = [x_hi - 0.3, hy - EAR_HALF_W], [hx + EAR_HALF_W, hy + EAR_HALF_W]
            tabs.append(trimesh.creation.box(bounds=[[lo[0], lo[1], 0.0],
                                                     [hi[0], hi[1], slab_top_z]]))
    return hole_centers, tabs


def add_mounting_ears_to_fixture(fixture_mesh, body_mesh, slab_top_z, verbose=True):
    """Fuse the mounting-ear tabs onto the fixture and drill their countersunk holes.

    Applied last (after the bottom-plate lightening) so nothing downstream re-analyses the
    slab. The ears grow the XY envelope outward on the -X / +X / +Y edges only; the mold
    islands, pockets, ArUco markers and pickup poses are all untouched. ``fixture_mesh``
    and ``body_mesh`` get the identical tabs + holes so the body+markers volume split holds.
    """
    hole_centers, tabs = _ear_hole_layout(fixture_mesh.bounds, slab_top_z)
    holes = []
    for hx, hy in hole_centers:
        h = generate_countersunk_hole(COUNTERSUNK_DIAMETER, HOLE_DIAMETER, slab_top_z)
        h.apply_translation([hx, hy, 0.0])
        holes.append(h)

    def _apply(mesh):
        mesh = trimesh.boolean.union([mesh] + tabs, engine='manifold', check_volume=False)
        for h in holes:
            mesh = trimesh.boolean.difference([mesh, h], engine='manifold', check_volume=False)
        return mesh

    fixture_out = _apply(fixture_mesh)
    body_out = fixture_out if body_mesh is fixture_mesh else _apply(body_mesh)
    assert fixture_out.body_count == 1 and body_out.body_count == 1, \
        f'mounting ears split the fixture ({fixture_out.body_count} / {body_out.body_count} bodies)'
    if verbose:
        print(f'[mounting_ears] {len(hole_centers)} screw holes on ears; '
              f'envelope {np.round(fixture_mesh.extents, 1).tolist()} -> '
              f'{np.round(fixture_out.extents, 1).tolist()} cm')
    return fixture_out, body_out


def lighten_fixture_bottom(fixture_mesh, body_mesh, markers_meta, slab_top_z,
                           pad_centers=(), min_cutout_area=None, verbose=True):
    """Carve the dead flat plate out of the ``z <= slab_top_z`` slab layer.

    The generated fixture has a solid full-footprint bottom slab; for a typical assembly
    that slab is ~45 % of the whole part and almost all of it is connective plate that
    carries nothing. This intersects *only* the slab layer with a keep mask made of a
    perimeter rail + a flange under every carved pocket + a bed under every ArUco marker +
    a disc around every countersunk pad + cross ribs that tie them together. Everything at
    ``z > slab_top_z`` (the mold islands) and the fixture bounding box are untouched, so
    the plan, the pickup poses, the marker layout and the mounting holes are unaffected.

    ``min_cutout_area`` (cm^2, default ``MIN_CUTOUT_AREA``): windows smaller than this are
    left solid instead of opened, so the print does not fill up with tiny holes.

    ``fixture_mesh`` and ``body_mesh`` are lightened with the identical mask so the
    ``vol(body) + vol(markers) == vol(fixture)`` split still holds. Returns them lightened.
    """
    if min_cutout_area is None:
        min_cutout_area = MIN_CUTOUT_AREA
    from shapely.geometry import box as _box, LineString, Point
    from shapely.ops import unary_union, nearest_points

    b0 = fixture_mesh.bounds.copy()
    slab = _section_polygon(fixture_mesh, slab_top_z - 0.10)
    island = _section_polygon(fixture_mesh, slab_top_z + 0.15)
    if slab is None or island is None:
        if verbose:
            print('[lighten_bottom] could not slice slab/island; leaving the bottom solid')
        return fixture_mesh, body_mesh

    sx_lo, sy_lo, sx_hi, sy_hi = slab.bounds
    keep = [island.buffer(FLOOR_POCKET_FLANGE)]                 # bed under every pocket
    if FLOOR_FRAME_W > 0:
        keep.append(slab.difference(slab.buffer(-FLOOR_FRAME_W)))   # perimeter rail

    # bed under every ArUco marker (native-frame corners straight from markers.json payload)
    for mk in (markers_meta or {}).get('markers', []):
        xs = [c[0] for c in mk['corners']]
        ys = [c[1] for c in mk['corners']]
        keep.append(_box(min(xs), min(ys), max(xs), max(ys)).buffer(FLOOR_MARKER_MARGIN))

    # cross ribs through the island centroid, spanning the whole footprint -> every bed is
    # tied to the rail and the result stays a single connected body
    icx, icy = island.centroid.x, island.centroid.y
    keep.append(LineString([(sx_lo, icy), (sx_hi, icy)]).buffer(FLOOR_RIB_W / 2))
    keep.append(LineString([(icx, sy_lo), (icx, sy_hi)]).buffer(FLOOR_RIB_W / 2))

    # a solid boss around every in-slab screw hole + the shortest rib tying it into whatever
    # is already kept, so no hole boss floats after the cut (empty when mounting on ears)
    anchor = unary_union([g for g in keep if not g.is_empty])
    for pcx, pcy in pad_centers:
        keep.append(Point(pcx, pcy).buffer(PAD_DIAMETER / 2 + FLOOR_PAD_MARGIN))
        tie = nearest_points(Point(pcx, pcy), anchor)[1]
        keep.append(LineString([(pcx, pcy), (tie.x, tie.y)]).buffer(FLOOR_RIB_W / 2))

    keep_poly = unary_union([g for g in keep if not g.is_empty])

    # leave small windows solid -- lots of tiny holes only cost print time
    if min_cutout_area and min_cutout_area > 0:
        removed = slab.difference(keep_poly)
        smalls = [g for g in (removed.geoms if removed.geom_type == 'MultiPolygon' else [removed])
                  if (not g.is_empty) and g.area < min_cutout_area]
        if smalls:
            keep_poly = unary_union([keep_poly, *smalls])
            if verbose:
                print(f'[lighten_bottom] kept {len(smalls)} sub-{min_cutout_area:g} cm^2 window(s) solid')

    # guarantee one connected body: weld every stray keep-region to the largest with a rib
    if keep_poly.geom_type == 'MultiPolygon':
        parts = sorted(keep_poly.geoms, key=lambda g: g.area, reverse=True)
        main = parts[0]
        for g in parts[1:]:
            p1, p2 = nearest_points(g, main)
            main = unary_union([main, g, LineString([p1, p2]).buffer(FLOOR_RIB_W / 2)])
        keep_poly = main
        if verbose and len(parts) > 1:
            print(f'[lighten_bottom] welded {len(parts) - 1} stray keep-region(s)')

    geoms = list(keep_poly.geoms) if keep_poly.geom_type == 'MultiPolygon' else [keep_poly]
    mask = trimesh.boolean.union(
        [trimesh.creation.extrude_polygon(g, height=slab_top_z) for g in geoms] +
        [trimesh.creation.box(bounds=[[b0[0][0], b0[0][1], slab_top_z],
                                      [b0[1][0], b0[1][1], b0[1][2] + 0.1]])],
        engine='manifold', check_volume=False)

    out = []
    for mesh in (fixture_mesh, body_mesh):
        out.append(trimesh.boolean.intersection([mesh, mask], engine='manifold', check_volume=False))
    fixture_light, body_light = out

    assert np.allclose(fixture_light.bounds, b0, atol=1e-3), \
        f'bottom lightening moved the fixture bbox: {b0.tolist()} -> {fixture_light.bounds.tolist()}'
    assert fixture_light.body_count == 1 and body_light.body_count == 1, \
        f'bottom lightening split the fixture ({fixture_light.body_count} / {body_light.body_count} bodies)'
    if verbose:
        print(f'[lighten_bottom] fixture volume {fixture_mesh.volume:.1f} -> {fixture_light.volume:.1f} cm^3 '
              f'({100 * (1 - fixture_light.volume / fixture_mesh.volume):.0f}% lighter)')
    return fixture_light, body_light


def render_marker_preview(body_mesh, markers_mesh, out_path):
    """Near-top-down preview: light body, dark flush ArUco inlays."""
    body = body_mesh.copy()
    body.visual.face_colors = [225, 225, 225, 255]
    mk = markers_mesh.copy()
    mk.visual.face_colors = [10, 10, 10, 255]
    scene = trimesh.Scene([body, mk])
    scene.set_camera(angles=[np.deg2rad(40), 0, 0], center=body.centroid,
                     distance=body.scale * 1.15)
    with open(out_path, 'wb') as fp:
        fp.write(scene.save_image(resolution=(1600, 1230), visible=False))


def run_fixture_gen(assembly_dir, log_dir, optimized, seed, render=False, markers='aruco',
                    min_cutout_area=None):
    import pyglet
    pyglet.options["headless"] = not render

    precedence_path = os.path.join(log_dir, 'precedence.pkl')
    if not os.path.exists(precedence_path):
        print(f'[run_fixture_gen] {precedence_path} not found')
        return
    grasps_path = os.path.join(log_dir, 'grasps.pkl')
    if not os.path.exists(grasps_path):
        print(f'[run_fixture_gen] {grasps_path} not found')
        return

    with open(precedence_path, 'rb') as fp:
        G_preced = pickle.load(fp)
    with open(grasps_path, 'rb') as fp:
        grasps = pickle.load(fp)
    arm_type = grasps['arm']

    tree_path = os.path.join(log_dir, 'tree_opt.pkl') if optimized else os.path.join(log_dir, 'tree.pkl')
    if not os.path.exists(tree_path):
        print(f'[run_fixture_gen] {tree_path} not found')
        return
    with open(tree_path, 'rb') as fp:
        tree = pickle.load(fp)

    asset_folder = os.path.join(project_base_dir, './assets')
    if optimized:
        seq_optimizer = SequenceOptimizer(G_preced, grasps)
        sequence, grasps_sequence = seq_optimizer.get_sequence(tree)
    else:
        seq_planner = SequencePlanner(asset_folder, assembly_dir, G_preced, grasps, save_sdf=True, contact_eps=None)
        sequence, grasps_sequence = seq_planner.sample_sequence(tree, seed=seed)

    if sequence is None or grasps_sequence is None:
        print(f'[run_fixture_gen] No feasible sequence found in {tree_path}')
        return

    # get part meshes
    part_meshes_final = load_part_meshes(assembly_dir, transform='final')
    part_meshes_final = {k.replace('part', ''): v for k, v in part_meshes_final.items()}
    for part_id, part_mesh in part_meshes_final.items():
        part_mesh.apply_translation(get_assembly_center(arm_type))
    part_pos_dict_final, part_quat_dict_final = load_pos_quat_dict(assembly_dir, transform='final')
    part_pos_dict_final = {part_id: part_pos_dict_final[part_id] + get_assembly_center(arm_type) for part_id in part_meshes_final.keys()}
    part_cfg_final = {'mesh': part_meshes_final, 'pos': part_pos_dict_final, 'quat': part_quat_dict_final}

    min_fixture_y = get_fixture_min_y(arm_type)

    t_start = time()

    # get part orientation from grasps
    pose_info_individual = generate_individual_pose_info(part_cfg_final, sequence, grasps_sequence, grasps['gripper'])

    bin_size = None
    while True: # 2d packing and make sure collision free with gripper

        # get pickup pose (relative to final pose)
        pose_pickup, bin_size = generate_pickup_pose(pose_info_individual, min_fixture_y, render=render)
        if bin_size is None:
            print(f'[run_fixture_gen] Bin size exceeds maximum size')
            return

        # get pickup meshes
        part_meshes_pickup, gripper_meshes_pickup = generate_pickup_meshes(part_cfg_final, sequence, grasps_sequence, grasps['gripper'], pose_pickup)

        # check part-gripper collision
        parts_to_buffer = check_part_gripper_collision(part_meshes_pickup, gripper_meshes_pickup, sequence)
        if len(parts_to_buffer) == 0:
            break
        else: # if collision, buffer parts
            for part_id in parts_to_buffer:
                pose_info_individual[part_id]['extent_x'] += DELTA_BUFFER_SIZE

    # generate fixture by subtracting part and gripper meshes
    fixture_mesh, part_translation = generate_fixture(part_meshes_pickup, gripper_meshes_pickup, bin_size, min_fixture_y)
    for part_id, part_mesh in part_meshes_pickup.items():
        part_mesh.apply_translation(part_translation)

    # add the mounting screw holes: 'corners' -> in-slab pads now; 'slab' -> grid holes
    # after the markers; 'ears' -> tabs added last (all below).
    pad_centers = ()
    if MOUNT_STYLE == 'corners':
        fixture_mesh, pad_centers = add_countersunk_pads_to_fixture(fixture_mesh, min_fixture_y)
    elif markers != 'none':
        fixture_mesh = widen_fixture_rim_for_markers(fixture_mesh)
    if MOUNT_STYLE == 'slab':
        pad_centers = slab_hole_corners(fixture_mesh)  # keep the marker rows off the corners

    # add flush multi-material ArUco markers to the fixture perimeter (does not touch the plan:
    # markers are recessed into the existing slab, fixture envelope + pickup poses unchanged)
    body_mesh, markers_mesh, markers_meta = fixture_mesh, None, None
    if markers != 'none':
        pickup_before = json.dumps(pose_pickup, sort_keys=True)
        fixture_mesh, body_mesh, markers_mesh, markers_meta = add_aruco_markers_to_fixture(
            fixture_mesh, min_fixture_y, BOTTOM_THICKNESS, pad_centers=pad_centers)
        assert json.dumps(pose_pickup, sort_keys=True) == pickup_before, 'markers mutated pickup poses'

    # 'slab': drill the 5 cm-grid screw holes into the still-solid slab, before the lightening
    if MOUNT_STYLE == 'slab':
        pad_centers = plan_slab_screw_holes(fixture_mesh, markers_meta, BOTTOM_THICKNESS)
        fixture_mesh, body_mesh = drill_slab_screw_holes(
            fixture_mesh, body_mesh, pad_centers, BOTTOM_THICKNESS)
        if markers_mesh is not None:
            assert abs((body_mesh.volume + markers_mesh.volume) - fixture_mesh.volume) < 1e-2, \
                'slab screw holes broke the body + markers volume split'

    # strip the dead flat plate out of the bottom slab (mold islands / bbox / plan untouched)
    if LIGHTEN_BOTTOM:
        fixture_mesh, body_mesh = lighten_fixture_bottom(
            fixture_mesh, body_mesh, markers_meta, BOTTOM_THICKNESS,
            pad_centers=pad_centers, min_cutout_area=min_cutout_area)
        if markers_mesh is None:
            body_mesh = fixture_mesh
        else:
            assert abs((body_mesh.volume + markers_mesh.volume) - fixture_mesh.volume) < 1e-2, \
                'bottom lightening broke the body + markers volume split'

    # mounting ears: fused on last so nothing downstream re-analyses the slab. Grows the
    # XY envelope outward on the -X / +X / +Y edges only (pockets / markers / plan intact).
    if MOUNT_STYLE == 'ears':
        fixture_mesh, body_mesh = add_mounting_ears_to_fixture(
            fixture_mesh, body_mesh, BOTTOM_THICKNESS)
        if markers_mesh is None:
            body_mesh = fixture_mesh
        else:
            assert abs((body_mesh.volume + markers_mesh.volume) - fixture_mesh.volume) < 1e-2, \
                'mounting ears broke the body + markers volume split'

    scene = trimesh.Scene([fixture_mesh] + list(part_meshes_pickup.values()))
    if render:
        scene.show()

    # transform pickup pose from relative-to-final to global
    pose_pickup_global = {}
    for part_id, part_pose in pose_pickup.items():
        part_pose_global = get_transform_matrix(part_pose) @ get_transform_matrix_quat(part_cfg_final['pos'][part_id], part_cfg_final['quat'][part_id])
        part_pose_global[:3, 3] += part_translation
        pose_pickup_global[part_id] = get_pos_euler_from_transform_matrix(part_pose_global).tolist()

    fixture_dir = os.path.join(log_dir, 'fixture')
    os.makedirs(fixture_dir, exist_ok=True)
    with open(os.path.join(fixture_dir, 'pickup.json'), 'w') as fp:
        json.dump(pose_pickup_global, fp)
    fixture_mesh.export(os.path.join(fixture_dir, 'fixture.obj'))
    with open(os.path.join(fixture_dir, 'fixture.png'), 'wb') as fp:
        fp.write(scene.save_image(visible=False))

    # multi-material print + perception artifacts
    if markers_meta is not None:
        with open(os.path.join(fixture_dir, 'markers.json'), 'w') as fp:
            json.dump(markers_meta, fp, indent=2)
    if markers_mesh is not None:
        body_mm = body_mesh.copy();    body_mm.apply_scale(10.0)
        mk_mm = markers_mesh.copy();   mk_mm.apply_scale(10.0)
        body_mm.export(os.path.join(fixture_dir, 'fixture_body.stl'))
        mk_mm.export(os.path.join(fixture_dir, 'fixture_markers.stl'))
        try:  # single-file two-colour object; optional (needs a 3mf writer)
            b, m = body_mm.copy(), mk_mm.copy()
            b.visual.face_colors = [210, 210, 210, 255]
            m.visual.face_colors = [15, 15, 15, 255]
            trimesh.Scene({'fixture_body': b, 'fixture_markers': m}).export(
                os.path.join(fixture_dir, 'fixture.3mf'))
        except Exception as e:
            print(f'[run_fixture_gen] 3mf export skipped: {e}')
        render_marker_preview(body_mesh, markers_mesh,
                              os.path.join(fixture_dir, 'fixture_markers.png'))


    stats_path = os.path.join(log_dir, 'stats.json')
    with open(stats_path, 'r') as fp:
        stats = json.load(fp)
    stats['fixture_gen'] = {'time': round(time() - t_start, 2)}
    with open(stats_path, 'w') as fp:
        json.dump(stats, fp)


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('--assembly-dir', type=str, required=True)
    parser.add_argument('--log-dir', type=str, required=True)
    parser.add_argument('--optimized', default=False, action='store_true')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--render', default=False, action='store_true')
    parser.add_argument('--markers', choices=['none', 'aruco'], default='aruco',
                        help="'aruco': inlay a flush multi-material ArUco board into the fixture "
                             "perimeter (default). 'none': legacy behaviour.")
    parser.add_argument('--min-cutout-area', type=float, default=None,
                        help="bottom-plate lightening: only open a slab window whose footprint "
                             f"exceeds this many cm^2 (default {MIN_CUTOUT_AREA:g}); smaller "
                             "cut-outs stay solid so the print isn't full of tiny holes. "
                             "0 = open every window.")
    args = parser.parse_args()

    run_fixture_gen(args.assembly_dir, args.log_dir, args.optimized, args.seed,
                    render=args.render, markers=args.markers,
                    min_cutout_area=args.min_cutout_area)
