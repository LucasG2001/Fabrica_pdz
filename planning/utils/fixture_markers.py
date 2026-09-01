"""Flush multi-material ArUco markers for the printed pickup fixture.

Adds a small ArUco *board* (several coplanar markers at known relative poses) inlaid
FLUSH into the top of the fixture's bottom slab, in the exposed perimeter strips on
+/-X that are left over after the part/gripper reliefs are carved into the mold.

Design (see docs/fixture_generation_handoff.md and the marker plan):
  * Markers never protrude and never change the fixture's XY/Z envelope -> the assembly
    plan (sequence / grasps / pickup.json) is untouched. The black modules are recessed
    INLAY_DEPTH below the existing slab top; the body keeps the same outer surface.
  * Multi-material FDM: the fixture body prints in a light filament, the black module
    prisms in a dark one. Physically one monolithic part (modules are fully enclosed).
  * Layout is emitted to fixture/markers.json (dictionary + per-marker id/size/corners
    in the fixture *native* frame, i.e. the same cm frame as fixture.obj vertices).

Only DICT_4X4_50 is bundled (assets/aruco/dict_4x4_50.json, 6x6 incl 1-module border,
1=white / 0=black). cv2.aruco is NOT required at fixture-gen time.
"""

import os
import json
import numpy as np
import trimesh
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from planning.utils.fixture_countersunk import (
    generate_countersunk_hole, COUNTERSUNK_DIAMETER, HOLE_DIAMETER)

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_DIR = os.path.abspath(os.path.join(_THIS_DIR, '..', '..'))
DICT_PATH = os.path.join(_PROJECT_DIR, 'assets', 'aruco', 'dict_4x4_50.json')

# --- marker layout parameters (centimetres, Fabrica planning convention) ---
ARUCO_DICT = 'DICT_4X4_50'
MARKER_IDS_LEFT = [0, 1, 2]      # -X strip, ordered -Y -> +Y
MARKER_IDS_RIGHT = [3, 4, 5]     # +X strip, ordered -Y -> +Y
MARKER_SIZE_MAX = 3.0            # preferred black-square side (the OpenCV "marker"), cm
MARKER_SIZE_MIN = 2.0            # below this, detection at working distance is unreliable -> warn
QUIET_RATIO = 1.0 / 6.0         # white quiet zone each side, as a fraction of marker side (=1 module)
INLAY_DEPTH = 0.25              # black modules recessed this far below the slab top, cm
STRIP_MARGIN = 0.15            # min clearance from a marker tile to the strip edges, cm
ISLAND_CLEARANCE = 0.25        # keep tiles this far from the carved mold island, cm
PAD_KEEPOUT = 1.65             # keep tiles this far from a countersunk-pad centre, cm (pad r 1.25 + slack)
GRID = 6                       # 4x4 data + 1-module border ring


def load_marker_bits(dict_path=DICT_PATH):
    with open(dict_path) as fp:
        data = json.load(fp)
    return {int(k): np.asarray(v, dtype=int) for k, v in data['markers'].items()}, data['dictionary']


def _section_polygon(mesh, z):
    """Union of the closed XY polygons of ``mesh`` sliced at height ``z`` (world xy coords)."""
    path3d = mesh.section(plane_origin=[0.0, 0.0, z], plane_normal=[0.0, 0.0, 1.0])
    if path3d is None:
        return None
    path2d, _ = path3d.to_planar(to_2D=np.eye(4))
    polys = list(path2d.polygons_full)
    if not polys:
        return None
    return unary_union(polys)


def _marker_black_solid(bits, cx, cy, top_z, marker_size, inlay_depth):
    """Single watertight mesh of the black (0) region of the 6x6 grid, recessed ``inlay_depth``
    below ``top_z``. Row 0 -> +Y, col 0 -> -X. Adjacent black cells merge cleanly (shapely)."""
    module = marker_size / GRID
    x0 = cx - marker_size / 2.0
    y0 = cy - marker_size / 2.0
    squares = []
    for r in range(GRID):
        for c in range(GRID):
            if bits[r, c] != 0:
                continue  # white -> body material
            mx = x0 + c * module
            my = y0 + (GRID - 1 - r) * module
            squares.append(Polygon([(mx, my), (mx + module, my),
                                    (mx + module, my + module), (mx, my + module)]))
    region = unary_union(squares).buffer(1e-6).buffer(-1e-6)  # heal touching-edge slivers
    geoms = list(region.geoms) if region.geom_type == 'MultiPolygon' else [region]
    parts = []
    for g in geoms:
        solid = trimesh.creation.extrude_polygon(g, height=inlay_depth)
        solid.apply_translation([0.0, 0.0, top_z - inlay_depth])
        parts.append(solid)
    return trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]


def _marker_corners(cx, cy, top_z, marker_size):
    """OpenCV corner order (top-left, top-right, bottom-right, bottom-left) of the marker
    square (border included, quiet zone excluded), in the fixture native frame."""
    h = marker_size / 2.0
    return [
        [cx - h, cy + h, top_z],
        [cx + h, cy + h, top_z],
        [cx + h, cy - h, top_z],
        [cx - h, cy - h, top_z],
    ]


def _place_side(usable, cx, y_lo, y_hi, ids, tile, verbose):
    """Fit ``len(ids)`` marker tiles into one strip at fixed ``cx``; return [(id, cx, cy), ...]."""
    lo = y_lo + tile / 2.0 + STRIP_MARGIN
    hi = y_hi - tile / 2.0 - STRIP_MARGIN
    if hi <= lo:
        return []
    n = len(ids)
    cys = [0.5 * (lo + hi)] if n == 1 else list(np.linspace(lo, hi, n))
    placed = []
    half = tile / 2.0
    for mid, cy in zip(ids, cys):
        tile_poly = Polygon(
            [(cx - half, cy - half), (cx + half, cy - half),
             (cx + half, cy + half), (cx - half, cy + half)])
        if tile_poly.difference(usable).area < 1e-3:
            placed.append((mid, cx, cy))
        elif verbose:
            print(f'[fixture_markers]   id {mid}: tile at ({cx:.1f}, {cy:.1f}) not clear -> dropped')
    return placed


def add_aruco_markers_to_fixture(fixture_mesh, min_fixture_y, slab_top_z,
                                 pad_centers=(), verbose=True):
    """Inlay a flush ArUco board into the +/-X perimeter strips of ``fixture_mesh``.

    ``pad_centers`` is the list of countersunk mounting-pad centres (native XY), supplied
    by the caller so there is a single source of truth for where the screw holes are:
    the rim fill below buries them, so each one is re-drilled afterwards, and marker tiles
    are kept ``PAD_KEEPOUT`` clear of them. Empty when the fixture uses mounting ears
    instead of in-slab corner pads.

    Returns ``(fixture_mesh, body_mesh, markers_mesh, meta)`` where ``fixture_mesh`` is
    geometrically unchanged (flush inlay), ``body_mesh`` is it minus the black module
    prisms, ``markers_mesh`` is the union of those prisms, and ``meta`` is the
    markers.json payload. ``markers_mesh`` / ``meta['markers']`` are empty if nothing fits.
    """
    bits_by_id, dict_name = load_marker_bits()
    bounds0 = fixture_mesh.bounds.copy()

    slab_poly = _section_polygon(fixture_mesh, slab_top_z - 0.10)
    island_poly = _section_polygon(fixture_mesh, slab_top_z + 0.15)
    if slab_poly is None or island_poly is None:
        if verbose:
            print('[fixture_markers] could not slice slab/island; skipping markers')
        return fixture_mesh, fixture_mesh.copy(), None, {
            'dictionary': dict_name, 'units': 'cm', 'frame': 'fixture_native',
            'markers': [], 'board_ids': [], 'status': 'no_section'}

    ix_lo, iy_lo, ix_hi, iy_hi = island_poly.bounds
    sx_lo, sy_lo, sx_hi, sy_hi = slab_poly.bounds

    # The carved reliefs (part pockets + gripper clearance) all live inside the island in X, so
    # the +/-X perimeter of the slab layer (z in [0, slab_top]) is free real estate. Fill the
    # gaps the corner pads leave in that perimeter into a solid band, flush with the slab top and
    # only out to the existing bbox edge -> more material at the fixture rim, ZERO change to the
    # envelope, the pockets, or the plan.
    band_y0, band_y1 = min(sy_lo, iy_lo), max(sy_hi, iy_hi)
    fillers = [
        trimesh.creation.box(bounds=[[sx_lo, band_y0, 0.0], [ix_lo, band_y1, slab_top_z]]),
        trimesh.creation.box(bounds=[[ix_hi, band_y0, 0.0], [sx_hi, band_y1, slab_top_z]]),
    ]
    fixture_mesh = trimesh.boolean.union([fixture_mesh] + fillers, engine='manifold', check_volume=False)

    # The rim fill above just buried any in-slab countersunk-pad bolt holes under solid slab
    # (add_countersunk_pads_to_fixture drilled them, then this union filled them back in).
    # Re-drill them here so the printed fixture keeps its mounting-screw holes. Done on
    # fixture_mesh before body_mesh is split off -> both the reference solid and the print
    # body get the holes, and the body+markers volume split is preserved. No-op when the
    # fixture mounts on ears (pad_centers empty).
    for pcx, pcy in pad_centers:
        hole = generate_countersunk_hole(COUNTERSUNK_DIAMETER, HOLE_DIAMETER, slab_top_z)
        hole.apply_translation([pcx, pcy, 0.0])
        fixture_mesh = trimesh.boolean.difference(
            [fixture_mesh, hole], engine='manifold', check_volume=False)

    strip_w = min(ix_lo - sx_lo, sx_hi - ix_hi)
    marker_size = min(MARKER_SIZE_MAX,
                      (strip_w - 2 * STRIP_MARGIN - ISLAND_CLEARANCE) / (1.0 + 2 * QUIET_RATIO))
    tile = marker_size * (1.0 + 2 * QUIET_RATIO)
    status = 'ok'
    if marker_size < MARKER_SIZE_MIN:
        status = 'markers_small'
        if verbose:
            print(f'[fixture_markers] WARNING: usable strip only {strip_w:.2f} cm wide -> marker '
                  f'{marker_size:.2f} cm < {MARKER_SIZE_MIN} cm floor; consider a bolt-on plate.')
    if verbose:
        print(f'[fixture_markers] strip width {strip_w:.2f} cm -> marker {marker_size:.2f} cm, '
              f'tile {tile:.2f} cm, dict {dict_name}')

    # solid strips to place into, minus the island (with clearance) and the pad keep-outs
    usable = _section_polygon(fixture_mesh, slab_top_z - 0.10).difference(
        island_poly.buffer(ISLAND_CLEARANCE))
    for pc in pad_centers:
        usable = usable.difference(Point(pc).buffer(PAD_KEEPOUT))

    if pad_centers:  # keep the marker rows clear of the in-slab corner pads
        pad_ys = [c[1] for c in pad_centers]
        y_lo = max(iy_lo, min(pad_ys) + PAD_KEEPOUT)
        y_hi = min(iy_hi, max(pad_ys) - PAD_KEEPOUT)
    else:            # ears: no in-slab pads, run the markers the full island span
        y_lo, y_hi = iy_lo, iy_hi
    cx_left = sx_lo + STRIP_MARGIN + tile / 2.0
    cx_right = sx_hi - STRIP_MARGIN - tile / 2.0
    placed = _place_side(usable, cx_left, y_lo, y_hi, MARKER_IDS_LEFT, tile, verbose)
    placed += _place_side(usable, cx_right, y_lo, y_hi, MARKER_IDS_RIGHT, tile, verbose)

    prisms, markers_meta = [], []
    for mid, cx, cy in placed:
        prisms.append(_marker_black_solid(bits_by_id[mid], cx, cy, slab_top_z, marker_size, INLAY_DEPTH))
        markers_meta.append({
            'id': int(mid),
            'size': round(float(marker_size), 4),
            'center': [round(float(cx), 4), round(float(cy), 4), round(float(slab_top_z), 4)],
            'corners': [[round(float(v), 4) for v in c]
                        for c in _marker_corners(cx, cy, slab_top_z, marker_size)],
        })

    if not prisms:
        if verbose:
            print('[fixture_markers] no marker fit the available strips')
        return fixture_mesh, fixture_mesh.copy(), None, {
            'dictionary': dict_name, 'units': 'cm', 'frame': 'fixture_native',
            'marker_size': round(float(marker_size), 4), 'inlay_depth': INLAY_DEPTH,
            'markers': [], 'board_ids': [], 'status': 'no_fit'}

    markers_mesh = trimesh.util.concatenate(prisms) if len(prisms) > 1 else prisms[0]
    body_mesh = fixture_mesh
    for solid in prisms:  # per-marker: keeps every operand a clean single-body volume
        body_mesh = trimesh.boolean.difference([body_mesh, solid], engine='manifold', check_volume=False)

    # plan-safety invariant: filled rim + flush inlay must not move the fixture envelope
    assert np.allclose(body_mesh.bounds, bounds0, atol=1e-3), \
        f'marker inlay changed the fixture bounding box: {bounds0.tolist()} -> {body_mesh.bounds.tolist()}'

    meta = {
        'dictionary': dict_name,
        'units': 'cm',
        'frame': 'fixture_native',
        'frame_note': 'same frame/scale as fixture.obj vertices; see fabrica_fixture_pose_conventions',
        'marker_size': round(float(marker_size), 4),
        'quiet_zone': round(float(marker_size * QUIET_RATIO), 4),
        'inlay_depth': INLAY_DEPTH,
        'plane_z': round(float(slab_top_z), 4),
        'markers': markers_meta,
        'board_ids': [m['id'] for m in markers_meta],
        'status': status,
    }
    if verbose:
        print(f'[fixture_markers] placed {len(markers_meta)} markers: '
              f'{[m["id"] for m in markers_meta]}')
    return fixture_mesh, body_mesh, markers_mesh, meta
