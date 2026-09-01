"""
Headless trimesh renderer for a motion plan, driven by logs/<run>/traj.npy.

Why this exists: rendering/render_motion_plan.py (redmax) works headless but its
camera constants in get_camera_option() are metre-scale while the Fabrica planning
scene is centimetre-scale, and the redmax viewer auto-orbits -- so framing a clean
still camera is impractical. This script reads traj.npy (a per-sim-frame dict of
{body_name: 4x4 world transform}) written by render_motion_plan.py, loads each
body's mesh once, applies the per-frame transform, and rasterises with a fixed
camera via trimesh + pyglet(EGL). Output is a folder of PNG frames; stitch with
ffmpeg.

Usage:
    python planning/utils/render_traj_trimesh_headless.py \
        --log-dir logs/plumbers_block_sim --out /tmp/frames --step 28
    ffmpeg -y -framerate 20 -i /tmp/frames/%04d.png -c:v libx264 -pix_fmt yuv420p plan.mp4
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import pyglet
pyglet.options['headless'] = True
import argparse
import numpy as np
import trimesh

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

PART_COLORS = {'0': [70, 140, 220, 255], '1': [240, 205, 55, 255], '2': [240, 150, 45, 255],
               '3': [90, 200, 100, 255], '4': [210, 80, 100, 255]}


def _load(path, color):
    m = trimesh.load(path, force='mesh', process=False)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(color, (len(m.faces), 1)))
    return m


def _look_at(eye, target, up=np.array([0, 0, 1.0])):
    f = target - eye; f /= np.linalg.norm(f)
    s = np.cross(f, up); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    M = np.eye(4)
    M[:3, 0] = s; M[:3, 1] = u; M[:3, 2] = -f; M[:3, 3] = eye
    return M


def build_base_meshes(log_dir, part_ids, arm_suffixes=('move', 'hold')):
    base = {}
    for pid in part_ids:
        # assembly part meshes: assets/fabrica/<assembly>/<pid>.obj -- resolved from the fixture run
        for cand in (os.path.join(PROJECT_ROOT, 'assets/fabrica/plumbers_block', f'{pid}.obj'),):
            if os.path.exists(cand):
                base[f'part{pid}'] = _load(cand, PART_COLORS.get(pid, [180, 180, 180, 255]))
    base['fixture'] = _load(os.path.join(log_dir, 'fixture/fixture.obj'), [120, 122, 128, 255])
    for suf in arm_suffixes:
        for k in range(8):
            p = os.path.join(PROJECT_ROOT, 'assets/kuka/collision', f'link{k}.obj')
            if os.path.exists(p):
                base[f'kuka_link{k}_{suf}'] = _load(p, [230, 230, 233, 255])
        for name, fn, col in (('pdz_base', 'base.obj', [80, 80, 88, 255]),
                              ('pdz_left_finger', 'finger_left.obj', [35, 35, 40, 255]),
                              ('pdz_right_finger', 'finger_right.obj', [35, 35, 40, 255])):
            p = os.path.join(PROJECT_ROOT, 'assets/pdz/collision', fn)
            if os.path.exists(p):
                base[f'{name}_{suf}'] = _load(p, col)
    return base


def render(log_dir, out_dir, step=28, res=(1000, 780), fov_deg=42,
           cam_dir=(0.55, -1.0, 0.5), cam_dist_mult=2.15):
    os.makedirs(out_dir, exist_ok=True)
    for f in os.listdir(out_dir):
        os.remove(os.path.join(out_dir, f))

    traj = np.load(os.path.join(log_dir, 'traj.npy'), allow_pickle=True)
    body_names = set()
    for fr in traj:
        body_names.update(fr.keys())
    part_ids = sorted(n[4:] for n in body_names if n.startswith('part'))
    base = build_base_meshes(log_dir, part_ids)

    lo = np.full(3, 1e9); hi = np.full(3, -1e9)
    for fr in traj:
        for name, T in fr.items():
            m = base.get(name)
            if m is None:
                continue
            T = np.asarray(T)
            v = (T[:3, :3] @ m.vertices.T).T + T[:3, 3]
            lo = np.minimum(lo, v.min(0)); hi = np.maximum(hi, v.max(0))
    center = (lo + hi) / 2
    radius = np.linalg.norm(hi - lo) / 2
    d = np.asarray(cam_dir, float); d /= np.linalg.norm(d)
    cam_T = _look_at(center + d * radius * cam_dist_mult,
                     center - np.array([0, 0, radius * 0.12]))

    idx = list(range(0, len(traj), step))
    fix_static = base['fixture'].copy()
    for fi, frame_i in enumerate(idx):
        fr = traj[frame_i]
        geoms = [fix_static]
        for name, T in fr.items():
            m = base.get(name)
            if m is None or name == 'fixture':
                continue
            g = m.copy(); g.apply_transform(np.asarray(T)); geoms.append(g)
        sc = trimesh.Scene(geoms)
        sc.camera.resolution = res
        sc.camera.fov = (fov_deg, fov_deg * res[1] / res[0])
        sc.camera_transform = cam_T
        with open(os.path.join(out_dir, f'{fi:04d}.png'), 'wb') as fp:
            fp.write(sc.save_image(resolution=res, visible=False))
    print(f'wrote {len(idx)} frames to {out_dir} (scene centre {np.round(center,1)}, radius {radius:.1f})')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--log-dir', required=True)
    ap.add_argument('--out', required=True, help='output frame directory')
    ap.add_argument('--step', type=int, default=28, help='render every Nth sim frame in traj.npy')
    args = ap.parse_args()
    render(args.log_dir, args.out, step=args.step)
