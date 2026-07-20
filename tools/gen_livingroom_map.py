#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a world-aligned, robot-height occupancy map for living_room.

v2 — no box proxies. Occupancy comes from the geometry the robot can actually
collide with at its own height: every collision shape (box or mesh) is sliced
at the robot's body band (Z_MIN..Z_MAX) and rasterized. Open-under furniture
(the marble table: top at 0.40 m on legs) contributes only its legs, so the
floor beneath it is correctly free/cleanable — a vacuum's whole job. Solid
furniture (sofa) still blocks. The stock SLAM map can't be used because it is
in a frame offset from the gz world; this map is world == map == ground truth.

Usage: gen_livingroom_map.py <world> <models_dir> <out_dir>
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from PIL import Image
from scipy import ndimage

RES = 0.05
OX, OY = -2.75, -2.75
W = H = 110                 # 5.5 x 5.5 m, world-aligned
Z_MIN, Z_MAX = 0.02, 0.20   # robot body band: what it can bump / LiDAR sees

# decor that is not an obstacle at floor level (flat, on-wall, or lighting)
SKIP = ('poster', 'sun', 'ground', 'curtain', 'rug', 'tv_65')


def T_of(p):
    x, y, z, _r, _p, yw = (list(p) + [0] * 6)[:6]
    cy, sy = math.cos(yw), math.sin(yw)   # roll/pitch are 0 in this world
    T = np.eye(4)
    T[0, 0], T[0, 1] = cy, -sy
    T[1, 0], T[1, 1] = sy, cy
    T[:3, 3] = [x, y, z]
    return T


def pose_of(elem):
    if elem is None:
        return [0.0] * 6
    p = elem.find('pose')
    if p is None or not p.text:
        return [0.0] * 6
    return [float(t) for t in p.text.split()]


def rasterize_tris(occ, tris):
    """Mark cells covered by xy-projected triangles (dense edge+interior sample)."""
    for tri in tris:
        # sample barycentric grid finer than the map resolution
        n = max(2, int(np.ceil(np.max(np.ptp(tri[:, :2], axis=0)) / (RES * 0.5))) + 1)
        for i in range(n + 1):
            for j in range(n + 1 - i):
                a, b = i / n, j / n
                p = tri[0] * (1 - a - b) + tri[1] * a + tri[2] * b
                gx = int((p[0] - OX) / RES)
                gy = int((p[1] - OY) / RES)
                if 0 <= gx < W and 0 <= gy < H:
                    occ[gy, gx] = True


def add_mesh(occ, mesh_path, scale, T):
    g = trimesh.load(mesh_path, force='mesh')
    V = g.vertices * scale
    V = np.c_[V, np.ones(len(V))] @ T.T
    V = V[:, :3]
    F = g.faces
    zmin = V[F][:, :, 2].min(axis=1)
    zmax = V[F][:, :, 2].max(axis=1)
    band = (zmax > Z_MIN) & (zmin < Z_MAX)
    rasterize_tris(occ, V[F[band]])
    return int(band.sum())


def add_box(occ, size, T):
    sx, sy, sz = size
    cz = T[2, 3]
    if cz + sz / 2 < Z_MIN or cz - sz / 2 > Z_MAX:
        return 0   # entirely above/below the robot band
    gx, gy = np.meshgrid(np.arange(W), np.arange(H))
    wx = OX + (gx + 0.5) * RES
    wy = OY + (gy + 0.5) * RES
    cx, cy = T[0, 3], T[1, 3]
    yaw = math.atan2(T[1, 0], T[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = c * (wx - cx) - s * (wy - cy)
    ly = s * (wx - cx) + c * (wy - cy)
    occ |= (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
    return 1


def collisions_of(model_elem, T_base, model_dir, occ):
    n_box = n_tri = 0
    Pm = T_of(pose_of(model_elem))
    for link in model_elem.findall('link'):
        Pl = T_of(pose_of(link))
        for col in link.findall('collision'):
            Pc = T_of(pose_of(col))
            T = T_base @ Pm @ Pl @ Pc
            box = col.find('.//box/size')
            mesh = col.find('.//mesh')
            if box is not None and box.text:
                n_box += add_box(occ, [float(t) for t in box.text.split()], T)
            elif mesh is not None:
                uri = mesh.find('uri').text.strip()
                scl = mesh.find('scale')
                scale = np.array([float(t) for t in scl.text.split()]) if (
                    scl is not None and scl.text) else np.ones(3)
                rel = uri[len('model://'):] if uri.startswith('model://') else None
                path = (os.path.join(os.path.dirname(model_dir), rel) if rel
                        else os.path.join(model_dir, uri))
                if os.path.exists(path):
                    n_tri += add_mesh(occ, path, scale, T)
    return n_box, n_tri


def main():
    world_path, models_dir, out_dir = sys.argv[1:4]
    world = ET.parse(world_path).getroot().find('world')
    occ = np.zeros((H, W), bool)
    for model in world.findall('model'):
        name = model.get('name', '')
        if any(s in name.lower() for s in SKIP):
            continue
        Tw = T_of(pose_of(model))
        inc = model.find('include')
        if inc is not None and inc.find('uri') is not None:
            mdir = os.path.join(models_dir, inc.find('uri').text.strip()[8:])
            msdf = os.path.join(mdir, 'model.sdf')
            if not os.path.exists(msdf):
                continue
            me = ET.parse(msdf).getroot().find('model')
            nb, nt = collisions_of(me, Tw, mdir, occ)
        else:
            nb, nt = collisions_of(model, np.eye(4), models_dir, occ)
        print(f'{name:<22} boxes={nb} band_tris={nt}')

    # free = largest connected non-occupied region (the room interior)
    lbl, n = ndimage.label(~occ)
    sizes = ndimage.sum(np.ones_like(lbl), lbl, index=range(1, n + 1))
    free = lbl == (1 + int(np.argmax(sizes)))

    img = np.full((H, W), 205, np.uint8)
    img[free] = 254
    img[occ] = 0
    os.makedirs(out_dir, exist_ok=True)
    Image.fromarray(np.flipud(img), 'L').save(os.path.join(out_dir, 'living_room.pgm'))
    with open(os.path.join(out_dir, 'living_room.yaml'), 'w') as f:
        f.write(f"image: living_room.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{OX}, {OY}, 0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f'\nfree={int(free.sum())} ({free.sum()*RES*RES:.1f} m2)  '
          f'occ={int(occ.sum())}  origin=({OX},{OY}) {W}x{H}')


if __name__ == '__main__':
    main()
