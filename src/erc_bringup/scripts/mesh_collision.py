#!/usr/bin/env python3
"""
Decompose an axis-aligned STL into exact collision boxes.

MuJoCo collides mesh geoms as their convex hull, so the arena's non-convex
props cannot use their visual mesh for physics: hulling the shelf turns it
into a solid slab with no shelves to put a book on, and hulling the table
fills the space the robot drives into.

The arena props are modelled as unions of axis-aligned rectangular slabs, so
an exact decomposition exists. This reads the mesh's own face planes, tests
solidity at the centre of every cell of the resulting grid, and greedily
merges occupied cells into maximal boxes. The union of those boxes is exactly
the mesh volume - verified before anything is written, so a mesh that is not
slab-shaped fails loudly instead of silently producing bad collision.

    ./mesh_collision.py path/to/mesh.STL --name shelf

Meshes with curved or bevelled faces (the collection bin) are not slab-shaped
and are rejected; those keep hand-fitted primitives.
"""

import argparse
import struct
import sys

import numpy as np

# A face counts as axis-aligned if its normal is within this of a unit axis.
NORMAL_TOL = 1e-4


def read_binary_stl(path):
    """Return (normals, triangles) as float64 arrays."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 84:
        sys.exit(f'{path}: too short to be a binary STL')
    count = struct.unpack('<I', data[80:84])[0]
    expected = 84 + 50 * count
    if len(data) != expected:
        sys.exit(f'{path}: not a binary STL (expected {expected} bytes for '
                 f'{count} triangles, file is {len(data)})')
    normals = np.empty((count, 3))
    tris = np.empty((count, 3, 3))
    for i in range(count):
        off = 84 + 50 * i
        v = struct.unpack('<12f', data[off:off + 48])
        normals[i] = v[0:3]
        tris[i] = (v[3:6], v[6:9], v[9:12])
    return normals, tris


def face_planes(normals, tris):
    """Distinct face-plane coordinates per axis, from axis-aligned faces."""
    planes = [set(), set(), set()]
    off_axis = 0
    for n, t in zip(normals, tris):
        axis = None
        for k in range(3):
            if abs(abs(n[k]) - 1.0) < NORMAL_TOL:
                axis = k
        if axis is None:
            off_axis += 1
            continue
        planes[axis].add(round(float(t[0][axis]), 6))
    return [sorted(p) for p in planes], off_axis


def _cast(tris, points, axis):
    """Even-odd ray cast along +axis. Points must not lie on a face plane."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    d = np.zeros(3)
    d[axis] = 1.0
    h = np.cross(d, e2)
    a = np.einsum('ij,ij->i', e1, h)
    usable = np.abs(a) > 1e-12
    f = np.zeros_like(a)
    f[usable] = 1.0 / a[usable]
    out = np.zeros(len(points), dtype=bool)
    for i, p in enumerate(points):
        s = p - v0
        u = f * np.einsum('ij,ij->i', s, h)
        q = np.cross(s, e1)
        w = f * np.einsum('j,ij->i', d, q)
        t = f * np.einsum('ij,ij->i', e2, q)
        # Strict inequalities: a ray through the shared diagonal of a
        # triangulated rectangular face would otherwise be counted in both
        # triangles, flipping the parity and silently inverting that cell.
        hit = usable & (u > 0) & (w > 0) & (u + w < 1) & (t > 1e-9)
        out[i] = bool(hit.sum() % 2)
    return out


def inside(tris, points):
    """Solidity at each point, cross-checked along two independent axes.

    A single ray can be defeated by grazing an edge or a vertex. Two axes
    disagreeing means the answer is not trustworthy for that point, and a
    wrong cell here becomes wrong collision geometry that nothing downstream
    would catch - so it is a hard error rather than a coin flip.
    """
    z = _cast(tris, points, 2)
    x = _cast(tris, points, 0)
    if not np.array_equal(z, x):
        bad = np.flatnonzero(z != x)
        sys.exit(f'ray casts disagree at {len(bad)} sample point(s), first at '
                 f'{points[bad[0]]}: this mesh needs a real convex '
                 f'decomposition, not a slab decomposition')
    return z


def occupancy(tris, planes):
    """Solid/empty for every cell of the face-plane grid."""
    nx, ny, nz = (len(p) - 1 for p in planes)
    centres = np.empty((nx * ny * nz, 3))
    idx = 0
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                centres[idx] = ((planes[0][i] + planes[0][i + 1]) / 2,
                                (planes[1][j] + planes[1][j + 1]) / 2,
                                (planes[2][k] + planes[2][k + 1]) / 2)
                idx += 1
    return inside(tris, centres).reshape(nx, ny, nz)


def greedy_boxes(grid):
    """Merge occupied cells into maximal boxes. Returns index ranges."""
    nx, ny, nz = grid.shape
    todo = grid.copy()
    boxes = []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if not todo[i, j, k]:
                    continue
                i1 = i
                while i1 + 1 < nx and todo[i1 + 1, j, k]:
                    i1 += 1
                j1 = j
                while j1 + 1 < ny and todo[i:i1 + 1, j1 + 1, k].all():
                    j1 += 1
                k1 = k
                while k1 + 1 < nz and todo[i:i1 + 1, j:j1 + 1, k1 + 1].all():
                    k1 += 1
                todo[i:i1 + 1, j:j1 + 1, k:k1 + 1] = False
                boxes.append((i, i1, j, j1, k, k1))
    return boxes


def verify(grid, boxes):
    rebuilt = np.zeros_like(grid)
    for i0, i1, j0, j1, k0, k1 in boxes:
        rebuilt[i0:i1 + 1, j0:j1 + 1, k0:k1 + 1] = True
    return bool((rebuilt == grid).all())


def mesh_volume(tris):
    """Signed volume via the divergence theorem."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    return float(abs(np.einsum('ij,ij->i', v0, np.cross(v1, v2)).sum()) / 6.0)


def decompose(path, verbose=True):
    """Return [(centre, half_size)] boxes in mesh-local coordinates."""
    normals, tris = read_binary_stl(path)
    planes, off_axis = face_planes(normals, tris)
    if off_axis:
        sys.exit(f'{path}: {off_axis}/{len(tris)} faces are not axis-aligned, '
                 f'so this mesh is not a union of slabs. Author its collision '
                 f'primitives by hand instead.')
    grid = occupancy(tris, planes)
    boxes = greedy_boxes(grid)
    if not verify(grid, boxes):
        sys.exit(f'{path}: box merge did not reproduce the occupancy grid')

    out = []
    volume = 0.0
    for i0, i1, j0, j1, k0, k1 in boxes:
        lo = (planes[0][i0], planes[1][j0], planes[2][k0])
        hi = (planes[0][i1 + 1], planes[1][j1 + 1], planes[2][k1 + 1])
        centre = tuple((a + b) / 2 for a, b in zip(lo, hi))
        half = tuple((b - a) / 2 for a, b in zip(lo, hi))
        volume += 8 * half[0] * half[1] * half[2]
        out.append((centre, half))

    # Volume equality alone would not catch a mis-marked pair of cells that
    # cancel out, so verify() has already compared the box union against the
    # occupancy grid cell by cell. This is the second, independent check.
    exact = mesh_volume(tris)
    if abs(volume - exact) > 1e-6:
        sys.exit(f'{path}: decomposed volume {volume:.6f} != mesh volume '
                 f'{exact:.6f}; the mesh is not a union of slabs')
    if verbose:
        print(f'{path}: {len(tris)} triangles -> {len(out)} boxes, '
              f'volume {volume:.6f} m^3 (exact)', file=sys.stderr)
    return out


def to_mjcf(boxes, name, indent='    '):
    lines = []
    for n, (centre, half) in enumerate(boxes):
        lines.append(
            f'{indent}<geom name="{name}_collision_{n}" type="box" '
            f'pos="{centre[0]:.6g} {centre[1]:.6g} {centre[2]:.6g}" '
            f'size="{half[0]:.6g} {half[1]:.6g} {half[2]:.6g}"/>')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mesh')
    ap.add_argument('--name', required=True, help='geom name prefix')
    args = ap.parse_args()
    print(to_mjcf(decompose(args.mesh), args.name))


if __name__ == '__main__':
    main()
