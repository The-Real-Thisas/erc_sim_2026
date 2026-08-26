#!/usr/bin/env python3
"""
Generate the competition arena as a MuJoCo scene (MJCF).

This is the MuJoCo counterpart of erc_world.sdf: the same floor, walls, start
zone, table, shelf, collection bin, books and number markers, at the same
poses, with the same ERC_SEED-driven layout. The MJCF converter inlines this
file's children into the robot model, so the robot and the arena end up in one
MuJoCo model.

Two things force this to be generated rather than checked in:

  * mesh and texture paths must be absolute (the converter copies this file's
    contents into an MJCF written somewhere else, so relative paths would
    resolve against the wrong directory), and
  * the book colour layout and the number-marker order are drawn from ERC_SEED
    at launch, exactly as simulation.launch.py draws them for Gazebo.

Collision geometry is not the visual mesh: MuJoCo collides a mesh as its convex
hull, which would make the shelf a solid slab with nowhere to put a book and
fill the leg room under the table. The table and shelf are decomposed into
exact boxes by mesh_collision.py; the bin is bevelled rather than slab-shaped,
so its interior is described by hand-fitted primitives measured from the mesh.

    ros2 run erc_bringup generate_mujoco_world.py -o /tmp/erc_world.xml
    ERC_SEED=7 ros2 run erc_bringup generate_mujoco_world.py -o /tmp/erc_world.xml
"""

import argparse
import os
import random
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_collision import decompose  # noqa: E402

# ── Arena layout ────────────────────────────────────────────────────────────
# Mirrors erc_world.sdf and simulation.launch.py. Keep the two in step: a book
# that spawns in a different column here than in Gazebo is a silent divergence.

BOOK_COLOURS = {
    'red':    (1.0, 0.0, 0.0, 1.0),
    'green':  (0.0, 1.0, 0.0, 1.0),
    'yellow': (1.0, 1.0, 0.0, 1.0),
    'blue':   (0.0, 0.0, 1.0, 1.0),
}

SHELF_X, SHELF_Y, SHELF_Z = 3.0, 0.0, 1.1
NUM_COLUMNS, NUM_ROWS = 5, 6
ACTIVE_ROWS = [1, 2, 3, 4]
COLUMN_WIDTH = 1.0
SHELF_HEIGHT = 2.10
COLUMN_JITTER_RANGE = COLUMN_WIDTH * 0.25
ROTATE_90_DEGREES_RAD = 1.5708
COLUMN_Y_OFFSETS = [((NUM_COLUMNS - 1) / 2 - c) * COLUMN_WIDTH for c in range(NUM_COLUMNS)]
TOP_BOOK_Z = 0.825
ROW_SPACING = 0.33
ROW_FLOOR_Z_OFFSETS = [TOP_BOOK_Z - (i * ROW_SPACING) for i in range(NUM_ROWS)]
NUMBER_MARKER_PLATE_X = SHELF_X - 0.245
NUMBER_MARKER_PLATE_Z = 2.26

BOOK_HALF = (0.125, 0.015, 0.08)     # the SDF's 0.25 x 0.03 x 0.16 box
BOOK_MASS = 0.3
BOOK_FRICTION = '5.0 0.01 0.002'     # sliding mu 5.0 is the SDF's own value
MARKER_HALF = (0.15, 0.15, 0.01)     # the SDF's 0.3 x 0.3 plate face

# Every arena prop is placed with the same SDF pose rpy "1.5708 0 -1.5708",
# which is this quaternion. Under it mesh +X is world -Y, mesh +Y is world +Z
# and mesh +Z is world -X.
PROP_QUAT = '0.5 0.5 -0.5 -0.5'

# Pitch the book 90 degrees about Y so it stands on the shelf like a book.
BOOK_QUAT = '0.70710678 0 0.70710678 0'
# The number plates are plane geoms, not boxes: MuJoCo maps a 2d texture across
# a plane but only samples a single line of it across a box face, which renders
# the digits as a blank white slab. These axes put the plane's normal along -X
# (facing the robot) with the digit upright and reading the right way round.
MARKER_XYAXES = '0 -1 0 0 0 1'

# The collection bin's interior, in bin-local coordinates. The mesh is bevelled,
# so mesh_collision.py refuses it and these are measured from the mesh instead:
# interior floor top at bin-local y -0.02 (world z 0.750 once the bin rests on
# the table), 0.31 x 0.56 clear, walls 0.10 high, and a lower lip at the open
# front edge. Named so a contact check can tell the floor from the walls.
BIN_COLLISION = [
    ('bin_floor',      (0.0, -0.100, -0.025), (0.155, 0.005, 0.280)),
    ('bin_wall_back',  (0.0, 0.005, 0.2498),  (0.155, 0.100, 0.0052)),
    ('bin_wall_left',  (-0.15, 0.005, -0.025), (0.005, 0.100, 0.280)),
    ('bin_wall_right', (0.15, 0.005, -0.025), (0.005, 0.100, 0.280)),
    ('bin_lip_front',  (0.0, -0.055, -0.2927), (0.155, 0.040, 0.0122)),
]
BIN_MASS = 4.70313898335195
# Spawned already resting on the table. erc_world.sdf drops it from z 1.3 onto
# a table whose top is at 0.74; it settles here, and starting it settled keeps
# the arena deterministic instead of depending on how a 4.7 kg box bounces.
BIN_POSE = (-1.0, 0.0, 0.845)


def fmt(values):
    return ' '.join(f'{v:g}' for v in values)


def collision_geoms(mesh_path, name, friction=None, indent=6):
    """Exact box decomposition of a slab-shaped mesh, as MJCF geoms."""
    pad = ' ' * indent
    extra = f' friction="{friction}"' if friction else ''
    out = []
    for i, (centre, half) in enumerate(decompose(mesh_path)):
        out.append(f'{pad}<geom name="{name}_collision_{i}" class="arena_collision" '
                   f'type="box" pos="{fmt(centre)}" size="{fmt(half)}"{extra}/>')
    return '\n'.join(out)


def visual_mesh(name, mesh_name, rgba, indent=6):
    pad = ' ' * indent
    return (f'{pad}<geom name="{name}_visual" class="arena_visual" '
            f'type="mesh" mesh="{mesh_name}" rgba="{fmt(rgba)}"/>')


def build_world(share, seed=None):
    if seed is not None:
        random.seed(int(seed))

    models = os.path.join(share, 'models')
    table_mesh = os.path.join(models, 'table', 'meshes', 'erc_base_table.STL')
    shelf_mesh = os.path.join(models, 'shelf', 'meshes', 'erc_base_shelf.STL')
    bin_mesh = os.path.join(models, 'collection_bin', 'meshes',
                            'erc_base_collection_bin.STL')
    floor_tex = os.path.join(share, 'materials', 'textures', 'arena_floor.png')
    marker_tex_dir = os.path.join(models, 'number_marker', 'textures')

    for path in (table_mesh, shelf_mesh, bin_mesh, floor_tex):
        if not os.path.exists(path):
            sys.exit(f'missing arena asset: {path}')

    # ── Assets ──
    assets = [
        '    <texture type="skybox" builtin="flat" rgb1="0.8 0.8 0.8" '
        'rgb2="0.8 0.8 0.8" width="512" height="512"/>',
        f'    <texture type="2d" name="arena_floor" file="{floor_tex}"/>',
        '    <material name="arena_floor" texture="arena_floor" '
        'texrepeat="1 1" texuniform="false"/>',
        f'    <mesh name="erc_table" file="{table_mesh}"/>',
        f'    <mesh name="erc_shelf" file="{shelf_mesh}"/>',
        f'    <mesh name="erc_collection_bin" file="{bin_mesh}"/>',
    ]
    for colour, rgba in BOOK_COLOURS.items():
        assets.append(f'    <material name="book_{colour}" rgba="{fmt(rgba)}"/>')
    for number in range(1, NUM_COLUMNS + 1):
        tex = os.path.join(marker_tex_dir, f'{number}.png')
        if not os.path.exists(tex):
            sys.exit(f'missing number marker texture: {tex}')
        assets.append(f'    <texture type="2d" name="marker_{number}" file="{tex}"/>')
        assets.append(f'    <material name="marker_{number}" texture="marker_{number}" '
                      f'texrepeat="1 1" texuniform="false"/>')

    # ── Static arena shell ──
    body = [
        '    <!-- The SDF\'s four directional lights. All four are needed, not just',
        '         an overhead one: a light pointing straight down leaves every',
        '         vertical face unlit, which renders the number markers black. -->',
        '    <light name="light_front" pos="0 -10 5" dir="0 1 -0.5" directional="true" '
        'diffuse="0.3 0.3 0.3" specular="0.05 0.05 0.05" castshadow="false"/>',
        '    <light name="light_back" pos="0 10 5" dir="0 -1 -0.5" directional="true" '
        'diffuse="0.3 0.3 0.3" specular="0.05 0.05 0.05" castshadow="false"/>',
        '    <light name="light_left" pos="-10 0 5" dir="1 0 -0.5" directional="true" '
        'diffuse="0.3 0.3 0.3" specular="0.05 0.05 0.05" castshadow="false"/>',
        '    <light name="light_right" pos="10 0 5" dir="-1 0 -0.5" directional="true" '
        'diffuse="0.3 0.3 0.3" specular="0.05 0.05 0.05" castshadow="false"/>',
        '',
        '    <!-- Ground plane, 10x10 centred on the arena like erc_world.sdf. -->',
        '    <geom name="arena_floor" type="plane" pos="1 0 0" size="5 5 0.05" '
        'material="arena_floor"/>',
        '    <!-- Painted arena boundary and start zone: visual only, as in the SDF. -->',
        '    <geom name="arena_boundary" class="arena_visual" type="box" pos="1 0 0.001" '
        'size="3.5 3.5 0.001" rgba="0.75 0.75 0.72 1"/>',
        '    <geom name="start_zone" class="arena_visual" type="box" pos="0 0 0.002" '
        'size="0.4 0.4 0.001" rgba="0.1 0.4 0.1 1"/>',
        '',
        '    <!-- Outer walls. -->',
        '    <geom name="wall_north" type="box" pos="1 5 1.5" size="5 0.025 1.5" '
        'rgba="0.8 0.8 0.8 1"/>',
        '    <geom name="wall_south" type="box" pos="1 -5 1.5" size="5 0.025 1.5" '
        'rgba="0.8 0.8 0.8 1"/>',
        '    <geom name="wall_east" type="box" pos="6 0 1.5" size="0.025 5 1.5" '
        'rgba="0.8 0.8 0.8 1"/>',
        '    <geom name="wall_west" type="box" pos="-4 0 1.5" size="0.025 5 1.5" '
        'rgba="0.8 0.8 0.8 1"/>',
        '',
        '    <!-- Table: visual mesh, collision decomposed into exact boxes. -->',
        f'    <body name="erc_table" pos="-1 0 0.7" quat="{PROP_QUAT}">',
        visual_mesh('erc_table', 'erc_table', (1, 1, 1, 1)),
        collision_geoms(table_mesh, 'erc_table'),
        '    </body>',
        '',
        '    <!-- Shelf: five 1.0 m columns over six 0.33 m rows. -->',
        f'    <body name="erc_shelf" pos="{SHELF_X} {SHELF_Y} {SHELF_Z}" quat="{PROP_QUAT}">',
        visual_mesh('erc_shelf', 'erc_shelf', (1, 1, 1, 1)),
        collision_geoms(shelf_mesh, 'erc_shelf'),
        '    </body>',
        '',
        '    <!-- Collection bin: a free body, resting on the table. -->',
        f'    <body name="erc_collection_bin" pos="{fmt(BIN_POSE)}" quat="{PROP_QUAT}">',
        '      <freejoint name="erc_collection_bin_joint"/>',
        f'      <inertial pos="0 0 0" mass="{BIN_MASS:g}" diaginertia="0.0896 0.2232 0.1752"/>',
        visual_mesh('erc_collection_bin', 'erc_collection_bin', (1, 0, 0, 1)),
    ]
    for name, pos, half in BIN_COLLISION:
        body.append(f'      <geom name="{name}" class="arena_collision" type="box" '
                    f'pos="{fmt(pos)}" size="{fmt(half)}"/>')
    body.append('    </body>')
    body.append('')

    # ── Books: same draw order as simulation.launch.py, so a given ERC_SEED
    #    produces the same layout on both backends. ──
    body.append('    <!-- Books: one per active shelf row, colours drawn from ERC_SEED. -->')
    for col in range(NUM_COLUMNS):
        colours_this_column = list(BOOK_COLOURS.keys())
        random.shuffle(colours_this_column)
        for i, row in enumerate(ACTIVE_ROWS):
            colour = colours_this_column[i]
            name = f'book_col_{col + 1}_row_{row + 1}_{colour}'
            y = (SHELF_Y + COLUMN_Y_OFFSETS[col]
                 + random.uniform(-COLUMN_JITTER_RANGE, COLUMN_JITTER_RANGE))
            z = SHELF_Z + ROW_FLOOR_Z_OFFSETS[row]
            x = SHELF_X - 0.1
            body.append(f'    <body name="{name}" pos="{x:g} {y:.6g} {z:g}" quat="{BOOK_QUAT}">')
            body.append(f'      <freejoint name="{name}_joint"/>')
            body.append(f'      <geom name="{name}_geom" type="box" size="{fmt(BOOK_HALF)}" '
                        f'mass="{BOOK_MASS:g}" friction="{BOOK_FRICTION}" condim="6" '
                        f'material="book_{colour}"/>')
            body.append('    </body>')
    body.append('')

    # ── Number markers: static, visual only, order drawn from ERC_SEED. ──
    body.append('    <!-- Number markers above each column, order drawn from ERC_SEED. -->')
    shelf_number_markers = list(range(1, NUM_COLUMNS + 1))
    random.shuffle(shelf_number_markers)
    for col in range(NUM_COLUMNS):
        number = shelf_number_markers[col]
        y = SHELF_Y + COLUMN_Y_OFFSETS[col]
        body.append(
            f'    <geom name="number_marker_col_{col + 1}" class="arena_visual" type="plane" '
            f'pos="{NUMBER_MARKER_PLATE_X:g} {y:g} {NUMBER_MARKER_PLATE_Z:g}" '
            f'xyaxes="{MARKER_XYAXES}" size="{fmt(MARKER_HALF)}" material="marker_{number}"/>')

    # A fixed vantage on the whole working area, for recordings and for a
    # human watching a headless run. Placed south of the arena looking north
    # so the table, the start zone and the shelf are all in frame.
    body.append('')
    body.append('    <camera name="spectator" mode="fixed" pos="1 -4.5 2.6" '
                'xyaxes="1 0 0 0 0.3714 0.9284" resolution="960 540"/>')

    return f'''<mujoco model="erc_arena">
  <!-- GENERATED by erc_bringup/scripts/generate_mujoco_world.py - do not edit.
       The MuJoCo counterpart of erc_description/worlds/erc_world.sdf. -->

  <!-- Sized for the working volume around the robot, not the whole 10 m arena:
       the camera near plane is a fraction of this extent, so a large extent
       would push it out past the table and clip everything the robot reaches. -->
  <statistic center="1 0 0.8" extent="2"/>

  <!-- The robot's own <option> repeats these and wins on merge (raw_inputs are
       appended after the scene). They are here so the arena is also correct
       when loaded on its own: under the Euler/pyramidal defaults a resting
       book sinks 6.4 mm into a shelf, against 0.1 mm with these. -->
  <option integrator="implicitfast" cone="elliptic" impratio="10">
    <flag multiccd="enable"/>
  </option>

  <!-- The offscreen framebuffer defaults to 640x480, which is smaller than the
       spectator camera; anything rendering it offscreen fails without this.
       Merges with the robot's own <visual>, which sets the near/far planes. -->
  <visual>
    <global offwidth="960" offheight="540"/>
    <!-- The SDF's ambient 0.7 cannot be copied across literally: Gazebo and
         MuJoCo sum ambient and diffuse differently, and four directional
         lights at the SDF's 0.5 plus that ambient saturate every upward face
         to pure white. Scaled so a lit floor lands near 0.8, not past 1.0. -->
    <headlight ambient="0.25 0.25 0.25" diffuse="0.05 0.05 0.05" specular="0 0 0"/>
  </visual>

  <default>
    <default class="arena_visual">
      <geom group="2" contype="0" conaffinity="0"/>
    </default>
    <default class="arena_collision">
      <geom group="3" rgba="0 0 0 0"/>
    </default>
  </default>

  <asset>
{chr(10).join(assets)}
  </asset>

  <worldbody>
{chr(10).join(body)}
  </worldbody>
</mujoco>
'''


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-o', '--output', required=True)
    ap.add_argument('--seed', default=os.environ.get('ERC_SEED'),
                    help='book/marker layout seed (default: $ERC_SEED)')
    args = ap.parse_args()

    share = get_package_share_directory('erc_description')
    xml = build_world(share, args.seed)
    with open(args.output, 'w') as f:
        f.write(xml)
    print(f'[generate_mujoco_world] wrote {args.output} '
          f'({len(xml):,} bytes, seed={args.seed})')


if __name__ == '__main__':
    main()
