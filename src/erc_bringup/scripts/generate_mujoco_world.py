#!/usr/bin/env python3
"""
Generate the competition arena as a MuJoCo scene (MJCF).

This file IS the arena definition: floor, walls, start zone, table, shelf,
collection bin, books and number markers. Poses, sizes, masses and frictions
below are the competition's own, carried over from the SDF world and model
files the arena was previously described by. The MJCF converter inlines this
file's children into the robot model, so the robot and the arena end up in one
MuJoCo model.

Two things force this to be generated rather than checked in:

  * mesh and texture paths must be absolute (the converter copies this file's
    contents into an MJCF written somewhere else, so relative paths would
    resolve against the wrong directory), and
  * the book colour layout and the number-marker order are drawn from ERC_SEED
    at launch.

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
COLUMN_JITTER_RANGE = COLUMN_WIDTH * 0.25
COLUMN_Y_OFFSETS = [((NUM_COLUMNS - 1) / 2 - c) * COLUMN_WIDTH for c in range(NUM_COLUMNS)]
TOP_BOOK_Z = 0.825
ROW_SPACING = 0.33
ROW_FLOOR_Z_OFFSETS = [TOP_BOOK_Z - (i * ROW_SPACING) for i in range(NUM_ROWS)]
NUMBER_MARKER_PLATE_X = SHELF_X - 0.245
NUMBER_MARKER_PLATE_Z = 1.93         # phase 2: on the top shelf row, in reach of the head camera (upstream cc3f2e0)

# Round 2 (upstream simulation_round_2.launch.py): columns 1, 3, 5 as round 1;
# column 2 holds four white books whose spines carry a word, shuffled by the
# seed; column 4 holds coloured books flush against the column's side walls,
# alternating left/right from the top row down. Round 2 lays books out on a
# 1.03 m pitch where round 1 uses 1.0; both are kept so a seed reproduces the
# organisers' layout draw for draw.
ROUND2_PITCH = COLUMN_WIDTH + 0.03
WHITE_COLUMN, EDGE_COLUMN = 2, 4
WHITE_BOOKS = [('A', 'A1'), ('B', 'B1'), ('C', 'C1'), ('D', 'D1')]   # (cover, spine) textures
# The words the spine textures carry, for a judge; the robot has to read them.
WHITE_WORDS = {'A': 'FUTURE', 'B': 'ROBOT', 'C': 'LIBRARY', 'D': 'AGENT'}
# The shelf mesh's dividers (mesh_collision.py's exact boxes): 35 mm thick on a
# 1.035 m pitch, so column c's inner faces sit at its centre +-0.5. Upstream
# puts its edge books at the 1.03 grid +-0.485, which leaves one of each pair
# 5 mm inside the divider and the other 5 mm clear of it; the physical shelf
# has them flush, so this does: book face 0.5 mm off the divider's face.
SHELF_DIVIDER_PITCH = 1.035
EDGE_CLEARANCE = 0.0005

BOOK_HALF = (0.125, 0.015, 0.08)     # the competition's 0.25 x 0.03 x 0.16 box (upstream cc3f2e0)
BOOK_MASS = 0.3
# Upstream sets mu 10 so books never slip in Gazebo. A grip or a drag that only
# works at mu 10 fails on the real robot, so the dev sim uses realistic values
# and sweeps them: this is a book's paper cover on a wooden board (ERC_BOOK_MU).
# priority="2" makes it govern the book's contacts with the shelf, the bin and
# other books (MuJoCo otherwise takes the larger of a pair); the fingertip pads
# carry their own, higher, at priority 3 (generate_mujoco_urdf.py PAD_MU), so a
# pinch and a drag each see the two frictions a real one would.
BOOK_MU = 0.5
MARKER_HALF = (0.15, 0.15, 0.01)     # the competition's 0.3 x 0.3 plate face

# Every arena prop is placed with the same pose rpy "1.5708 0 -1.5708",
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
# The white books' printed faces. A plane is only allowed on a static body, so
# each print is a single textured quad (a flat mesh, shell inertia, no mass) a
# hair off the box face, in book-local axes: the body's x is world -z, its y
# world y, its z world +x. The spine, at local z -half, faces the robot with
# the image's columns across it (u along world -y) and its rows top to bottom
# (v along world -z), as upstream paints it; the covers sit on the two large
# +-y faces with the image's long side along the book's height.
PRINT_PROUD = 0.0005


def _quad(name, verts, uv, faces):
    return (f'    <mesh name="{name}" inertia="shell" vertex="{fmt(v for p in verts for v in p)}" '
            f'texcoord="{fmt(t for p in uv for t in p)}" face="{faces}"/>')


def print_meshes():
    hx, hy, hz = BOOK_HALF
    z = -(hz + PRINT_PROUD)
    yp = hy + PRINT_PROUD
    corners = ((0, 0), (1, 0), (1, 1), (0, 1))
    return [
        # spine: top-left, top-right, bottom-right, bottom-left as seen from the robot
        _quad('white_spine', [(-hx, hy, z), (-hx, -hy, z), (hx, -hy, z), (hx, hy, z)],
              corners, '0 2 1 0 3 2'),
        _quad('white_cover_p', [(-hx, yp, -hz), (hx, yp, -hz), (hx, yp, hz), (-hx, yp, hz)],
              corners, '0 2 1 0 3 2'),
        _quad('white_cover_n', [(-hx, -yp, -hz), (hx, -yp, -hz), (hx, -yp, hz), (-hx, -yp, hz)],
              corners, '0 1 2 0 2 3'),
    ]

# The collection bin's interior, in bin-local coordinates. The mesh is bevelled,
# so mesh_collision.py refuses it and these are measured from the mesh instead:
# the interior floor's top face is at bin-local y -0.095, which is world z
# 0.750 once the bin rests on the table; 0.31 x 0.56 clear, walls 0.20 tall,
# and a lower lip at the open front edge. Sizes below are MuJoCo half-extents.
# Named so a contact check can tell the floor from the walls.
BIN_COLLISION = [
    ('bin_floor',      (0.0, -0.100, -0.025), (0.155, 0.005, 0.280)),
    ('bin_wall_back',  (0.0, 0.005, 0.2498),  (0.155, 0.100, 0.0052)),
    ('bin_wall_left',  (-0.15, 0.005, -0.025), (0.005, 0.100, 0.280)),
    ('bin_wall_right', (0.15, 0.005, -0.025), (0.005, 0.100, 0.280)),
    ('bin_lip_front',  (0.0, -0.055, -0.2927), (0.155, 0.040, 0.0122)),
]
BIN_MASS = 4.70313898335195
# Straight from the competition's <inertial>: the centre of mass sits 34.9 mm off the
# body origin, and ixy is 7% of ixx, so neither can be dropped without changing
# how the bin rocks when a book lands in it.
BIN_COM = '-0.000869 -0.034922 0'
BIN_INERTIA = '0.0896407437951023 0.223231929617035 0.175200579933907 0.00627075393219412 0 0'
# Spawned already resting on the table. The arena spec drops it from z 1.3 onto
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


def book_friction(mu):
    return f'{mu:g} 0.01 0.002'


def book_body(name, pos, material, mu, prints=()):
    """A free standing book; `prints` are extra textured planes (white books)."""
    x, y, z = pos
    out = [f'    <body name="{name}" pos="{x:g} {y:.6g} {z:g}" quat="{BOOK_QUAT}">',
           f'      <freejoint name="{name}_joint"/>',
           f'      <geom name="{name}_geom" type="box" size="{fmt(BOOK_HALF)}" '
           f'mass="{BOOK_MASS:g}" friction="{book_friction(mu)}" condim="6" priority="2" '
           f'material="{material}"/>']
    out += [f'      {p}' for p in prints]
    out.append('    </body>')
    return out


def white_prints(name, cover, spine):
    return [f'<geom name="{name}_{part}" class="arena_visual" type="mesh" mesh="white_{part}" '
            f'mass="0" material="white_{tex}"/>'
            for part, tex in (('spine', spine), ('cover_p', cover), ('cover_n', cover))]


def build_world(share, seed=None, rnd=1, book_mu=BOOK_MU):
    # An empty ERC_SEED means 'unseeded'; int('') would otherwise abort the
    # whole launch with a traceback.
    if seed:
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
    if rnd == 2:
        assets.append('    <material name="book_white" rgba="1 1 1 1"/>')
        white_tex_dir = os.path.join(models, 'white_book', 'textures')
        for tex_name in sorted({t for pair in WHITE_BOOKS for t in pair}):
            tex = os.path.join(white_tex_dir, f'{tex_name}.png')
            if not os.path.exists(tex):
                sys.exit(f'missing white book texture: {tex}')
            assets.append(f'    <texture type="2d" name="white_{tex_name}" file="{tex}"/>')
            assets.append(f'    <material name="white_{tex_name}" texture="white_{tex_name}" '
                          f'texrepeat="1 1" texuniform="false"/>')
        assets += print_meshes()
    for number in range(1, NUM_COLUMNS + 1):
        tex = os.path.join(marker_tex_dir, f'{number}.png')
        if not os.path.exists(tex):
            sys.exit(f'missing number marker texture: {tex}')
        assets.append(f'    <texture type="2d" name="marker_{number}" file="{tex}"/>')
        assets.append(f'    <material name="marker_{number}" texture="marker_{number}" '
                      f'texrepeat="1 1" texuniform="false"/>')

    # ── Static arena shell ──
    body = [
        '    <!-- The arena\'s four directional lights. All four are needed, not just',
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
        '    <!-- Ground plane, centred on the arena. The size',
        '         bounds what is drawn; a MuJoCo plane is infinite for collision. -->',
        '    <geom name="arena_floor" type="plane" pos="1 0 0" size="5 5 0.05" '
        'material="arena_floor"/>',
        '    <!-- Painted arena boundary and start zone: visual only. -->',
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
        '    <!-- Collection bin: a free body, resting on the table. It never sleeps',
        '         so that a book placed in it never does either (a sleeping body has',
        '         no contacts with anything static or asleep, and /bin_contacts is',
        '         how a placement is judged). The books on the shelf do sleep. -->',
        f'    <body name="erc_collection_bin" pos="{fmt(BIN_POSE)}" quat="{PROP_QUAT}" sleep="never">',
        '      <freejoint name="erc_collection_bin_joint"/>',
        f'      <inertial pos="{BIN_COM}" mass="{BIN_MASS:g}" '
        f'fullinertia="{BIN_INERTIA}"/>',
        visual_mesh('erc_collection_bin', 'erc_collection_bin', (1, 0, 0, 1)),
    ]
    for name, pos, half in BIN_COLLISION:
        body.append(f'      <geom name="{name}" class="arena_collision" type="box" '
                    f'pos="{fmt(pos)}" size="{fmt(half)}"/>')
    body.append('    </body>')
    body.append('')

    # ── Books and markers: the draw order is part of the arena spec, so a given
    #    ERC_SEED reproduces the organisers' layout. Round 1 (upstream
    #    simulation_round_1.launch.py): five columns each shuffle their four
    #    colours then jitter each book's y; then the markers are shuffled.
    #    Round 2 (simulation_round_2.launch.py): the white books' order is
    #    shuffled first, then the columns draw as round 1 except that column 2
    #    draws no shuffle (all white) and column 4 no jitter (flush to the
    #    walls); the markers are NOT shuffled (column n carries n). ──
    if rnd == 2:
        white_books = list(WHITE_BOOKS)
        random.shuffle(white_books)
    pitch = ROUND2_PITCH if rnd == 2 else COLUMN_WIDTH
    offsets = [((NUM_COLUMNS - 1) / 2 - c) * pitch for c in range(NUM_COLUMNS)]
    body.append(f'    <!-- Books: round {rnd}, one per active shelf row, drawn from ERC_SEED. -->')
    for col in range(NUM_COLUMNS):
        shelf_num = col + 1
        white = rnd == 2 and shelf_num == WHITE_COLUMN
        edge = rnd == 2 and shelf_num == EDGE_COLUMN
        if white:
            colours_this_column = ['white'] * len(ACTIVE_ROWS)
        else:
            colours_this_column = list(BOOK_COLOURS.keys())
            random.shuffle(colours_this_column)
        for i, row in enumerate(ACTIVE_ROWS):
            colour = colours_this_column[i]
            if edge:
                # left, right, left, right from the top row, as upstream; flush
                # against the mesh's own divider face (see SHELF_DIVIDER_PITCH)
                side = 1 if i % 2 == 0 else -1
                centre = SHELF_Y + ((NUM_COLUMNS - 1) / 2 - col) * SHELF_DIVIDER_PITCH
                y = centre + side * (COLUMN_WIDTH / 2 - BOOK_HALF[1] - EDGE_CLEARANCE)
            else:
                y = (SHELF_Y + offsets[col]
                     + random.uniform(-COLUMN_JITTER_RANGE, COLUMN_JITTER_RANGE))
            z = SHELF_Z + ROW_FLOOR_Z_OFFSETS[row]
            x = SHELF_X - 0.1
            if white:
                cover, spine = white_books[i]
                name = f'book_col_{shelf_num}_row_{row + 1}_white_{cover}'
                body += book_body(name, (x, y, z), 'book_white', book_mu,
                                  white_prints(name, cover, spine))
            else:
                name = f'book_col_{shelf_num}_row_{row + 1}_{colour}'
                body += book_body(name, (x, y, z), f'book_{colour}', book_mu)
    body.append('')

    # ── Number markers: static, visual only. ──
    shelf_number_markers = list(range(1, NUM_COLUMNS + 1))
    if rnd == 2:
        body.append('    <!-- Number markers on the top row: column n carries n (round 2). -->')
    else:
        body.append('    <!-- Number markers on the top row, order drawn from ERC_SEED. -->')
        random.shuffle(shelf_number_markers)
    for col in range(NUM_COLUMNS):
        number = shelf_number_markers[col]
        y = SHELF_Y + offsets[col]
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
  <!-- GENERATED by erc_bringup/scripts/generate_mujoco_world.py - do not edit. -->

  <!-- Sized for the working volume around the robot, not the whole 10 m arena:
       the camera near plane is a fraction of this extent, so a large extent
       would push it out past the table and clip everything the robot reaches. -->
  <statistic center="1 0 0.8" extent="2"/>

  <!-- The robot's own <option> repeats these and wins on merge (raw_inputs are
       appended after the scene). They are here so the arena is also correct
       when loaded on its own: under the Euler/pyramidal defaults a resting
       book sinks 6.4 mm into a shelf, against 0.1 mm with these. -->
  <option integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="5">
    <flag multiccd="enable" sleep="enable"/>
  </option>

  <!-- The offscreen framebuffer defaults to 640x480, which is smaller than the
       spectator camera; anything rendering it offscreen fails without this.
       Merges with the robot's own <visual>, which sets the near/far planes. -->
  <visual>
    <global offwidth="960" offheight="540"/>
    <!-- The arena's nominal ambient of 0.7 cannot be copied across literally:
         MuJoCo sums ambient and diffuse differently, and four directional
         lights at the nominal 0.5 plus that ambient saturate every upward face
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
    ap.add_argument('--round', type=int, choices=(1, 2),
                    default=int(os.environ.get('ERC_ROUND') or 1),
                    help='competition round layout (default: $ERC_ROUND or 1)')
    ap.add_argument('--book-mu', type=float,
                    default=float(os.environ.get('ERC_BOOK_MU') or BOOK_MU),
                    help=f'book sliding friction (default: $ERC_BOOK_MU or {BOOK_MU})')
    args = ap.parse_args()

    share = get_package_share_directory('erc_description')
    xml = build_world(share, args.seed, args.round, args.book_mu)
    with open(args.output, 'w') as f:
        f.write(xml)
    print(f'[generate_mujoco_world] wrote {args.output} '
          f'({len(xml):,} bytes, seed={args.seed}, round={args.round}, book_mu={args.book_mu:g})')


if __name__ == '__main__':
    main()
