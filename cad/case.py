"""CO2 monitor case, design v3 (docs section 13): base and lid, built with build123d.

Run from the project folder:
    cad\\.venv\\Scripts\\python.exe cad\\case.py

Writes cad/out/:
    base.step, lid.step   the files for the school's printer (one STEP per part, mm)
    *.stl                 meshes of every part, for Cura and the viewer
    viewer.html           open it in a browser to turn the model around

Coordinates are the ones of the docs' case drawings (the parameter object P in
section 13): x along the breadboard rows (0 = inside of the back wall at the USB
end), y across (0 = middle of the breadboard, minus = column-a side with the
sensor chamber), z up (0 = underside of the floor). All sizes in millimetres.

Sensors: the BMP580, SCD-41 and SHT45 stand on standoffs in the chamber, each screw
behind a side slot. The VEML7700 (5378, right angle) lies in a pocket in the floor
under the display and looks out through a hole in the front wall. The OLED stands
in a holder behind its window; the speaker hangs under the lid.

Not modelled yet, because the design is still open (docs "Still open"):
mesh groove, LED hole.
"""

import base64
import itertools
import json
import math
from datetime import datetime
from pathlib import Path

import build123d
from build123d import (Align, Axis, Box, Circle, Compound, Cylinder, Edge, Keep, Plane, Pos, Rot,
                       export_step, export_stl, fillet, import_step, sweep)

HERE = Path(__file__).parent
OUT = HERE / "out"
ADAFRUIT = HERE / "adafruit"

# ---- printing (docs "Printing at school")
LAYER = 0.254              # Stratasys F370, standard layer
FIT = 0.3                  # gap around parts that drop in; checked with the test piece

# ---- shell
WALL = 2.0                 # outer walls
FLOOR = 8 * LAYER          # 2.03, whole layers
LID_T = 8 * LAYER
CORNER_R = 2.0             # outer vertical edges
X0, X1, Y0, Y1 = -2.0, 103.0, -54.0, 32.0   # outside of the walls
LID_Z = 38.0               # top of the base = underside of the lid

# ---- inside: electronics bay, sensor chamber, double wall between them
BAY_Y = (-30.0, 30.0)
DOUBLE_WALL = ((-34.0, -32.8), (-31.2, -30.0))   # 2 x 1.2 with a 1.6 air gap

# ---- breadboard, measured (B1-B3): body 83 x 55 x 9.3; joining nubs 1.5 past the
# row-1 (USB) end and 0.7 past the column-a side. The nubs touch the back wall.
BB_L, BB_W, BB_H = 83.0, 55.0, 9.3
BB_NUB_END, BB_NUB_SIDE = 1.5, 0.7
BB_X0 = BB_NUB_END
BB_X1 = BB_X0 + BB_L                   # 84.5
POCKET_H = 8 * LAYER                   # rim that locates the breadboard

# ---- ESP32-C6-DevKitC-1 in rows 3-18. Its USB-C ports stick out 2.5 past the
# breadboard body (E4); their top is 16.5 above the breadboard's bottom (E3).
USB_Y = (7.2, -7.2)                    # UART, USB (from the column-a edge: 34.7, 20.3)
USB_TOP = FLOOR + 16.5
USB_Z = USB_TOP - 1.6                  # middle of a 3.2 mm USB-C port
# Room for each plug's grip: 12.5 x 7.5 around each port. The two overlap, so they
# are cut as one opening. Small corner radius: the ESP32 board's edge reaches into it.
USB_CUT_W, USB_CUT_H, USB_CUT_R = 12.5, 7.5, 1.0
ESP_SHIFT = BB_NUB_END - 2.54          # vs the v3 drawings: board moved +1.5, ESP32 one row back

# ---- lid screws: 4 x M3 x 10, self-tapping. Each boss fills an inside corner of the
# case. The one at the back of the column-j side can't reach the floor (breadboard),
# so it hangs from the walls with a 45 degree underside (prints without support).
CORNERS = [(0.0, -52.0, 1, 1), (101.0, -52.0, -1, 1), (0.0, 30.0, 1, -1), (101.0, 30.0, -1, -1)]   # x, y, into the case
HANGING_CORNER = 2
BOSS_INSET, BOSS_R = 2.0, 3.0          # hole centre from both walls; rounding towards the inside
BOSSES = [(x + sx * BOSS_INSET, y + sy * BOSS_INSET) for x, y, sx, sy in CORNERS]
BOSS_PILOT, BOSS_HOLE_DEPTH = 1.8, 12.0    # drill to 2.5
LID_PILOT = 2.5                            # drill to 3.2
HANGING_BOSS_Z = 15.0                      # its lowest point, in the corner

# ---- stick-on rubber feet, bought (e.g. 3M Bumpon SJ5012: dia 12.7 x 3.5), so the
# floor prints flat on the build plate. At least 3 mm tall: air reaches the floor vents.
FOOT_D, FOOT_H = 12.7, 3.5
FEET = [(x, y) for x in (X0 + 7.5, X1 - 7.5) for y in (Y0 + 7.5, Y1 - 7.5)]   # centres

# ---- vents. The same 4 mm slots everywhere, in two equal rows centred on the base wall.
# Chamber side wall: 13 columns, a third of the sensor screws' 20.32 spacing apart, centred.
# The column-j wall is the electronics' only vent: cool air in low, warm air out high. It
# has only the first 8 columns, beside the ESP32, where the warmth comes from.
SLOT_W, SLOT_PITCH, SLOT_COLS = 4.0, 20.32 / 3, 13
SLOT_X = [(X0 + X1) / 2 + (k - (SLOT_COLS - 1) / 2) * SLOT_PITCH for k in range(SLOT_COLS)]
SLOT_ROWS = ((5.5, 17.5), (20.5, 32.5))
BAY_SLOT_COLS = 8
# Back of the chamber: 2 columns between the corner boss and the double wall. The front
# (display side) has none.
END_SLOT_MID = (Y0 + WALL + BOSS_INSET + BOSS_R + DOUBLE_WALL[0][0]) / 2
END_SLOT_Y = (END_SLOT_MID - SLOT_PITCH / 2, END_SLOT_MID + SLOT_PITCH / 2)
FLOOR_VENT_X = SLOT_X[1:-1]                    # under the chamber, between the rubber feet
FLOOR_VENT_W, FLOOR_VENT_Y = 2.0, (-50.0, -42.0)   # in front of the sensor standoffs

# ---- front face (x = 101..103)
OLED_WIN = (-25.5, 16.5, -0.5, 30.5)           # y0, z0, y1, z1
BUTTON_Y, BUTTON_Z, BUTTON_PILOT = 14.0, 23.5, 6.0   # level with the middle of the display; drill to 7.0 (M7)

# ---- OLED (Joy-it SBC-OLED01, measured O1-O7). It stands behind its window, its glass in a
# recess on the inside of the front wall, so the glass is only 1 mm behind the outside. The
# recess is open at the top: the OLED slides in from above. Two rails hold its side edges,
# two small shelves carry its bottom edge (left and right of the flex strip's notch), and
# two pegs on the lid stop it lifting. Nothing of this shows on the outside.
OLED_W, OLED_H = 27.75, 28.0
OLED_Y0, OLED_Z0 = -26.75, 7.4                 # board edge on the column-a side, bottom edge
OLED_GLASS = (26.75, 19.15, 1.8, 4.4)          # width, height, thickness, top edge below the board's top
OLED_RECESS = 1.0                              # glass this far into the 2 mm wall: 1 mm of wall left in front
OLED_BOARD_T = 1.2                             # assumed: not measured yet (O3)
OLED_X = X1 - WALL + OLED_RECESS - OLED_GLASS[2]   # 100.2: front face of the board
OLED_PEGS, OLED_PEG_D = (-23.5, -2.25), 2.4    # lid pegs on the board's top edge, clear of the header
OLED_HOLE_D, OLED_HOLE_IN = 1.5, (1.9, 2.1)    # from the sides, from top and bottom
OLED_NOTCH = (14.5, 2.0)                       # flex strip, middle of the bottom edge
OLED_PIN_DOWN, OLED_PIN_BACK = 1.5, 6.0        # pin row below the top edge; pins behind the plastic
JUMPER_L = 14.0                                # female jumper-wire plugs on the pins
RAIL_T, RAIL_LIP, RAIL_Z = 1.2, 1.0, 10.0      # thickness, lip over the board's back, lowest point (at the wall)
SHELVES = ((-24.5, -21.0), (-5.0, -1.5))       # y ranges under the board's bottom edge

# ---- sensor boards (Adafruit, 25.4 x 17.78; header edge down, parts facing the outer wall)
BOARD_T = 1.57
BOARD_BACK_Y = -40.0                   # back face of the boards; standoffs between it and the double wall
STANDOFF_W = 5.0
STANDOFF_PILOT = 1.6                   # teardrop, drilled to 2.1 through the slot; M2.5 x 6
STANDOFF_PILOT_DEPTH = 5.0
HOLE_IN = 2.54                         # mounting holes from the board edges
BOARD_Z = 7.0                          # bottom edge of the SCD-41 and SHT45
SCD_X = SLOT_X[4] - HOLE_IN            # screws behind slot columns 5 and 8 (of 13)
SHT_X = SLOT_X[9] - HOLE_IN            # columns 10 and 13: the front, farthest from the ESP32
BOARD_GAP = SHT_X - SCD_X - 25.4       # 8.47, room for two STEMMA QT plugs
# The BMP580 stands on its side behind the SCD-41: turned a quarter, so that its two holes
# (both on one long edge) are one above the other, behind slot column 3, one in each row.
BMP_X = SLOT_X[2] - (17.78 - HOLE_IN)  # 8.16; the holes' edge faces the SCD-41
BMP_Z = 4.6                            # bottom edge, low: the cable goes into the socket at the top

# VEML7700 right angle (5378): lies in a pocket in the floor under the display and looks out
# through a hole in the front wall. It needs 17.8 mm in front of the breadboard where 16.2
# are left, so its back edge goes 1.4 mm under the breadboard: the pocket sinks it almost
# to the level of the floor, and the breadboard's front end rests on it and holds it down.
VEML_Y = -24.3                         # board edge on the column-a side, clear of the breadboard stop
VEML_Z = 3 * LAYER                     # board bottom: 3 layers of floor under it; its top 0.3 above the floor
VEML_X = X1 - WALL - 0.1 - 17.78       # 83.12: back edge; the front edge is 0.1 from the wall
VEML_SENSOR = (9.26, 16.16, 2.5)       # sensor: from and to (from the board's column-a edge), height
VEML_WINDOW_GAP = 0.3                  # the window is the sensor's face plus this all round

# ---- STEMMA QT cables: 50 mm plug to plug. Wire between the plugs, assumed: 50 - 2 x 4.5.
CABLE_WIRE, CABLE_D = 41.0, 1.6
PLUG_OUT = 3.0                         # plug body outside the socket

# ---- cable hole: at the bottom of the double wall, at its front end, so the cable from the
# breadboard reaches the SHT45's front socket. A STEMMA QT plug (about 6 x 3) passes through.
# Pointed top (45 degrees): prints without support. Seal it after wiring.
CABLE_HOLE_X = (SLOT_X[12] + STANDOFF_W / 2 + 0.05, X1 - WALL)   # 93.7..101, beside the SHT45 standoff
CABLE_HOLE_H = 4.0                     # straight part, above the floor

# ---- STEMMA speaker (3885): hangs under the lid on 4 standoffs, speaker up against the grille.
# Board 30.48 x 36.83 with M2 holes (2.2) 2.54 from the edges; its JST PH socket faces column j.
SPK_X, SPK_Y = 2.0, 10.0               # back edge, edge with the socket
SPK_Z = LID_Z - 0.5 - 6.57             # board bottom: the speaker's top 0.5 under the lid
SPK_HOLES = [(2.54, 2.54), (27.94, 2.54), (2.54, 34.29), (27.94, 34.29)]   # on the board
SPK_STANDOFF_D, SPK_PILOT = 4.5, 1.2   # drill to 1.6 for M2 x 6
GRILLE_D = 1.8                                 # over the speaker, 7 x 5 holes
GRILLE = [(SPK_X + 30.48 - 15.39 + 4 * i, SPK_Y - 21.54 + 4 * j) for i in range(-3, 4) for j in range(-2, 3)]

# ---- other parts, for the fit check only
BUTTON_D, BUTTON_DEPTH = 9.0, 25.0     # panel button, inside the case


def block(x0, y0, z0, x1, y1, z1):
    """Box between two opposite corners."""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def rod(x, y, z0, z1, d):
    """Vertical cylinder of diameter d from z0 to z1."""
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def rounded_z(shape, r):
    return fillet(shape.edges().filter_by(Axis.Z), r)


def teardrop_y(x, z, y0, y1, d, up=1):
    """Horizontal hole along y with a 45 degree point, so it prints without support.
    up = +1 points it to +z (base, printed upright), -1 to -z (lid, printed upside down)."""
    r = d / 2
    hole = Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)
    return hole + Pos(x, (y0 + y1) / 2, z + up * r / math.sqrt(2)) * Rot(0, 45, 0) * Box(r, y1 - y0, r)


def hanging(shape, x_wall, z_low, into):
    """Cut the underside of a part on a wall at 45 degrees, rising away from the wall
    (into = +1 or -1 in x), so it prints without support. z_low: its lowest point, at the wall."""
    return shape.split(Plane(origin=(x_wall, 0, z_low), z_dir=(-into, 0, 1)), keep=Keep.TOP)


def corner_boss(x, y, sx, sy, z0, z1):
    """Lid-screw boss filling an inside corner, rounded towards the inside."""
    size = BOSS_INSET + BOSS_R
    boss = block(min(x, x + sx * size), min(y, y + sy * size), z0, max(x, x + sx * size), max(y, y + sy * size), z1)
    far = (x + sx * size, y + sy * size)
    edge = min(boss.edges().filter_by(Axis.Z), key=lambda e: (e.center().X - far[0]) ** 2 + (e.center().Y - far[1]) ** 2)
    return fillet([edge], BOSS_R)


def upright(shape, x, z, y=BOARD_BACK_Y):
    """Stand a board model up: header edge down at z, parts towards the outer wall."""
    return Pos(x, y, z) * Rot(90, 0, 0) * shape


def board_screws(x, z, height, holes=(HOLE_IN, 25.4 - HOLE_IN)):
    """Screw positions (x, z) of an upright board using the 2 holes away from its header."""
    return [(x + hx, z + height - HOLE_IN) for hx in holes]


def sensor_screws():
    """(board, x, z) of every sensor screw. All are turned from outside, through a side slot."""
    screws = [("SCD-41", x, z) for x, z in board_screws(SCD_X, BOARD_Z, 22.86)]
    screws += [("SHT45", x, z) for x, z in board_screws(SHT_X, BOARD_Z, 17.78)]
    screws += [("BMP580", BMP_X + 17.78 - HOLE_IN, BMP_Z + hz) for hz in (HOLE_IN, 25.4 - HOLE_IN)]
    return screws


def speaker_holes():
    """Speaker screw positions (x, y) in the case: the board is turned half round."""
    return [(SPK_X + 30.48 - hx, SPK_Y - hy) for hx, hy in SPK_HOLES]


def make_base():
    base = rounded_z(block(X0, Y0, 0, X1, Y1, LID_Z), CORNER_R)
    base -= block(X0 + WALL, Y0 + WALL, FLOOR, X1 - WALL, Y1 - WALL, LID_Z + 1)

    for y0, y1 in DOUBLE_WALL:
        base += block(0, y0, FLOOR, X1 - WALL, y1, LID_Z)
    hx0, hx1 = CABLE_HOLE_X
    w = hx1 - hx0
    base -= block(hx0, DOUBLE_WALL[0][0] - 1, FLOOR, hx1, DOUBLE_WALL[1][1] + 1, FLOOR + CABLE_HOLE_H)
    base -= Pos((hx0 + hx1) / 2, sum(DOUBLE_WALL[0] + DOUBLE_WALL[1]) / 4, FLOOR + CABLE_HOLE_H) * Rot(0, 45, 0) * Box(
        w / math.sqrt(2), 6, w / math.sqrt(2))

    # breadboard pocket: rims along both long sides, stops at the front corners
    a_edge = -BB_W / 2 - BB_NUB_SIDE - FIT
    j_edge = BB_W / 2 + FIT
    front = BB_X1 + FIT
    top = FLOOR + POCKET_H
    base += block(0, BAY_Y[0], FLOOR, front, a_edge, top)
    base += block(0, j_edge, FLOOR, front, BAY_Y[1], top)
    base += block(front, BAY_Y[0], FLOOR, front + 1.6, a_edge + 4, top)
    base += block(front, j_edge - 4, FLOOR, front + 1.6, BAY_Y[1], top)

    # light sensor: pocket in the floor, window in the front wall
    base -= block(VEML_X - 0.2, VEML_Y - 0.2, VEML_Z, X1 - WALL, VEML_Y + 25.4 + 0.2, FLOOR + 0.5)
    s0, s1, sh = VEML_SENSOR
    g = VEML_WINDOW_GAP
    # its bottom edge (sensor bottom less the gap) is the floor's top
    base -= block(X1 - WALL - 0.5, VEML_Y + s0 - g, FLOOR, X1 + 1, VEML_Y + s1 + g, VEML_Z + BOARD_T + sh + g)

    for i, ((x, y, sx, sy), (bx, by)) in enumerate(zip(CORNERS, BOSSES)):
        if i == HANGING_CORNER:
            boss = corner_boss(x, y, sx, sy, HANGING_BOSS_Z, LID_Z)
            for normal in ((-sx, 0, 1), (0, -sy, 1)):
                boss = boss.split(Plane(origin=(x, y, HANGING_BOSS_Z), z_dir=normal), keep=Keep.TOP)
        else:
            boss = corner_boss(x, y, sx, sy, FLOOR, LID_Z)
        base += boss
        base -= rod(bx, by, LID_Z - BOSS_HOLE_DEPTH, LID_Z + 1, BOSS_PILOT)

    # standoffs for the sensor boards, against the double wall (the BMP580's two are one pillar)
    for _, sx, sz in sensor_screws():
        base += block(sx - STANDOFF_W / 2, BOARD_BACK_Y, FLOOR, sx + STANDOFF_W / 2, DOUBLE_WALL[0][0], sz + 3)
    for _, sx, sz in sensor_screws():
        base -= teardrop_y(sx, sz, BOARD_BACK_Y - 0.5, BOARD_BACK_Y + STANDOFF_PILOT_DEPTH, STANDOFF_PILOT)

    # OLED holder: the recess for its glass (open at the top), and, hanging from the front wall,
    # rails with a lip behind the board's side edges and shelves under its bottom edge
    x_wall = X1 - WALL
    gw, gh, _, gdown = OLED_GLASS
    ym = OLED_Y0 + OLED_W / 2
    base -= block(x_wall - 1, ym - gw / 2 - 0.2, OLED_Z0 + OLED_H - gdown - gh - 0.3, x_wall + OLED_RECESS,
                  ym + gw / 2 + 0.2, LID_Z + 1)
    x_lip = OLED_X - OLED_BOARD_T - FIT                        # front face of the lips
    for side, edge in ((-1, OLED_Y0), (1, OLED_Y0 + OLED_W)):
        stem = sorted((edge + side * 0.2, edge + side * (0.2 + RAIL_T)))
        lip = sorted((edge + side * 0.2, edge - side * RAIL_LIP))
        rail = block(x_lip - RAIL_T, stem[0], RAIL_Z, x_wall, stem[1], LID_Z - 1)
        rail += block(x_lip - RAIL_T, lip[0], RAIL_Z, x_lip, lip[1], LID_Z - 1)
        base += hanging(rail, x_wall, RAIL_Z, -1)
    shelf_back = x_lip - FIT
    for y0, y1 in SHELVES:
        shelf = block(shelf_back, y0, FLOOR, x_wall, y1, OLED_Z0)
        base += hanging(shelf, x_wall, OLED_Z0 - 0.8 - (x_wall - shelf_back), -1)

    for z0, z1 in SLOT_ROWS:
        for x in SLOT_X:
            base -= block(x - SLOT_W / 2, Y0 - 1, z0, x + SLOT_W / 2, Y0 + WALL + 0.5, z1)
        for x in SLOT_X[:BAY_SLOT_COLS]:
            base -= block(x - SLOT_W / 2, Y1 - WALL - 0.5, z0, x + SLOT_W / 2, Y1 + 1, z1)
        for y in END_SLOT_Y:
            base -= block(X0 - 1, y - SLOT_W / 2, z0, X0 + WALL + 0.5, y + SLOT_W / 2, z1)
    for x in FLOOR_VENT_X:
        base -= block(x - FLOOR_VENT_W / 2, FLOOR_VENT_Y[0], -1, x + FLOOR_VENT_W / 2, FLOOR_VENT_Y[1], FLOOR + 0.5)

    cut = block(X0 - 1, min(USB_Y) - USB_CUT_W / 2, USB_Z - USB_CUT_H / 2,
                1, max(USB_Y) + USB_CUT_W / 2, USB_Z + USB_CUT_H / 2)
    base -= fillet(cut.edges().filter_by(Axis.X), USB_CUT_R)

    y0, z0, y1, z1 = OLED_WIN
    base -= block(X1 - WALL - 1, y0, z0, X1 + 1, y1, z1)
    base -= Pos(X1 - WALL - 1, BUTTON_Y, BUTTON_Z) * Rot(0, 90, 0) * Cylinder(
        BUTTON_PILOT / 2, WALL + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))

    base.label = "base"
    return base


def make_lid():
    lid = rounded_z(block(X0, Y0, LID_Z, X1, Y1, LID_Z + LID_T), CORNER_R)
    for x, y in GRILLE:
        lid -= rod(x, y, LID_Z - 1, LID_Z + LID_T + 1, GRILLE_D)
    for x, y in BOSSES:
        lid -= rod(x, y, LID_Z - 1, LID_Z + LID_T + 1, LID_PILOT)

    # speaker standoffs; the screws go in from below with the lid upside down on the table
    board_top = SPK_Z + BOARD_T
    for x, y in speaker_holes():
        lid += rod(x, y, board_top, LID_Z, SPK_STANDOFF_D)
        lid -= rod(x, y, board_top - 0.5, board_top + 5.0, SPK_PILOT)

    # pegs that hold the OLED down, 0.2 above its top edge
    for y in OLED_PEGS:
        lid += rod(OLED_X - OLED_BOARD_T / 2, y, OLED_Z0 + OLED_H + 0.2, LID_Z, OLED_PEG_D)

    lid.label = "lid"
    return lid


def cable(a, b, wire=CABLE_WIRE, bulge_y=-48.0, up=True, d=CABLE_D):
    """Cable between the plugs of two neighbouring boards, which face each other: from
    plug end a (cable leaves towards -x) to plug end b (leaves towards +x). The spare wire
    loops up (or down) and out towards the outer wall (to y = bulge_y); the loop grows
    until the cable is `wire` long."""
    V = build123d.Vector
    a, b = V(*a), V(*b)
    s = 1 if up else -1
    far = max(a.Z, b.Z) if up else min(a.Z, b.Z)

    def path(h):
        pts = [a, V(a.X - 1.5, (a.Y + bulge_y) / 2, a.Z + s * h / 3), V((a.X + b.X) / 2, bulge_y, far + s * h),
               V(b.X + 1.5, (b.Y + bulge_y) / 2, b.Z + s * h / 3), b]
        return Edge.make_spline(pts, tangents=[(-1, 0, 0), (-1, 0, 0)])

    return grow(path, wire, d, a, (-1, 0, 0))


def cable_over(a, b, wire=CABLE_WIRE, bulge_y=-47.5, d=CABLE_D):
    """Cable from plug end a (cable leaves towards -x) up and over to plug end b on the top
    edge of the board behind it (cable leaves upwards), bulging out towards the outer wall.
    The spare wire dips down a little in the gap between the boards; the dip grows until
    the cable is `wire` long. Keep it low: there are only 5.2 mm between b and the lid."""
    V = build123d.Vector
    a, b = V(*a), V(*b)
    top = b.Z + 0.4

    def path(h):
        pts = [a, V(a.X - 2.2, (a.Y + bulge_y) / 2, a.Z + 1.2 - h), V(a.X - 3.2, bulge_y, (a.Z + top) / 2 - h / 2),
               V((a.X - 3.2 + b.X) / 2 + 1, bulge_y, top), V(b.X + 0.8, (a.Y + bulge_y) / 2, top - 0.6), b]
        return Edge.make_spline(pts, tangents=[(-1, 0, 0), (0, 0, -1)])

    return grow(path, wire, d, a, (-1, 0, 0), h_max=3.0)


def grow(path, wire, d, start, start_dir, h_max=30.0):
    """Find the loop size h (0..h_max) that makes path(h) `wire` long; returns the tube and its path."""
    lo, hi = 0.0, h_max
    for _ in range(40):
        h = (lo + hi) / 2
        lo, hi = (h, hi) if path(h).length < wire else (lo, h)
    edge = path((lo + hi) / 2)
    return sweep(Plane(origin=start, z_dir=start_dir) * Circle(d / 2), path=edge), edge


def routed_cable(points, start_dir, d=CABLE_D):
    """Cable through fixed points (for the long cables to the breadboard). Parameters by
    distance and unscaled end directions: the curve stays close to the points, without loops."""
    pts = [build123d.Vector(*p) for p in points]
    par = [0.0] + list(itertools.accumulate((b - a).length for a, b in zip(pts, pts[1:])))
    edge = Edge.make_spline(pts, tangents=[start_dir, (0, 0, -1)], parameters=par, scale=False)
    return sweep(Plane(origin=points[0], z_dir=start_dir) * Circle(d / 2), path=edge), edge


def plug(x_open, out, y_mid, z_mid, horizontal=True):
    """Plug body outside a socket opening at x_open, sticking out in direction out (+1/-1)."""
    x_end = x_open + out * PLUG_OUT
    if horizontal:    # upright board: socket 6.0 along z, 2.96 along y
        return block(min(x_open, x_end), y_mid - 1.4, z_mid - 2.8, max(x_open, x_end), y_mid + 1.4, z_mid + 2.8)
    return block(min(x_open, x_end), y_mid - 2.8, z_mid - 1.4, max(x_open, x_end), y_mid + 2.8, z_mid + 1.4)


def make_sensors():
    """The sensor boards, plugs and cables. Returns parts and the cable lengths."""
    scd = upright(import_step(ADAFRUIT / "5187 SCD-40 C02 Sensor.step"), SCD_X, BOARD_Z)
    sht = upright(import_step(ADAFRUIT / "5665 SHT45 Sensor.step"), SHT_X, BOARD_Z)
    # BMP390 model, standing in for the BMP580; turned a quarter in its own plane
    bmp = upright(Pos(0, 25.4, 0) * Rot(0, 0, -90) * import_step(ADAFRUIT / "4816 BMP390 QT.step"), BMP_X, BMP_Z)
    # the 5378 lies parts up, its sensor at the front wall
    veml = Pos(VEML_X, VEML_Y + 25.4, VEML_Z) * Rot(0, 0, -90) * import_step(ADAFRUIT / "5378 VEML7700 Lux Sensor.step")

    y_sock = BOARD_BACK_Y - BOARD_T - 1.48     # middle of a socket, in front of an upright board
    # socket openings (x) and middles (z): (board x + opening, board z + socket middle)
    sht_back = (SHT_X + 0.70, BOARD_Z + 8.89)
    sht_front = (SHT_X + 24.70, BOARD_Z + 8.89)
    scd_front = (SCD_X + 25.34, BOARD_Z + 11.43)
    scd_back = (SCD_X + 0.06, BOARD_Z + 11.43)
    bmp_top = (BMP_X + 8.89, BMP_Z + 25.21)                    # middle (x), opening (z)
    veml_side = (VEML_X + 8.89, VEML_Y + 25.21, VEML_Z + BOARD_T + 1.48)   # middle (x), opening (y), middle (z)

    plugs = [plug(x, out, y_sock, z) for (x, z), out in
             ((sht_front, 1), (sht_back, -1), (scd_front, 1), (scd_back, -1))]
    bx, bz = bmp_top
    plugs.append(block(bx - 2.8, y_sock - 1.4, bz, bx + 2.8, y_sock + 1.4, bz + PLUG_OUT))
    vx, vy, vz = veml_side
    plugs.append(block(vx - 2.8, vy, vz - 1.4, vx + 2.8, vy + PLUG_OUT, vz + 1.4))

    lengths = {}
    tubes = []
    for name, (tube, edge) in (
            ("SHT45 - SCD-41", cable((sht_back[0] - PLUG_OUT, y_sock, sht_back[1]),
                                     (scd_front[0] + PLUG_OUT, y_sock, scd_front[1]))),
            ("SCD-41 - BMP580", cable_over((scd_back[0] - PLUG_OUT, y_sock, scd_back[1]),
                                           (bx, y_sock, bz + PLUG_OUT)))):
        tubes.append(tube)
        zs = [edge.position_at(i / 200).Z for i in range(201)]   # the bounding box of a spline is too big
        lengths[name] = (edge.length, min(zs) - CABLE_D / 2, max(zs) + CABLE_D / 2)

    # long STEMMA QT to male-pin cables, into the breadboard (rows 27-30, middle x 76):
    # the sensor chain through the hole at the bottom of the double wall, into d27-d30;
    # the light sensor round the back of the display, into b27-b30
    hole_x = sum(CABLE_HOLE_X) / 2 - 0.5
    entry, _ = routed_cable([(sht_front[0] + PLUG_OUT, y_sock, sht_front[1]), (98.6, -41.5, 12.5), (98.3, -37.5, 7.0),
                             (hole_x, -32.0, FLOOR + 2.3), (95.0, -29.0, 5.0), (91.0, -28.6, 7.0), (86.0, -28.0, 11.0),
                             (82.0, -24.0, 15.0), (78.5, -14.0, 17.5), (76.5, -8.5, 17.0), (76.0, -6.35, 15.0)], (1, 0, 0))
    light, _ = routed_cable([(vx, vy + PLUG_OUT, vz), (vx, 6.5, 5.5), (89.5, 8.0, 10.5), (85.0, 4.0, 15.0),
                             (80.0, -5.0, 17.0), (76.5, -10.0, 16.0), (76.0, -11.43, 14.0)], (0, 1, 0))
    tubes += [entry, light]

    sensors = {"scd41": scd, "sht45": sht, "bmp580": bmp, "veml7700": veml,
               "plugs": Compound(plugs), "cables": Compound(tubes)}
    return sensors, lengths


def make_oled():
    """Joy-it SBC-OLED01 from the measurements (O1-O7), with the 4 jumper-wire plugs on its pins."""
    x_glass = OLED_X
    x_back = x_glass - OLED_BOARD_T
    y0, y1, z0, z1 = OLED_Y0, OLED_Y0 + OLED_W, OLED_Z0, OLED_Z0 + OLED_H
    ym = (y0 + y1) / 2
    board = block(x_back, y0, z0, x_glass, y1, z1)
    for hy in (y0 + OLED_HOLE_IN[0], y1 - OLED_HOLE_IN[0]):
        for hz in (z0 + OLED_HOLE_IN[1], z1 - OLED_HOLE_IN[1]):
            board -= Pos(x_back - 1, hy, hz) * Rot(0, 90, 0) * Cylinder(
                OLED_HOLE_D / 2, OLED_BOARD_T + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    nw, nd = OLED_NOTCH
    board -= block(x_back - 1, ym - nw / 2, z0 - 1, x_glass + 1, ym + nw / 2, z0 + nd)
    gw, gh, gt, gdown = OLED_GLASS
    glass = block(x_glass, ym - gw / 2, z1 - gdown - gh, x_glass + gt, ym + gw / 2, z1 - gdown)

    zp = z1 - OLED_PIN_DOWN
    x_plastic = x_back - 2.5
    header = block(x_plastic, ym - 5.08, zp - 1.25, x_back, ym + 5.08, zp + 1.25)
    pins = [block(x_plastic - OLED_PIN_BACK, ym + k * 2.54 - 0.32, zp - 0.32, x_glass + gt, ym + k * 2.54 + 0.32, zp + 0.32)
            for k in (-1.5, -0.5, 0.5, 1.5)]
    jumpers = block(x_plastic - JUMPER_L, ym - 5.08, zp - 1.27, x_plastic, ym + 5.08, zp + 1.27)
    return Compound([board, glass, header, jumpers] + pins)


def make_speaker():
    """Adafruit 3885 under the lid, speaker up, with the plug of its JST PH cable."""
    speaker = Pos(SPK_X + 30.48, SPK_Y, SPK_Z) * Rot(0, 0, 180) * import_step(ADAFRUIT / "3885 STEMMA Speaker rev A.step")
    xm = SPK_X + 30.48 - 15.24
    ph = block(xm - 4.0, SPK_Y - 0.27, SPK_Z + BOARD_T + 0.15, xm + 4.0, SPK_Y + 6.0, SPK_Z + BOARD_T + 4.65)
    return Compound(list(speaker.solids()) + [ph])


def make_references():
    """Not printed: the parts that go into the case, to check the fit."""
    breadboard = block(BB_X0, -BB_W / 2, FLOOR, BB_X1, BB_W / 2, FLOOR + BB_H)

    port_x0 = BB_X0 - 2.5                       # front of the USB ports
    pcb_x0 = port_x0 + 0.4
    pcb_top = USB_TOP - 3.2
    esp = [block(pcb_x0, -12.7, pcb_top - 1.6, pcb_x0 + 51.8, 12.7, pcb_top)]
    esp += [block(port_x0, y - 4.45, pcb_top, port_x0 + 7.3, y + 4.45, USB_TOP) for y in USB_Y]
    module = block(pcb_x0 + 32.6, -9, pcb_top, pcb_x0 + 58.1, 9, pcb_top + 3.2)

    feet = Compound([rod(x, y, -FOOT_H, 0, FOOT_D) for x, y in FEET])
    button = Pos(X1 - WALL - BUTTON_DEPTH, BUTTON_Y, BUTTON_Z) * Rot(0, 90, 0) * Cylinder(
        BUTTON_D / 2, BUTTON_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN))

    return {
        "breadboard": breadboard,
        "esp32": Compound(esp),
        "esp32_module": module,
        "oled": make_oled(),
        "button": button,
        "speaker": make_speaker(),
        "rubber_feet": feet,
    }


def overlap(a, b):
    """Volume of b inside a. Solid by solid: on a whole multi-part board model the
    boolean returns stray bits far outside the model."""
    try:
        return f"{sum((a & s).volume for s in b.solids()):.2f} mm3"
    except Exception as e:
        return f"not checked ({type(e).__name__})"


def check_slots():
    """Each standoff screw must sit behind a slot: the 2.3 mm screwdriver blade fits."""
    blade = 2.3 / 2
    for name, sx, sz in sensor_screws():
        col = min(SLOT_X, key=lambda c: abs(c - sx))
        ok_x = abs(col - sx) + blade <= SLOT_W / 2
        ok_z = any(z0 + blade <= sz <= z1 - blade for z0, z1 in SLOT_ROWS)
        print(f"  {name} screw at x {sx:.2f}, z {sz:.2f}: slot {SLOT_X.index(col) + 1}, "
              f"{'reachable' if ok_x and ok_z else 'NOT reachable'}")


def main():
    OUT.mkdir(exist_ok=True)
    base, lid = make_base(), make_lid()
    refs = make_references()
    sensors, lengths = make_sensors()

    for part in (base, lid):
        bb = part.bounding_box()
        print(f"{part.label:5} valid={part.is_valid}  {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.2f} mm"
              f"  volume {part.volume / 1000:.1f} cm3")
        export_step(part, OUT / f"{part.label}.step")
    print("overlap with the case:")
    for name, shape in {**refs, **sensors}.items():
        print(f"  {name:13} base {overlap(base, shape)}, lid {overlap(lid, shape)}")
    print(f"  base/lid      {overlap(base, lid)}")
    print("screws behind the side slots:")
    check_slots()
    print(f"STEMMA QT cables, {CABLE_WIRE:.0f} mm of wire between the plugs:")
    for name, (length, low, top) in lengths.items():
        print(f"  {name}: {length:.1f} mm, loop from z {low:.1f} to {top:.1f} (floor {FLOOR:.1f}, lid {LID_Z:.0f})")

    on_lid = {"lid", "speaker"}
    parts = [
        ("base", base, "#ddd5c0", 1.0, "print"),
        ("lid", lid, "#ddd5c0", 1.0, "print"),
        ("scd41", sensors["scd41"], "#3a7bd5", 1.0, "sensor"),
        ("sht45", sensors["sht45"], "#2da44e", 1.0, "sensor"),
        ("bmp580", sensors["bmp580"], "#8250df", 1.0, "sensor"),
        ("veml7700", sensors["veml7700"], "#d4a72c", 1.0, "sensor"),
        ("plugs", sensors["plugs"], "#f5f5f5", 1.0, "sensor"),
        ("cables", sensors["cables"], "#cf222e", 1.0, "sensor"),
        ("breadboard", refs["breadboard"], "#f5f1e8", 1.0, "ref"),
        ("esp32", refs["esp32"], "#2d333b", 1.0, "ref"),
        ("esp32_module", refs["esp32_module"], "#b8bec6", 1.0, "ref"),
        ("oled", refs["oled"], "#1d3557", 1.0, "ref"),
        ("button", refs["button"], "#e8a37e", 1.0, "ref"),
        ("speaker", refs["speaker"], "#d85a30", 1.0, "ref"),
        ("rubber_feet", refs["rubber_feet"], "#24292f", 1.0, "ref"),
    ]
    data = {"stamp": f"{datetime.now():%d %b %Y %H:%M}, build123d {build123d.__version__}", "parts": []}
    for name, shape, color, opacity, group in parts:
        path = OUT / f"{name}.stl"
        fine = group == "print"
        export_stl(shape, path, tolerance=0.02 if fine else 0.05, angular_tolerance=0.2 if fine else 0.4)
        data["parts"].append({
            "name": name, "color": color, "opacity": opacity, "group": group, "lid": name in on_lid,
            "stl": base64.b64encode(path.read_bytes()).decode(),
        })
    template = (HERE / "viewer_template.html").read_text(encoding="utf-8")
    (OUT / "viewer.html").write_text(template.replace("/*DATA*/null", json.dumps(data)), encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
