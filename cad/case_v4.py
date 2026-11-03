"""CO2 monitor case v4 (compact, no breadboard): base and lid, built with build123d.

Run from the project folder:
    cad\\.venv\\Scripts\\python.exe cad\\case_v4.py

Writes cad/out/v4/:
    base.step, lid.step   the files for the school's printer (one STEP per part, mm)
    *.stl                 meshes of every part, for Cura and the viewer
    viewer.html           open it in a browser to turn the model around

What changed from v3 (cad/case.py):
- No breadboard. The Waveshare ESP32-C6-DEV-KIT-N16 lies upside down on 4 posts at the
  back (3 screws, the 4th corner only rests: the module is too close to that hole). Its
  pins point up and the jumper-wire plugs sit on them; the USB-C port is low in the back wall.
- The sensor chamber holds only the SCD-41 and SHT45.
- A second, smaller compartment along the column-j wall, behind its own side slots and
  separated from the ESP32 by a wall: the BMP580 and, later, an SGP40. Both are screwed to
  posts from outside, through the side slots, like the chamber sensors.
- The ESP32 side breathes through floor vents under the board and slots high in the back wall.
- The VEML7700 (5378) lies on 4 low posts under the display, held by 2 screws from above.
- An 8 mm addressable LED (PL9823-F8) sits in a sleeve below the button.
- The panel button (Berrybase 10130) is screwed into the front wall with its own nut.
- The speaker hangs under the lid, over the ESP32.
- One I2C chain: ESP32 -> SGP40 -> BMP580 -> VEML7700 -> (hole in the double wall) -> SHT45
  -> SCD-41. Until the SGP40 is fitted, the first cable goes straight to the BMP580.
  The OLED is wired to the VEML7700's header pads.

Coordinates: x along the case (0 = inside of the back wall, at the USB port), y across
(0 = bay face of the double wall; + = electronics bay, towards the column-j wall;
- = sensor chamber), z up (0 = underside of the floor). All sizes in millimetres.
Values marked ASSUMED are not measured yet.
"""

import base64
import itertools
import json
import math
from datetime import datetime
from pathlib import Path

import build123d
from build123d import (Align, Axis, Box, Circle, Compound, Cylinder, Edge, Keep, Plane, Pos, Rot, Sphere,
                       export_step, export_stl, fillet, import_step, sweep)

HERE = Path(__file__).parent
OUT = HERE / "out" / "v4"
ADAFRUIT = HERE / "adafruit"

# ---- printing (docs "Printing at school")
LAYER = 0.254              # Stratasys F370, standard layer
FIT = 0.3                  # gap around parts that drop in

# ---- shell. Same look as v3: rounded box, lid with 4 screws, slots in the side walls.
WALL = 2.0
FLOOR = 8 * LAYER          # 2.03
LID_T = 8 * LAYER
CORNER_R = 2.0
LID_Z = 38.0               # top of the base = underside of the lid (as v3: display + light sensor)
L = 82.3                   # inside length: SGP40 + BMP580 along the column-j wall, then the button
BAY_W = 46.0               # electronics bay: ESP32, the wall, BMP580/SGP40; display + button column at the front
CHAMBER_D = 18.0           # sensor chamber, as v3
DOUBLE_WALL = ((-4.0, -2.8), (-1.2, 0.0))   # 2 x 1.2 with a 1.6 air gap
DW_OUT = DOUBLE_WALL[0][0]                  # its chamber face
X0, X1 = -WALL, L + WALL
Y0, Y1 = DW_OUT - CHAMBER_D - WALL, BAY_W + WALL
Y_OUTER_IN = Y0 + WALL                      # inside of the chamber's outer wall

# ---- lid screws: 4 x M3 x 10 into corner bosses. Both on the column-j side hang from the
# walls (45 degree underside): the front one above the button, the back one above the SGP40's plug.
CORNERS = [(0.0, Y_OUTER_IN, 1, 1), (L, Y_OUTER_IN, -1, 1), (0.0, BAY_W, 1, -1), (L, BAY_W, -1, -1)]
HANGING_BOSS_Z = {2: 22.0, 3: 26.0}         # corner index: lowest point, at the walls
BOSS_INSET, BOSS_R = 2.0, 3.0
BOSSES = [(x + sx * BOSS_INSET, y + sy * BOSS_INSET) for x, y, sx, sy in CORNERS]
BOSS_PILOT, BOSS_HOLE_DEPTH = 1.8, 12.0     # drill to 2.5
LID_PILOT = 2.5                             # drill to 3.2

FOOT_D, FOOT_H = 12.7, 3.5
FEET = [(x, y) for x in (X0 + 7.5, X1 - 7.5) for y in (Y0 + 7.5, Y1 - 7.5)]

# ---- vents: 4 mm slots in two rows, columns a third of 20.32 apart.
SLOT_W, SLOT_PITCH = 4.0, 20.32 / 3
SLOT_ROWS = ((5.5, 17.5), (20.5, 32.5))
# chamber side: 9 columns centred on the case
SLOT_X = [(X0 + X1) / 2 + (k - 4) * SLOT_PITCH for k in range(9)]
END_SLOT_MID = (Y_OUTER_IN + BOSS_INSET + BOSS_R + DW_OUT) / 2
END_SLOT_Y = (END_SLOT_MID - SLOT_PITCH / 2, END_SLOT_MID + SLOT_PITCH / 2)
FLOOR_VENT_X = SLOT_X[1:-1]
FLOOR_VENT_W, FLOOR_VENT_Y = 2.0, (Y_OUTER_IN + 2.0, Y_OUTER_IN + 10.0)

# ---- chamber sensors (Adafruit, upright, header edge down, parts towards the outer wall)
BOARD_T = 1.57
BOARD_BACK_Y = DW_OUT - 6.0            # back face of the boards; standoffs between it and the double wall
STANDOFF_W = 5.0
STANDOFF_PILOT = 1.6                   # teardrop, drilled to 2.1 through the slot; M2.5 x 6
STANDOFF_PILOT_DEPTH = 5.0
HOLE_IN = 2.54
BOARD_Z = 7.0
BOARD_GAP = 8.47                       # between two boards: room for two STEMMA QT plugs
SCD_X = SLOT_X[0] - HOLE_IN            # screws behind slot columns 1 and 4 (of 9)
SHT_X = SLOT_X[5] - HOLE_IN            # columns 6 and 9: the front, away from the ESP32

# ---- cable hole at the bottom of the double wall, at its front end: the chain goes from the
# light sensor into the SHT45's front socket. Pointed top: prints without support. Seal after wiring.
CABLE_HOLE_X = (SLOT_X[8] + STANDOFF_W / 2 + 0.05, L)
CABLE_HOLE_H = 4.0

# ---- column-j compartment: SGP40 (back, fitted later) and BMP580 (front), upright, header edge
# down, parts towards the wall. Each hangs on its 2 top holes, behind the column-j slots, so a
# 2 mm hex screwdriver reaches them from outside. The BMP390 model (4816) stands in for the
# BMP580 (6411): same outline and holes.
J_WALL_GAP = 4.0                       # parts side to the wall: sockets 2.96, screw heads 2.5
J_FRONT_Y = BAY_W - J_WALL_GAP         # parts side of the boards
J_BACK_Y = J_FRONT_Y - BOARD_T
J_Z = 9.0                              # bottom edge; holes at z 24.24 (upper slot row)
J_POST_D = 5.0                         # posts behind the boards
SGP_X0 = 7.0                           # its back plug leaves 4 mm for the cable's bend
BMP_X0 = SGP_X0 + 25.4 + BOARD_GAP
J_BOARDS = {"SGP40": SGP_X0, "BMP580": BMP_X0}
J_SLOT_X = [SGP_X0 + HOLE_IN + k * SLOT_PITCH for k in range(9)]   # all 4 screws on this grid
# The wall between the ESP32 and the compartment, against the posts. Under the speaker it stops
# 0.5 below it; in front of the speaker it reaches the lid.
PART_WALL_T = 1.6
PART_WALL_Y = (J_BACK_Y - J_POST_D - PART_WALL_T, J_BACK_Y - J_POST_D)
PART_WALL_X = (SGP_X0 - 0.5, BMP_X0 + 25.4 - HOLE_IN + STANDOFF_W / 2 + 0.05)

# ---- Waveshare ESP32-C6-DEV-KIT-N16, upside down: parts face the floor, pins point up.
# Outline, holes and pins from Waveshare's drawing; parts underneath read from its photo.
ESP_L, ESP_W, ESP_T = 55.0, 25.36, 1.6
ESP_X0, ESP_Y0 = 0.5, 3.5              # USB end 0.5 from the back wall; 3.5 from the double wall
ESP_POST_H = 5.6                       # floor to the board: the USB plug's grip clears the floor
ESP_ZB = FLOOR + ESP_POST_H            # board underside (its parts side)
ESP_ZT = ESP_ZB + ESP_T                # pin side
ESP_HOLE_IN, ESP_HOLE_D = 1.77, 2.0
ESP_HOLES = [(ESP_HOLE_IN, ESP_HOLE_IN), (ESP_HOLE_IN, ESP_W - ESP_HOLE_IN),
             (ESP_L - ESP_HOLE_IN, ESP_HOLE_IN), (ESP_L - ESP_HOLE_IN, ESP_W - ESP_HOLE_IN)]
ESP_SCREWED = 3                        # the first 3 holes get M2 x 6; the 4th only rests (module next to it)
ESP_NECK_D, ESP_NECK_H, ESP_POST_D = 3.4, 3.6, 5.0
ESP_PILOT = 1.2                        # drill to 1.6: M2 x 6 cuts its own thread
# The pin row with 3V3 (P1) lies on the chamber side, the one with I2C (P2) on the column-j side.
ESP_ROW_Y = {"P1": ESP_Y0 + 1.25, "P2": ESP_Y0 + ESP_W - 1.25}
HEADER_H, PIN_UP, JUMPER_L = 2.5, 6.0, 14.0
USED_PINS = {"P1": (1, 12, 13, 14, 15),       # 3V3, GPIO2 button, GPIO3 speaker, 5V, GND
             "P2": (1, 5, 6, 10, 12, 15)}     # GND, GPIO23 SDA, GPIO22 SCL, GPIO18 LED, GND, GND
ESP_MODULE = (29.2, 55.0, 3.9, 22.6, 3.2)     # x from the USB end, y from the P1 edge, height (ASSUMED from the photo)
ESP_USB = (-0.8, 6.55, ESP_W / 2, 8.94, 3.26)  # x0, x1, y middle, width, height
ESP_BUTTONS = ((3.9, 9.5, 0.9, 3.9), (3.9, 9.5, 21.46, 24.46))   # RESET, BOOT; 2.5 tall (ASSUMED)


def pin_x(k):
    """x of pin k (1..16): pin 16 (NC) is 12.0 from the USB end, pin 1 (3V3 / GND) at the antenna end."""
    return ESP_X0 + 12.0 + (16 - k) * 2.54


USB_Y = ESP_Y0 + ESP_USB[2]
USB_Z = ESP_ZB - ESP_USB[4] / 2
USB_CUT_W, USB_CUT_H, USB_CUT_R = 12.5, 7.5, 1.0    # room for the plug's grip, as v3
# ESP32 side vents: cool air in through the floor under the board, warm air out high in the back wall
ESP_FLOOR_VENT_X = [ESP_X0 + 12.0 + k * SLOT_PITCH for k in range(6)]
ESP_FLOOR_VENT_Y = (ESP_Y0 + 6.0, ESP_Y0 + 19.4)
BACK_SLOT_Y = [USB_Y + k * SLOT_PITCH for k in (-2, -1, 0, 1, 2)]   # upper row only, over the USB port

# ---- front face (x = L .. X1)
OLED_W, OLED_H = 27.75, 28.0
OLED_Y0, OLED_Z0 = 3.5, 9.0            # board edge on the chamber side, bottom edge (v3: 7.4; the light sensor is higher now)
OLED_GLASS = (26.75, 19.15, 1.8, 4.4)  # width, height, thickness, top edge below the board's top
OLED_RECESS = 1.0
OLED_BOARD_T = 1.2                     # ASSUMED (O3)
OLED_X = X1 - WALL + OLED_RECESS - OLED_GLASS[2]   # front face of the board
OLED_PEGS, OLED_PEG_D = (OLED_Y0 + 3.25, OLED_Y0 + 24.5), 2.4
OLED_HOLE_D, OLED_HOLE_IN = 1.5, (1.9, 2.1)
OLED_NOTCH = (14.5, 2.0)
OLED_PIN_DOWN, OLED_PIN_BACK = 1.5, 6.0
RAIL_T, RAIL_LIP, RAIL_Z = 1.2, 1.0, OLED_Z0 + 2.6
SHELVES = ((OLED_Y0 + 2.25, OLED_Y0 + 5.75), (OLED_Y0 + 21.75, OLED_Y0 + 25.25))
OLED_WIN = (OLED_Y0 + 1.25, OLED_Z0 + 9.1, OLED_Y0 + 26.25, OLED_Z0 + 23.1)   # y0, z0, y1, z1

# VEML7700 right angle (5378): lies parts up under the display, its sensor at the front wall,
# on 4 posts; the 2 at the back take M2.5 x 5 screws from above (floor + post = 3.6 mm).
VEML_X = L - 0.1 - 17.78               # back edge; the front edge is 0.1 from the wall
VEML_Y = OLED_Y0 + 2.45                # edge on the chamber side
VEML_Z = 3.6                           # board underside
VEML_SENSOR = (9.26, 16.16, 2.5)       # sensor: from and to (from the board's chamber-side edge), height
VEML_WINDOW_GAP = 0.3
VEML_HOLES = [(VEML_X + hx, VEML_Y + hy) for hx in (HOLE_IN, 17.78 - HOLE_IN) for hy in (HOLE_IN, 25.4 - HOLE_IN)]
VEML_SCREWED = [h for h in VEML_HOLES if h[0] < VEML_X + 5]
VEML_PILOT = 1.6                       # drill to 2.1, through the floor

# Panel button, Berrybase 10130 (drawing in its product sheet): collar inside the wall, nut outside.
BUTTON_Y, BUTTON_Z = 39.6, (OLED_WIN[1] + OLED_WIN[3]) / 2    # level with the middle of the display
BUTTON_PILOT = 6.0                     # drill to 7.0 (M7 x 0.75)
BUTTON = {"cap": (7.4, 1.7), "stem": (3.4, 3.8), "thread": (6.7, 6.3), "collar": (9.7, 2.4),
          "body": (7.9, 7.0), "lugs": 6.3, "washer": (9.3, 1.1), "nut": (10.1, 1.7)}   # (diameter, length)

# 8 mm addressable LED (Berrybase PL9823-F8): pushed into a sleeve from behind until its rim
# stops; the dome stands LED_PROUD out of the front. Sizes ASSUMED (Adafruit's 8 mm NeoPixel: 11 tall).
LED_Y, LED_Z = BUTTON_Y, 14.0
LED_D, LED_RIM_D, LED_RIM_T, LED_H = 8.0, 8.9, 1.0, 11.0
LED_PROUD = 1.5
LED_RIM_X = X1 + LED_PROUD - LED_H     # back face of the rim
LED_PILOT = 7.5                        # drill to 8.0 through the wall and the sleeve

# ---- speaker (3885) under the lid, over the ESP32; its JST PH socket faces the double wall,
# right above the ESP32's 5V/GPIO3/GND pins.
SPK_X, SPK_Y = 6.0, 6.5                # board corner (socket edge at SPK_Y)
SPK_Z = LID_Z - 0.5 - 6.57             # board bottom: the speaker's top 0.5 under the lid
SPK_HOLES = [(2.54, 2.54), (27.94, 2.54), (2.54, 34.29), (27.94, 34.29)]
SPK_STANDOFF_D, SPK_PILOT = 4.5, 1.2   # drill to 1.6 for M2 x 6
GRILLE_D = 1.8
GRILLE = [(SPK_X + 15.39 + 4 * i, SPK_Y + 21.54 + 4 * j) for i in range(-3, 4) for j in range(-2, 3)]

# ---- STEMMA QT cables: 50 mm plug to plug = about 41 mm of wire between the plugs
CABLE_WIRE, CABLE_D = 41.0, 1.6
PLUG_OUT = 3.0


def block(x0, y0, z0, x1, y1, z1):
    """Box between two opposite corners."""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def rod(x, y, z0, z1, d):
    """Vertical cylinder of diameter d from z0 to z1."""
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def rod_x(x0, x1, y, z, d):
    """Cylinder along x."""
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def rounded_z(shape, r):
    return fillet(shape.edges().filter_by(Axis.Z), r)


def teardrop_y(x, z, y0, y1, d, up=1):
    """Horizontal hole along y with a 45 degree point, so it prints without support."""
    r = d / 2
    hole = Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)
    return hole + Pos(x, (y0 + y1) / 2, z + up * r / math.sqrt(2)) * Rot(0, 45, 0) * Box(r, y1 - y0, r)


def teardrop_x(y, z, x0, x1, d):
    """Horizontal hole along x with a 45 degree point upwards."""
    r = d / 2
    hole = rod_x(x0, x1, y, z, d)
    return hole + Pos((x0 + x1) / 2, y, z + r / math.sqrt(2)) * Rot(45, 0, 0) * Box(x1 - x0, r, r)


def hanging(shape, x_wall, z_low, into):
    """Cut the underside of a part on a wall at 45 degrees, rising away from the wall."""
    return shape.split(Plane(origin=(x_wall, 0, z_low), z_dir=(-into, 0, 1)), keep=Keep.TOP)


def corner_boss(x, y, sx, sy, z0, z1):
    """Lid-screw boss filling an inside corner, rounded towards the inside."""
    size = BOSS_INSET + BOSS_R
    boss = block(min(x, x + sx * size), min(y, y + sy * size), z0, max(x, x + sx * size), max(y, y + sy * size), z1)
    far = (x + sx * size, y + sy * size)
    edge = min(boss.edges().filter_by(Axis.Z), key=lambda e: (e.center().X - far[0]) ** 2 + (e.center().Y - far[1]) ** 2)
    return fillet([edge], BOSS_R)


def upright(shape, x, z, y=BOARD_BACK_Y):
    """Stand a board model up: header edge down at z, parts towards -y (the chamber's outer wall)."""
    return Pos(x, y, z) * Rot(90, 0, 0) * shape


def upright_j(shape, x0, z, y=J_BACK_Y):
    """Stand a board model up facing +y (the column-j wall): header edge down at z, back face at y,
    the board reaching from x0 to x0 + 25.4 (the model's x runs backwards)."""
    return Pos(x0 + 25.4, y, z) * Rot(0, 0, 180) * Rot(90, 0, 0) * shape


def board_screws(x, z, height, holes=(HOLE_IN, 25.4 - HOLE_IN)):
    """Screw positions (x, z) of an upright board using the 2 holes away from its header."""
    return [(x + hx, z + height - HOLE_IN) for hx in holes]


def sensor_screws():
    """(board, wall, x, z) of every screw turned from outside through a side slot."""
    screws = [("SCD-41", "a", x, z) for x, z in board_screws(SCD_X, BOARD_Z, 22.86)]
    screws += [("SHT45", "a", x, z) for x, z in board_screws(SHT_X, BOARD_Z, 17.78)]
    for name, x0 in J_BOARDS.items():
        screws += [(name, "j", x, z) for x, z in board_screws(x0, J_Z, 17.78)]
    return screws


def speaker_holes():
    return [(SPK_X + hx, SPK_Y + hy) for hx, hy in SPK_HOLES]


def esp_hole(i):
    hx, hy = ESP_HOLES[i]
    return ESP_X0 + hx, ESP_Y0 + hy


def make_base():
    base = rounded_z(block(X0, Y0, 0, X1, Y1, LID_Z), CORNER_R)
    base -= block(X0 + WALL, Y0 + WALL, FLOOR, X1 - WALL, Y1 - WALL, LID_Z + 1)

    for y0, y1 in DOUBLE_WALL:
        base += block(0, y0, FLOOR, L, y1, LID_Z)
    hx0, hx1 = CABLE_HOLE_X
    w = hx1 - hx0
    base -= block(hx0, DOUBLE_WALL[0][0] - 1, FLOOR, hx1, DOUBLE_WALL[1][1] + 1, FLOOR + CABLE_HOLE_H)
    base -= Pos((hx0 + hx1) / 2, sum(DOUBLE_WALL[0] + DOUBLE_WALL[1]) / 4, FLOOR + CABLE_HOLE_H) * Rot(0, 45, 0) * Box(
        w / math.sqrt(2), 6, w / math.sqrt(2))

    for i, ((x, y, sx, sy), (bx, by)) in enumerate(zip(CORNERS, BOSSES)):
        if i in HANGING_BOSS_Z:
            z_low = HANGING_BOSS_Z[i]
            boss = corner_boss(x, y, sx, sy, z_low, LID_Z)
            for normal in ((-sx, 0, 1), (0, -sy, 1)):
                boss = boss.split(Plane(origin=(x, y, z_low), z_dir=normal), keep=Keep.TOP)
        else:
            boss = corner_boss(x, y, sx, sy, FLOOR, LID_Z)
        base += boss
        base -= rod(bx, by, LID_Z - BOSS_HOLE_DEPTH, LID_Z + 1, BOSS_PILOT)

    # chamber sensors: standoffs against the double wall
    for _, wall, sx, sz in sensor_screws():
        if wall == "a":
            base += block(sx - STANDOFF_W / 2, BOARD_BACK_Y, FLOOR, sx + STANDOFF_W / 2, DW_OUT, sz + 3)
            base -= teardrop_y(sx, sz, BOARD_BACK_Y - 0.5, BOARD_BACK_Y + STANDOFF_PILOT_DEPTH, STANDOFF_PILOT)

    # column-j compartment: 2 posts per board, a low web between them, and the wall in front of
    # the posts (against them, so it is stiff)
    post_y0 = J_BACK_Y - J_POST_D
    for name in J_BOARDS:
        posts = [(sx, sz) for n, wall, sx, sz in sensor_screws() if n == name]
        for sx, sz in posts:
            base += block(sx - STANDOFF_W / 2, post_y0, FLOOR, sx + STANDOFF_W / 2, J_BACK_Y, sz + 3)
        base += block(posts[0][0], post_y0, FLOOR, posts[1][0], post_y0 + 2.0, J_Z - 0.5)
        for sx, sz in posts:
            base -= teardrop_y(sx, sz, J_BACK_Y - STANDOFF_PILOT_DEPTH, J_BACK_Y + 0.5, STANDOFF_PILOT)
    wx0, wx1 = PART_WALL_X
    wy0, wy1 = PART_WALL_Y
    x_speaker = SPK_X + 30.48 + 0.5
    base += block(wx0, wy0, FLOOR, x_speaker, wy1, SPK_Z - 0.5)
    base += block(x_speaker, wy0, FLOOR, wx1, wy1, LID_Z)

    # ESP32 posts: a wide foot and a narrow neck that fits between the parts under the board;
    # the 4th (next to the module) is cut flat on the module's side and has no screw
    neck_z = ESP_ZB - ESP_NECK_H
    for i in range(4):
        x, y = esp_hole(i)
        post = rod(x, y, FLOOR, neck_z, ESP_POST_D) + rod(x, y, neck_z, ESP_ZB, ESP_NECK_D)
        if i >= ESP_SCREWED:
            module_edge = ESP_Y0 + ESP_MODULE[3] + 0.3
            post -= block(x - 3, y - 3, neck_z, x + 3, module_edge, ESP_ZB + 1)
        base += post
    for i in range(ESP_SCREWED):
        x, y = esp_hole(i)
        base -= rod(x, y, ESP_ZB - 5.0, ESP_ZB + 1, ESP_PILOT)

    # light sensor: 4 posts, 2 with screw holes through the floor; window in the front wall
    for x, y in VEML_HOLES:
        base += rod(x, y, FLOOR - 0.01, VEML_Z, 5.0)
    for x, y in VEML_SCREWED:
        base -= rod(x, y, -1, VEML_Z + 1, VEML_PILOT)
    s0, s1, sh = VEML_SENSOR
    g = VEML_WINDOW_GAP
    zt = VEML_Z + BOARD_T
    base -= block(X1 - WALL - 0.5, VEML_Y + s0 - g, zt - g, X1 + 1, VEML_Y + s1 + g, zt + sh + g)

    # OLED holder (as v3): recess for its glass, rails behind its side edges, shelves under it
    x_wall = X1 - WALL
    gw, gh, _, gdown = OLED_GLASS
    ym = OLED_Y0 + OLED_W / 2
    base -= block(x_wall - 1, ym - gw / 2 - 0.2, OLED_Z0 + OLED_H - gdown - gh - 0.3, x_wall + OLED_RECESS,
                  ym + gw / 2 + 0.2, LID_Z + 1)
    x_lip = OLED_X - OLED_BOARD_T - FIT
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

    # LED sleeve: stands on the floor behind the front wall
    base += block(LED_RIM_X + LED_RIM_T, LED_Y - 5.0, FLOOR, x_wall, LED_Y + 5.0, LED_Z + LED_D / 2 + 1.5)
    base -= teardrop_x(LED_Y, LED_Z, LED_RIM_X, X1 + 1, LED_PILOT)

    # vents
    for z0, z1 in SLOT_ROWS:
        for x in SLOT_X:
            base -= block(x - SLOT_W / 2, Y0 - 1, z0, x + SLOT_W / 2, Y0 + WALL + 0.5, z1)
        for x in J_SLOT_X:
            base -= block(x - SLOT_W / 2, Y1 - WALL - 0.5, z0, x + SLOT_W / 2, Y1 + 1, z1)
        for y in END_SLOT_Y:
            base -= block(X0 - 1, y - SLOT_W / 2, z0, X0 + WALL + 0.5, y + SLOT_W / 2, z1)
    z0, z1 = SLOT_ROWS[1]
    for y in BACK_SLOT_Y:
        base -= block(X0 - 1, y - SLOT_W / 2, z0, X0 + WALL + 0.5, y + SLOT_W / 2, z1)
    for x in FLOOR_VENT_X:
        base -= block(x - FLOOR_VENT_W / 2, FLOOR_VENT_Y[0], -1, x + FLOOR_VENT_W / 2, FLOOR_VENT_Y[1], FLOOR + 0.5)
    for x in ESP_FLOOR_VENT_X:
        base -= block(x - FLOOR_VENT_W / 2, ESP_FLOOR_VENT_Y[0], -1, x + FLOOR_VENT_W / 2, ESP_FLOOR_VENT_Y[1], FLOOR + 0.5)

    # USB-C port, display window, button hole
    cut = block(X0 - 1, USB_Y - USB_CUT_W / 2, USB_Z - USB_CUT_H / 2, 1, USB_Y + USB_CUT_W / 2, USB_Z + USB_CUT_H / 2)
    base -= fillet(cut.edges().filter_by(Axis.X), USB_CUT_R)
    y0, z0, y1, z1 = OLED_WIN
    base -= block(X1 - WALL - 1, y0, z0, X1 + 1, y1, z1)
    base -= rod_x(X1 - WALL - 1, X1 + 1, BUTTON_Y, BUTTON_Z, BUTTON_PILOT)

    base.label = "base"
    return base


def make_lid():
    lid = rounded_z(block(X0, Y0, LID_Z, X1, Y1, LID_Z + LID_T), CORNER_R)
    for x, y in GRILLE:
        lid -= rod(x, y, LID_Z - 1, LID_Z + LID_T + 1, GRILLE_D)
    for x, y in BOSSES:
        lid -= rod(x, y, LID_Z - 1, LID_Z + LID_T + 1, LID_PILOT)
    board_top = SPK_Z + BOARD_T
    for x, y in speaker_holes():
        lid += rod(x, y, board_top, LID_Z, SPK_STANDOFF_D)
        lid -= rod(x, y, board_top - 0.5, board_top + 5.0, SPK_PILOT)
    for y in OLED_PEGS:
        lid += rod(OLED_X - OLED_BOARD_T / 2, y, OLED_Z0 + OLED_H + 0.2, LID_Z, OLED_PEG_D)
    lid.label = "lid"
    return lid


# ---------------------------------------------------------------- cables

def grow(path, wire, d, start, start_dir, h_max=30.0):
    """Find the loop size h (0..h_max) that makes path(h) `wire` long; returns the tube and its path."""
    lo, hi = 0.0, h_max
    for _ in range(40):
        h = (lo + hi) / 2
        lo, hi = (h, hi) if path(h).length < wire else (lo, h)
    edge = path((lo + hi) / 2)
    return sweep(Plane(origin=start, z_dir=start_dir) * Circle(d / 2), path=edge), edge


def cable(a, b, wire=CABLE_WIRE, bulge_y=Y_OUTER_IN + 4.0, up=True, lean=0.0, d=CABLE_D):
    """Cable between the plugs of two neighbouring upright boards facing each other: from plug
    end a (cable leaves towards -x) to plug end b (leaves towards +x). The spare wire loops up
    (or down) and out to y = bulge_y, leaning `lean` x h along x (under a board); the loop grows
    until the cable is `wire` long."""
    V = build123d.Vector
    a, b = V(*a), V(*b)
    s = 1 if up else -1
    far = max(a.Z, b.Z) if up else min(a.Z, b.Z)

    def path(h):
        pts = [a, V(a.X - 1.5, (a.Y + bulge_y) / 2, a.Z + s * h / 3), V((a.X + b.X) / 2 + lean * h, bulge_y, far + s * h),
               V(b.X + 1.5, (b.Y + bulge_y) / 2, b.Z + s * h / 3), b]
        return Edge.make_spline(pts, tangents=[(-1, 0, 0), (-1, 0, 0)])

    return grow(path, wire, d, a, (-1, 0, 0))


def routed(points, start_dir, end_dir, d=CABLE_D):
    """Cable through fixed points. Parameters by distance and unscaled end directions: the
    curve stays close to the points, without loops."""
    pts = [build123d.Vector(*p) for p in points]
    par = [0.0] + list(itertools.accumulate((b - a).length for a, b in zip(pts, pts[1:])))
    edge = Edge.make_spline(pts, tangents=[start_dir, end_dir], parameters=par, scale=False)
    return sweep(Plane(origin=points[0], z_dir=start_dir) * Circle(d / 2), path=edge), edge


def stub(a, b, d=CABLE_D):
    """Straight bit of cable from a to b (into a plug)."""
    direction = tuple(q - p for p, q in zip(a, b))
    return sweep(Plane(origin=a, z_dir=direction) * Circle(d / 2), path=Edge.make_line(a, b))


def plug_x(x_open, out, y_mid, z_mid):
    """Plug in a socket of an upright board (socket 6.0 along z, 2.96 along y), sticking out along x."""
    x_end = x_open + out * PLUG_OUT
    return block(min(x_open, x_end), y_mid - 1.4, z_mid - 2.8, max(x_open, x_end), y_mid + 1.4, z_mid + 2.8)


def plug_y(x_mid, y_open, out, z_mid):
    """Plug in a socket of a flat board (6.0 along x, 2.96 along z), sticking out along y."""
    y_end = y_open + out * PLUG_OUT
    return block(x_mid - 2.8, min(y_open, y_end), z_mid - 1.4, x_mid + 2.8, max(y_open, y_end), z_mid + 1.4)


def make_sensors():
    """Sensor boards, plugs and cables. Returns parts and the cable lengths."""
    scd = upright(import_step(ADAFRUIT / "5187 SCD-40 C02 Sensor.step"), SCD_X, BOARD_Z)
    sht = upright(import_step(ADAFRUIT / "5665 SHT45 Sensor.step"), SHT_X, BOARD_Z)
    bmp = upright_j(import_step(ADAFRUIT / "4816 BMP390 QT.step"), BMP_X0, J_Z)
    sgp = upright_j(import_step(ADAFRUIT / "4829 SGP40 Sensor.step"), SGP_X0, J_Z)
    veml = Pos(VEML_X, VEML_Y + 25.4, VEML_Z) * Rot(0, 0, -90) * import_step(ADAFRUIT / "5378 VEML7700 Lux Sensor.step")

    y_sock = BOARD_BACK_Y - BOARD_T - 1.48          # middle of a socket of a chamber board
    sht_back = (SHT_X + 0.70, BOARD_Z + 8.89)
    sht_front = (SHT_X + 24.70, BOARD_Z + 8.89)
    scd_front = (SCD_X + 25.34, BOARD_Z + 11.43)
    y_j = J_FRONT_Y + 1.48                          # middle of a socket of a column-j board
    zj = J_Z + 8.89
    sgp_back, sgp_front = SGP_X0 + 0.06, SGP_X0 + 25.34       # socket openings (from the models)
    bmp_back, bmp_front = BMP_X0 + 0.19, BMP_X0 + 25.21
    vx = VEML_X + 8.89
    vz = VEML_Z + BOARD_T + 1.48
    veml_a, veml_j = VEML_Y + 0.19, VEML_Y + 25.21  # socket openings, chamber side and column-j side

    plugs = [plug_x(sht_front[0], 1, y_sock, sht_front[1]), plug_x(sht_back[0], -1, y_sock, sht_back[1]),
             plug_x(scd_front[0], 1, y_sock, scd_front[1]),
             plug_x(sgp_back, -1, y_j, zj), plug_x(sgp_front, 1, y_j, zj),
             plug_x(bmp_back, -1, y_j, zj), plug_x(bmp_front, 1, y_j, zj),
             plug_y(vx, veml_a, -1, vz), plug_y(vx, veml_j, 1, vz)]

    lengths, tubes = {}, []

    def note(name, edge, wire):
        zs = [edge.position_at(i / 200).Z for i in range(201)]
        lengths[name] = (edge.length, wire, min(zs) - CABLE_D / 2, max(zs) + CABLE_D / 2)

    # chamber: SHT45 back socket -> SCD-41 front socket (50 mm), loop up towards the outer wall
    tube, edge = cable((sht_back[0] - PLUG_OUT, y_sock, sht_back[1]), (scd_front[0] + PLUG_OUT, y_sock, scd_front[1]))
    tubes.append(tube)
    note("SHT45 - SCD-41 (50 mm)", edge, CABLE_WIRE)

    # column-j compartment: BMP580 back socket -> SGP40 front socket (50 mm), loop down in the gap
    tube, edge = cable((bmp_back - PLUG_OUT, y_j, zj), (sgp_front + PLUG_OUT, y_j, zj), bulge_y=41.5, up=False, lean=0.9)
    tubes.append(tube)
    note("BMP580 - SGP40 (50 mm, later)", edge, CABLE_WIRE)

    # light sensor -> through the hole in the double wall -> SHT45 front socket (50 mm)
    hx = sum(CABLE_HOLE_X) / 2
    end = (sht_front[0] + PLUG_OUT, y_sock, sht_front[1])
    tube, edge = routed([(vx, veml_a - PLUG_OUT, vz), (vx + 1.5, 0.8, FLOOR + 2.6), (hx - 1.0, -2.0, FLOOR + 2.0),
                         (hx, -6.5, FLOOR + 4.5), (hx - 0.5, -11.0, 11.0), (end[0] + 2.0, y_sock, end[2])],
                        (0, -1, 0), (-1, 0, 0))
    tubes += [tube, stub((end[0] + 2.0, y_sock, end[2]), end)]
    note("light sensor - SHT45 (50 mm)", edge, CABLE_WIRE)

    # BMP580 front socket -> light sensor's column-j socket (50 mm): down first, under the LED's legs
    a = (bmp_front + PLUG_OUT, y_j, zj)
    end = (vx, veml_j + PLUG_OUT, vz)
    tube, edge = routed([a, (a[0] + 2.0, y_j + 0.2, zj - 4.0), (vx - 0.8, y_j - 1.5, vz + 2.0), (vx, 38.0, vz + 0.2),
                         (vx, end[1] + 1.5, vz)], (1, 0, 0), (0, -1, 0))
    tubes += [tube, stub((vx, end[1] + 1.5, vz), end)]
    note("BMP580 - light sensor (50 mm)", edge, CABLE_WIRE)

    # ESP32 pins -> SGP40 back socket: STEMMA QT cable with female sockets, 150 mm. Drawn from above
    # the SDA/SCL plugs, along the ESP32 side of the wall and round its back end; the 3V3 and GND
    # wires branch off to pin 1 of both rows. Without the SGP40 it goes on to the BMP580's back socket.
    top = ESP_ZT + HEADER_H + JUMPER_L
    sx = (pin_x(5) + pin_x(6)) / 2
    end = (sgp_back - PLUG_OUT, y_j, zj)
    yr = PART_WALL_Y[0] - 3.0                      # along the wall, clear of it and of the plugs' wires
    tube, edge = routed([(sx, ESP_ROW_Y["P2"], top + 1.0), (sx - 3.0, yr, 28.5), (28.0, yr, 28.3),
                         (15.0, yr, 28.0), (3.5, yr + 1.0, 25.0),
                         (2.0, PART_WALL_Y[1] + 3.0, 21.5), (1.6, y_j - 0.5, zj + 0.6), (end[0] - 1.2, y_j, zj)],
                        (0, 0, 1), (1, 0, 0))
    tubes += [tube, stub((end[0] - 1.2, y_j, zj), end)]
    note("ESP32 - SGP40 (150 mm)", edge, 150.0 - 9.0)

    # OLED: 4 wires from the light sensor's header pads to female plugs on the OLED's pins
    x_back = OLED_X - OLED_BOARD_T
    zp = OLED_Z0 + OLED_H - OLED_PIN_DOWN
    ym = OLED_Y0 + OLED_W / 2
    x_end = x_back - HEADER_H - JUMPER_L
    tube, edge = routed([(x_end, ym, zp), (x_end - 3.0, ym, zp - 3.0), (x_end - 3.5, ym + 1.0, 20.0),
                         (VEML_X - 0.5, VEML_Y + 12.7, 9.0), (VEML_X + 1.3, VEML_Y + 12.7, VEML_Z + BOARD_T + 0.6)],
                        (-1, 0, 0), (0, 0, -1), d=2.6)
    tubes.append(tube)
    note("OLED wires (4 wires, about 100 mm)", edge, 90.0)

    sensors = {"scd41": scd, "sht45": sht, "bmp580": bmp, "sgp40": sgp, "veml7700": veml,
               "plugs": Compound(plugs), "cables": Compound(tubes)}
    return sensors, lengths


# ---------------------------------------------------------------- reference parts (not printed)

def make_esp():
    """Waveshare ESP32-C6-DEV-KIT-N16, upside down, with the header strips and the jumper plugs."""
    x0, y0 = ESP_X0, ESP_Y0
    pcb = block(x0, y0, ESP_ZB, x0 + ESP_L, y0 + ESP_W, ESP_ZT)
    for hx, hy in ESP_HOLES:
        pcb -= rod(x0 + hx, y0 + hy, ESP_ZB - 1, ESP_ZT + 1, ESP_HOLE_D)
    mx0, mx1, my0, my1, mh = ESP_MODULE
    ux0, ux1, uy, uw, uh = ESP_USB
    under = [block(x0 + mx0, y0 + my0, ESP_ZB - mh, x0 + mx1, y0 + my1, ESP_ZB),
             block(x0 + ux0, y0 + uy - uw / 2, ESP_ZB - uh, x0 + ux1, y0 + uy + uw / 2, ESP_ZB)]
    under += [block(x0 + bx0, y0 + by0, ESP_ZB - 2.5, x0 + bx1, y0 + by1, ESP_ZB) for bx0, bx1, by0, by1 in ESP_BUTTONS]
    strips, tails, plugs = [], [], []
    for row, y in ESP_ROW_Y.items():
        strips.append(block(pin_x(16) - 1.27, y - 1.27, ESP_ZT, pin_x(1) + 1.27, y + 1.27, ESP_ZT + HEADER_H))
        strips.append(block(pin_x(16) - 0.32, y - 0.32, ESP_ZT + HEADER_H, pin_x(1) + 0.32, y + 0.32, ESP_ZT + HEADER_H + PIN_UP))
        tails.append(block(pin_x(16) - 0.32, y - 0.32, ESP_ZB - 2.5, pin_x(1) + 0.32, y + 0.32, ESP_ZB))
        for k in USED_PINS[row]:
            x = pin_x(k)
            z = ESP_ZT + HEADER_H
            plugs.append(block(x - 1.27, y - 1.27, z, x + 1.27, y + 1.27, z + JUMPER_L))
            plugs.append(rod(x, y, z + JUMPER_L, z + JUMPER_L + 4.0, 1.4))      # wire leaving the plug, before its bend
    return Compound([pcb] + under + strips + tails), Compound(plugs)


def make_oled():
    """Joy-it SBC-OLED01 from the measurements (O1-O7), with the 4 jumper-wire plugs on its pins."""
    x_glass = OLED_X
    x_back = x_glass - OLED_BOARD_T
    y0, y1, z0, z1 = OLED_Y0, OLED_Y0 + OLED_W, OLED_Z0, OLED_Z0 + OLED_H
    ym = (y0 + y1) / 2
    board = block(x_back, y0, z0, x_glass, y1, z1)
    for hy in (y0 + OLED_HOLE_IN[0], y1 - OLED_HOLE_IN[0]):
        for hz in (z0 + OLED_HOLE_IN[1], z1 - OLED_HOLE_IN[1]):
            board -= rod_x(x_back - 1, x_glass + 1, hy, hz, OLED_HOLE_D)
    nw, nd = OLED_NOTCH
    board -= block(x_back - 1, ym - nw / 2, z0 - 1, x_glass + 1, ym + nw / 2, z0 + nd)
    gw, gh, gt, gdown = OLED_GLASS
    glass = block(x_glass, ym - gw / 2, z1 - gdown - gh, x_glass + gt, ym + gw / 2, z1 - gdown)
    zp = z1 - OLED_PIN_DOWN
    x_plastic = x_back - HEADER_H
    header = block(x_plastic, ym - 5.08, zp - 1.25, x_back, ym + 5.08, zp + 1.25)
    pins = [block(x_plastic - OLED_PIN_BACK, ym + k * 2.54 - 0.32, zp - 0.32, x_glass + gt, ym + k * 2.54 + 0.32, zp + 0.32)
            for k in (-1.5, -0.5, 0.5, 1.5)]
    jumpers = block(x_plastic - JUMPER_L, ym - 5.08, zp - 1.27, x_plastic, ym + 5.08, zp + 1.27)
    return Compound([board, glass, header, jumpers] + pins)


def make_button():
    """Berrybase 10130 mini push button: collar against the inside of the front wall."""
    x = L
    parts = []
    d, t = BUTTON["collar"]
    parts.append(rod_x(x - t, x, BUTTON_Y, BUTTON_Z, d))
    x_body = x - t
    d, t = BUTTON["body"]
    parts.append(rod_x(x_body - t, x_body, BUTTON_Y, BUTTON_Z, d))
    x_lug = x_body - t
    for dy in (-1.6, 1.6):
        parts.append(block(x_lug - BUTTON["lugs"], BUTTON_Y + dy - 0.2, BUTTON_Z - 1.25, x_lug, BUTTON_Y + dy + 0.2, BUTTON_Z + 1.25))
    d, t = BUTTON["thread"]
    parts.append(rod_x(x, x + t, BUTTON_Y, BUTTON_Z, d))
    x_top = x + t
    d, t = BUTTON["stem"]
    parts.append(rod_x(x_top, x_top + t, BUTTON_Y, BUTTON_Z, d))
    d, t2 = BUTTON["cap"]
    parts.append(rod_x(x_top + t, x_top + t + t2, BUTTON_Y, BUTTON_Z, d))
    dw, tw = BUTTON["washer"]
    washer = rod_x(X1, X1 + tw, BUTTON_Y, BUTTON_Z, dw) - rod_x(X1 - 1, X1 + tw + 1, BUTTON_Y, BUTTON_Z, 6.9)
    dn, tn = BUTTON["nut"]
    nut = Pos(X1 + tw + tn / 2, BUTTON_Y, BUTTON_Z) * Rot(0, 90, 0) * build123d.extrude(
        build123d.RegularPolygon(dn / 2, 6), amount=tn / 2, both=True)
    nut -= rod_x(X1, X1 + tw + tn + 1, BUTTON_Y, BUTTON_Z, 6.7)
    return Compound(parts), Compound([washer, nut])


def make_led():
    """8 mm LED: rim, body, dome, and the legs cut to 8 mm."""
    x = LED_RIM_X
    rim = rod_x(x, x + LED_RIM_T, LED_Y, LED_Z, LED_RIM_D)
    body_end = x + LED_H - LED_D / 2
    body = rod_x(x + LED_RIM_T, body_end, LED_Y, LED_Z, LED_D)
    dome = Pos(body_end, LED_Y, LED_Z) * Sphere(LED_D / 2)
    dome = dome.split(Plane(origin=(body_end, 0, 0), z_dir=(1, 0, 0)), keep=Keep.TOP)
    legs = [block(x - 8.0, LED_Y + dy - 0.25, LED_Z - 0.25, x, LED_Y + dy + 0.25, LED_Z + 0.25)
            for dy in (-1.905, -0.635, 0.635, 1.905)]
    return Compound([rim, body, dome] + legs)


def make_speaker():
    """Adafruit 3885 under the lid, speaker up, with the plug of its JST PH cable."""
    speaker = Pos(SPK_X, SPK_Y, SPK_Z) * import_step(ADAFRUIT / "3885 STEMMA Speaker rev A.step")
    xm = SPK_X + 15.24
    ph = block(xm - 4.0, SPK_Y - 6.0, SPK_Z + BOARD_T + 0.15, xm + 4.0, SPK_Y + 0.27, SPK_Z + BOARD_T + 4.65)
    return Compound(list(speaker.solids()) + [ph])


def make_screws():
    """Screw heads, to check that the screwdriver and the heads have room."""
    heads = []
    for i in range(ESP_SCREWED):                      # M2 x 6 button head (ISO 7380: 3.5 x 1.1)
        x, y = esp_hole(i)
        heads.append(rod(x, y, ESP_ZT, ESP_ZT + 1.1, 3.5))
    for x, y in VEML_SCREWED:                          # M2.5 x 5 socket head (4.5 x 2.5)
        heads.append(rod(x, y, VEML_Z + BOARD_T, VEML_Z + BOARD_T + 2.5, 4.5))
    for _, wall, sx, sz in sensor_screws():           # M2.5 x 6 socket head, on the parts side
        if wall == "a":
            heads.append(Pos(sx, BOARD_BACK_Y - BOARD_T - 1.25, sz) * Rot(90, 0, 0) * Cylinder(2.25, 2.5))
        else:
            heads.append(Pos(sx, J_FRONT_Y + 1.25, sz) * Rot(90, 0, 0) * Cylinder(2.25, 2.5))
    return Compound(heads)


def make_references():
    esp, jumpers = make_esp()
    button, nut = make_button()
    return {
        "esp32": esp,
        "esp32_plugs": jumpers,
        "oled": make_oled(),
        "button": button,
        "button_nut": nut,
        "led": make_led(),
        "speaker": make_speaker(),
        "screws": make_screws(),
        "rubber_feet": Compound([rod(x, y, -FOOT_H, 0, FOOT_D) for x, y in FEET]),
    }


# ---------------------------------------------------------------- checks

def overlap(a, b):
    """Volume of b inside a, solid by solid."""
    try:
        return sum((a & s).volume for s in b.solids())
    except Exception:
        try:
            return sum((s & a).volume for s in b.solids())
        except Exception:
            return float("nan")


def check_slots():
    """Each side-wall screw must sit behind a slot: the 2.3 mm screwdriver blade fits."""
    blade = 2.3 / 2
    for name, wall, sx, sz in sensor_screws():
        cols = SLOT_X if wall == "a" else J_SLOT_X
        col = min(cols, key=lambda c: abs(c - sx))
        ok_x = abs(col - sx) + blade <= SLOT_W / 2
        ok_z = any(z0 + blade <= sz <= z1 - blade for z0, z1 in SLOT_ROWS)
        print(f"  {name} screw at x {sx:.2f}, z {sz:.2f}: {wall}-side slot {cols.index(col) + 1}, "
              f"{'reachable' if ok_x and ok_z else 'NOT reachable'}")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    base, lid = make_base(), make_lid()
    refs = make_references()
    sensors, lengths = make_sensors()

    print(f"case outside {X1 - X0:.1f} x {Y1 - Y0:.1f} x {LID_Z + LID_T:.2f} mm (v3: 105 x 86 x 40.03)")
    for part in (base, lid):
        bb = part.bounding_box()
        print(f"{part.label:5} valid={part.is_valid}  {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.2f} mm"
              f"  volume {part.volume / 1000:.1f} cm3")
        export_step(part, OUT / f"{part.label}.step")

    every = {**refs, **sensors}
    print("overlap with the case (mm3):")
    for name, shape in every.items():
        print(f"  {name:12} base {overlap(base, shape):7.2f}   lid {overlap(lid, shape):7.2f}")
    print(f"  base/lid     {overlap(base, lid):.2f}")
    print("overlap between parts (mm3, only pairs above 0.01):")
    names = [n for n in every if n not in ("rubber_feet",)]
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = sum(overlap(sa, every[b]) for sa in every[a].solids())
            if v > 0.01:
                print(f"  {a} / {b}: {v:.2f}")
    print("screws behind the side slots:")
    check_slots()
    print("cables (path length vs wire available, lowest and highest point):")
    for name, (length, wire, low, top) in lengths.items():
        print(f"  {name}: {length:.1f} of {wire:.0f} mm, z {low:.1f} to {top:.1f} (floor {FLOOR:.1f}, lid {LID_Z:.0f})")

    on_lid = {"lid", "speaker"}
    parts = [
        ("base", base, "#ddd5c0", 1.0, "print"),
        ("lid", lid, "#ddd5c0", 1.0, "print"),
        ("scd41", sensors["scd41"], "#3a7bd5", 1.0, "sensor"),
        ("sht45", sensors["sht45"], "#2da44e", 1.0, "sensor"),
        ("bmp580", sensors["bmp580"], "#8250df", 1.0, "sensor"),
        ("sgp40", sensors["sgp40"], "#bf3989", 1.0, "sensor"),
        ("veml7700", sensors["veml7700"], "#d4a72c", 1.0, "sensor"),
        ("plugs", sensors["plugs"], "#f5f5f5", 1.0, "sensor"),
        ("cables", sensors["cables"], "#cf222e", 1.0, "sensor"),
        ("esp32", refs["esp32"], "#2d333b", 1.0, "ref"),
        ("esp32_plugs", refs["esp32_plugs"], "#57606a", 1.0, "ref"),
        ("oled", refs["oled"], "#1d3557", 1.0, "ref"),
        ("button", refs["button"], "#cf222e", 1.0, "ref"),
        ("button_nut", refs["button_nut"], "#b8bec6", 1.0, "ref"),
        ("led", refs["led"], "#f0f0f0", 1.0, "ref"),
        ("speaker", refs["speaker"], "#d85a30", 1.0, "ref"),
        ("screws", refs["screws"], "#8c959f", 1.0, "ref"),
        ("rubber_feet", refs["rubber_feet"], "#24292f", 1.0, "ref"),
    ]
    data = {"stamp": f"v4 compact, {datetime.now():%d %b %Y %H:%M}, build123d {build123d.__version__}", "parts": []}
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
