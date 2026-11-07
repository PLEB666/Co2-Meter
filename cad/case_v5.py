"""CO2 monitor case v5: the v4 case with the Adafruit 4741 1.5in OLED, built with build123d.

Run from the project folder:
    cad\\.venv\\Scripts\\python.exe cad\\case_v5.py

Writes cad/out/v5/:
    base.step, lid.step   the files for the school's printer (one STEP per part, mm)
    *.stl                 meshes of every part, for Cura and the viewer
    viewer.html           open it in a browser to turn the model around

What changed from v4 (cad/case_v4.py). The outside is 86.3 x 77 x 41.43 (v4: 86.3 x 72 x 40.03):
- The SGP40/BMP580 compartment is 18 mm deep like the sensor chamber, so the case is 5 mm wider.
- The display is the Adafruit 4741 (1.5in, 128 x 128, SSD1327), turned 90 degrees so that its
  35.56 mm side is the height. It stands in a 1 mm groove in the floor behind the front wall with
  its glass in a 1 mm recess, and spans the front of the sensor chamber and part of the electronics
  bay. Turned, its STEMMA QT sockets face the floor and the lid. The top one is used: a plug with
  its cable bent tight needs 4 mm (measured), so the lid has a pocket over it and the case is 1.4 mm
  taller than v4. The display is the end of the I2C chain. Two M2.5 screws from behind hold its header
  end to spacers on the front wall.
- A double cross wall behind the display closes the sensor chamber, which gets 10 mm shorter. The
  SCD-41 and SHT45 move back, and the side slots move with them.
- The two front lid screws move 10 mm back: one into the cross wall, one hanging from the
  column-j wall above the button.
- The VEML7700 (5378) stands on its edge right next to the display, 4.2 mm above the floor so that
  both its sockets take plugs, and looks out of an upright window. It is screwed (M2.5 + nut) to a
  wall that carries on from the display's rail; a gap behind it lets the LED and button wires through.
- All vents are the same 4 x 12 mm slots (walls and floor); the ESP32's back vents are centred on it.
- Printed without supports: holes in walls have pointed tops, parts hanging from a wall have 45 degree
  undersides.
- One STEMMA QT chain, no soldering: ESP32 -> SGP40 -> BMP580 -> light sensor (top socket in,
  bottom socket out) -> hole in the double wall -> SHT45 -> SCD-41 -> back along the chamber floor
  and out through the same hole -> display.
- The button and LED stay in their column, 7 mm above and below the middle of the display. The
  LED hole is round.
- The SGP40/BMP580 compartment gets a double wall like the chamber's, from the back wall, open at
  the front. The ESP32's I2C cable goes through a hole at its bottom, at the rear end. The ESP32
  sits 2 mm nearer the chamber's double wall to make room beside it.
- All 4 ESP32 posts are the same and all 4 corners are screwed (v4's 4th post was cut flat on the
  module's side and only carried the board).

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
OUT = HERE / "out" / "v5"
ADAFRUIT = HERE / "adafruit"

# ---- printing (docs "Printing at school")
LAYER = 0.254              # Stratasys F370, standard layer
FIT = 0.3                  # gap around parts that drop in

# ---- shell: as v4
WALL = 2.0
FLOOR = 8 * LAYER          # 2.03
LID_T = 8 * LAYER
CORNER_R = 2.0
# Top of the base = underside of the lid. The 4741's top STEMMA QT socket faces the lid, and a plug
# with its cable bent as tight as it goes reaches 4 mm out of a socket (measured). The display stands
# in a 1 mm groove in the floor and the lid has a pocket over the plug, so the case grows by 1.4 mm.
LID_Z = 39.4
LID_POCKET = 5 * LAYER     # 1.27, leaves 3 layers of lid over the plug
PLUG_ROOM = 4.0            # socket face to the far side of the bent cable (measured)
L = 82.3
BAY_W = 51.0               # 5 more than v4: the column-j compartment is 18 deep, as the chamber
CHAMBER_D = 18.0
DOUBLE_WALL = ((-4.0, -2.8), (-1.2, 0.0))   # 2 x 1.2 with a 1.6 air gap
DW_OUT = DOUBLE_WALL[0][0]                  # its chamber face
X0, X1 = -WALL, L + WALL
Y0, Y1 = DW_OUT - CHAMBER_D - WALL, BAY_W + WALL
Y_OUTER_IN = Y0 + WALL                      # inside of the chamber's outer wall

# ---- the chamber ends at a double cross wall behind the display (same skins and gap as the long wall)
CROSS_WALL = ((72.0, 73.2), (74.8, 76.0))   # chamber skin, bay skin (x)

# ---- lid screws: 4 x M3 x 10. The back two fill the inside corners from the floor up (with the
# 18 mm column-j compartment the SGP40's plug is clear of its corner). The front two sit in the middle
# of the cross wall's x, out of the display's way: one in the cross wall, one on the column-j wall
# beside the button; both from the floor up as well.
BOSS_INSET, BOSS_R = 2.0, 3.0
BACK_CORNERS = [(0.0, Y_OUTER_IN, 1, 1), (0.0, BAY_W, 1, -1)]
BACK_HANGING_Z = {}                         # index in BACK_CORNERS: lowest point, at the walls (none now)
FRONT_BOSS_X = sum(CROSS_WALL[0] + CROSS_WALL[1]) / 4
FRONT_BOSSES = [(FRONT_BOSS_X, Y_OUTER_IN + BOSS_INSET), (FRONT_BOSS_X, BAY_W - BOSS_INSET)]
BOSSES = [(x + sx * BOSS_INSET, y + sy * BOSS_INSET) for x, y, sx, sy in BACK_CORNERS] + FRONT_BOSSES
BOSS_PILOT, BOSS_HOLE_DEPTH = 1.8, 12.0     # drill to 2.5
LID_PILOT = 2.5                             # drill to 3.2

FOOT_D, FOOT_H = 12.7, 3.5
FEET = [(x, y) for x in (X0 + 7.5, X1 - 7.5) for y in (Y0 + 7.5, Y1 - 7.5)]

# ---- chamber sensors (Adafruit, upright, header edge down, parts towards the outer wall)
BOARD_T = 1.57
BOARD_BACK_Y = DW_OUT - 6.0            # back face of the boards; standoffs between it and the double wall
STANDOFF_W = 5.0
STANDOFF_PILOT = 1.6                   # teardrop, drilled to 2.1 through the slot; M2.5 x 6
STANDOFF_PILOT_DEPTH = 5.0
HOLE_IN = 2.54
BOARD_Z = 7.0
BOARD_GAP = 8.47                       # between two boards: room for two STEMMA QT plugs
# The SCD-41 sits at the same x as the SGP40 on the other side, so both side walls get the same slots.
# That is as far forward as it goes: the SHT45's front plug and its bent cable end 1.4 before the cross wall.
SCD_X = 7.0
SHT_X = SCD_X + 25.4 + BOARD_GAP

# ---- vents: 4 mm slots in two rows, columns a third of 20.32 apart.
SLOT_W, SLOT_PITCH = 4.0, 20.32 / 3
SLOT_ROWS = ((5.5, 17.5), (20.5, 32.5))
# chamber side: 9 columns on the sensors' screws (SCD-41 on 1/4, SHT45 on 6/9)
SLOT_X = [SCD_X + HOLE_IN + k * SLOT_PITCH for k in range(9)]
END_SLOT_MID = (Y_OUTER_IN + BOSS_INSET + BOSS_R + DW_OUT) / 2
END_SLOT_Y = (END_SLOT_MID - SLOT_PITCH / 2, END_SLOT_MID + SLOT_PITCH / 2)
# floor vents: the same 4 x 12 slots as in the walls. In the chamber on columns 2..8 as in v4, from the
# outer wall to the boards' back face (the standoffs start there).
VENT_L = SLOT_ROWS[0][1] - SLOT_ROWS[0][0]
FLOOR_VENT_X = SLOT_X[1:-1]
FLOOR_VENT_W, FLOOR_VENT_Y = SLOT_W, (BOARD_BACK_Y - VENT_L, BOARD_BACK_Y)

# ---- cable hole at the bottom of the double wall, just behind the cross wall: the chain comes from
# the light sensor into the SHT45's front socket. Pointed top: prints without support. Seal after wiring.
CABLE_HOLE_X = (SLOT_X[8] + STANDOFF_W / 2 + 0.05, CROSS_WALL[0][0])
CABLE_HOLE_H = 4.0
CHAMBER_HOLE_H = 5.0       # this one is narrower (5.7): a plug passes on edge

# ---- column-j compartment: as v4. SGP40 (back, fitted later) and BMP580 (front), upright, header
# edge down, parts towards the wall, screwed through the column-j slots.
# The compartment is 18 deep (wall to outer wall), the same as the chamber: the case is 5 mm wider
# for it, because the ESP32 bay can't give any room (its I2C cable drops between the board edge and
# the wall). The posts are as deep as the chamber's standoffs, so the boards' parts side is 10.43 from
# the outer wall, as in the chamber.
J_COMP_D = CHAMBER_D
J_POST_D = DW_OUT - BOARD_BACK_Y               # 6.0, as in the chamber
J_BACK_Y = BAY_W - J_COMP_D + J_POST_D
J_FRONT_Y = J_BACK_Y + BOARD_T
J_WALL_GAP = BAY_W - J_FRONT_Y
J_Z = 9.0
SGP_X0 = SCD_X
BMP_X0 = SGP_X0 + 25.4 + BOARD_GAP
J_BOARDS = {"SGP40": SGP_X0, "BMP580": BMP_X0}
J_SLOT_X = SLOT_X                      # same columns as the chamber side
# Double wall between the ESP32 and the compartment, as the chamber's (2 x 1.2, 1.6 air gap), against
# the posts and from the back wall. Under the speaker it stops 0.5 below it; elsewhere it reaches the
# lid. The ESP32-side skin runs to the BMP580's front post, where a cross piece joins it to the
# compartment skin and closes the air gap; the compartment skin goes on to just past the post.
_jp = J_BACK_Y - J_POST_D
J_DOUBLE_WALL = ((_jp - 4.0, _jp - 2.8), (_jp - 1.2, _jp))   # ESP32 skin, compartment skin
J_WALL_END = BMP_X0 + 25.4 - HOLE_IN + STANDOFF_W / 2 + 0.05  # just past the BMP580's front post
J_CAP_X = BMP_X0 + 25.4 - HOLE_IN - STANDOFF_W / 2           # 61.23: the cross piece ends at the post's back face
J_CAP_T = 1.2                                                # as thick as the skins
# Cable hole at the bottom, shaped like the chamber's, at the rear end: between the back wall and the
# SGP40's back post, so 6.3 wide instead of 8.
J_HOLE_X = (0.45, SGP_X0 + HOLE_IN - STANDOFF_W / 2 - 0.3)
# vents as the chamber's: 4 x 12 floor slots on columns 2..8 from the outer wall to the boards' back
# face; in the back wall 2 slots in each row, between the double wall and the lid boss in the corner.
J_FLOOR_VENT_X = SLOT_X[1:-1]
J_FLOOR_VENT_Y = (BAY_W - VENT_L, BAY_W)
J_END_SLOT_MID = (J_DOUBLE_WALL[1][1] + BAY_W - BOSS_INSET - BOSS_R) / 2
J_END_SLOT_Y = (J_END_SLOT_MID - SLOT_PITCH / 2, J_END_SLOT_MID + SLOT_PITCH / 2)

# ---- Waveshare ESP32-C6-DEV-KIT-N16, upside down: as v4
ESP_L, ESP_W, ESP_T = 55.0, 25.36, 1.6
# The board's USB end is 0.5 from the back wall as in v4. The USB-C socket sticks out 0.5 past the board
# edge (7 Nov: Berrybase's straight top-view photo 0.4-0.5, "almost flush"; Waveshare's drawing reads
# up to 0.9), so its face is flush with the inside of the back wall, 2.0 inside the outside face, and the
# plug's grip goes into the opening.
ESP_X0, ESP_Y0 = 0.5, 1.5              # y: 2 mm nearer the double wall than in v4: room for the I2C cable beside it
ESP_POST_H = 5.6
ESP_ZB = FLOOR + ESP_POST_H
ESP_ZT = ESP_ZB + ESP_T
ESP_HOLE_IN, ESP_HOLE_D = 1.77, 2.0
ESP_HOLES = [(ESP_HOLE_IN, ESP_HOLE_IN), (ESP_HOLE_IN, ESP_W - ESP_HOLE_IN),
             (ESP_L - ESP_HOLE_IN, ESP_HOLE_IN), (ESP_L - ESP_HOLE_IN, ESP_W - ESP_HOLE_IN)]
ESP_NECK_D, ESP_NECK_H, ESP_POST_D = 3.4, 3.6, 5.0
ESP_PILOT = 1.2
ESP_ROW_Y = {"P1": ESP_Y0 + 1.25, "P2": ESP_Y0 + ESP_W - 1.25}
HEADER_H, PIN_UP, JUMPER_L = 2.5, 6.0, 14.0
USED_PINS = {"P1": (1, 12, 13, 14, 15),
             "P2": (1, 5, 6, 10, 12, 15)}
# Module (x from the USB end, y from the P1 edge, height): ESP32-C6-WROOM-1, 18.0 wide per its datasheet,
# centred across the board (ASSUMED; Waveshare's photo suggested y 3.9..22.6). Centred, it clears the
# 3.4 necks of the two antenna-end posts by 0.21 on each side, so all 4 posts are the same.
ESP_MODULE = (29.2, 55.0, (ESP_W - 18.0) / 2, (ESP_W + 18.0) / 2, 3.2)
ESP_USB = (-0.5, 6.55, ESP_W / 2, 8.94, 3.26)
# RESET and BOOT, with their legs, from Waveshare's drawing (7 Nov): right at the long edges
ESP_BUTTONS = ((3.9, 9.1, 0.3, 3.3), (3.9, 9.1, ESP_W - 3.3, ESP_W - 0.3))


def pin_x(k):
    """x of pin k (1..16): pin 16 (NC) is 12.0 from the USB end, pin 1 (3V3 / GND) at the antenna end."""
    return ESP_X0 + 12.0 + (16 - k) * 2.54


USB_Y = ESP_Y0 + ESP_USB[2]
USB_Z = ESP_ZB - ESP_USB[4] / 2
# USB-C opening: the four cables' plug grips (7 Nov) are at most 12.0 wide (Samsung) and 6.5 thick
# (Ugreen, Toocki, Nintendo), so at least 0.5 per side for each. 7.5 is the most it can be: centred on
# the socket, its bottom edge is 0.22 above the floor.
USB_CUT_W, USB_CUT_H, USB_CUT_R = 13.0, 7.5, 1.0
ESP_FLOOR_VENT_X = [ESP_X0 + 12.0 + k * SLOT_PITCH for k in range(6)]
ESP_FLOOR_VENT_Y = (USB_Y - VENT_L / 2, USB_Y + VENT_L / 2)       # under the middle of the board
# back wall over the ESP32: 4 slots centred on the board, upper row only (the USB port is below them)
BACK_SLOT_Y = [USB_Y + (k + 0.5) * SLOT_PITCH for k in (-2, -1, 0, 1)]

# ---- display: Adafruit 4741. Outline, holes and glass from its board file (Adafruit-Grayscale-1.5-inch-
# 128x128-OLED-PCB on GitHub). Board coordinates there: bx across (0..35.56), by up (main part
# 0..41.91, two ears below it to -5.08; header at by 39.37). Here it is turned 90 degrees: by runs along
# +y (ears at the chamber's outer wall, header end towards the button), bx runs down (bx 0 at the top).
OLED_LEN, OLED_HT, OLED_EAR = 46.99, 35.56, 5.08
OLED_Y0 = Y_OUTER_IN + 0.5             # ear tips
OLED_Z0 = FLOOR - 4 * LAYER            # stands in a 1.02 mm groove in the floor (4 layers of floor left)
OLED_RECESS = 1.0                      # the glass sits 1 mm deep in the front wall
OLED_GLASS_T, OLED_TAPE, OLED_PCB_T = 2.0, 0.2, 1.6     # glass and tape ASSUMED
OLED_GLASS_FRONT = L + OLED_RECESS
OLED_FRONT = OLED_GLASS_FRONT - OLED_GLASS_T - OLED_TAPE   # front face of the board
OLED_BACK = OLED_FRONT - OLED_PCB_T
OLED_GLASS = (1.007, 34.807, 0.419, 36.919)   # bx0, bx1, by0, by1: 33.8 x 36.5
OLED_LIT = (4.48, 31.34, 8.07, 34.92)         # 26.86 square, centred in the viewing-area outline
OLED_HOLES = [(bx, by) for bx in (2.54, 33.02) for by in (-2.54, 39.37)]
OLED_HEADER_BY = 39.37                        # 8 pads at bx 8.89 .. 26.67 (not used)
OLED_SOCKET_BY = 18.415                       # both STEMMA QT sockets (in parallel); the top one is used
OLED_SOCKET_IN = 0.45                         # socket face inside the board edge (board file: 0.454)
OLED_PEG_BY = (8.0, 30.0)                     # lid pegs on the board's top edge, clear of the socket
OLED_END_Y = OLED_Y0 + OLED_LEN               # header end
OLED_WIN_MARGIN = 1.5
RAIL_T, RAIL_LIP = 1.2, 1.0


def oled_y(by):
    return OLED_Y0 + OLED_EAR + by


def oled_z(bx):
    return OLED_Z0 + OLED_HT - bx


def oled_rect(r):
    """(y0, z0, y1, z1) of a board rectangle (bx0, bx1, by0, by1)."""
    bx0, bx1, by0, by1 = r
    return oled_y(by0), oled_z(bx1), oled_y(by1), oled_z(bx0)


_lit = oled_rect(OLED_LIT)
OLED_WIN = (_lit[0] - OLED_WIN_MARGIN, _lit[1] - OLED_WIN_MARGIN, _lit[2] + OLED_WIN_MARGIN, _lit[3] + OLED_WIN_MARGIN)
OLED_MID_Z = (OLED_WIN[1] + OLED_WIN[3]) / 2
# The display is screwed at its header end: 2 x M2.5 x 4 socket head from behind (an L-shaped 2 mm hex
# key from the bay) through its two holes there into square spacers on the front wall. Its other two
# holes are hidden behind the chamber's cross wall; that end stays held by the floor groove, the block
# behind its lower ear and the lid's tab.
OLED_SCREWS = [(oled_y(39.37), oled_z(bx)) for bx in (33.02, 2.54)]   # (y, z): lower, upper
OLED_SPACER_W = 4.4                    # board front face to the front wall: 1.2 deep
OLED_PILOT = 1.6                       # teardrop, drill to 2.1; 2.5 deep from the spacer face, 0.7 of the wall left
OLED_PILOT_END = L + 1.3

# ---- VEML7700 right angle (5378), standing on its edge right next to the display: the edge with the
# sensor against the front wall, the header edge at the back, parts towards the button (+y). Its back
# is screwed to a wall that carries on from the display's rail: 2 x M2.5 x 6 through its rear holes,
# heads on the bay side, nuts on its parts side; two floor posts under its bottom edge. Both sockets
# take plugs (the board sits 4.2 mm above the floor). Between the end of the double wall and this
# wall there is a gap for the LED's and the button's wires. Nothing needs support when printing.
VEML_W, VEML_D = 25.4, 17.78
VEML_X0 = L - 0.1 - VEML_D             # header edge; the sensor edge is 0.1 from the wall
VEML_FIN_T = 1.6
VEML_FIN_Y = (OLED_END_Y + 0.2, OLED_END_Y + 0.2 + VEML_FIN_T)   # the wall; also the display's rail
VEML_FIN_X0 = VEML_X0 - 1.0
WIRE_GAP = VEML_FIN_X0 - J_CAP_X       # 2.19, from the double wall's cross piece to the VEML wall
VEML_BACK_Y = VEML_FIN_Y[1]            # back face (no parts), on the wall
VEML_PARTS_Y = VEML_BACK_Y + BOARD_T
VEML_Z0 = FLOOR + 4.2                  # bottom edge
VEML_SENSOR = (9.26, 16.16, 2.5)       # along the edge from the bottom, and its height above the board
VEML_WINDOW_GAP = 0.3
VEML_SOCKET_X = VEML_X0 + VEML_D / 2   # both sockets (top and bottom edge)
VEML_SOCKET_Y = VEML_PARTS_Y + 1.48
VEML_SCREWS = [(VEML_X0 + HOLE_IN, VEML_Z0 + z) for z in (HOLE_IN, VEML_W - HOLE_IN)]
VEML_SCREW_HOLE = 2.7                  # M2.5 clearance, teardrop
VEML_NOTCH_X = (VEML_SOCKET_X - 4.5, VEML_SOCKET_X + 1.5)   # pointed hole in the wall for the bottom socket's cable

# ---- panel button, Berrybase 10130: as v4, at the display's middle + 7
# y 38.5, near the light sensor: the LED's sleeve stays 1.7 from its sockets, the button nut 1.8 and the
# LED rim 2.4 from its window
BUTTON_Y, BUTTON_Z = 38.5, OLED_MID_Z + 7.0
BUTTON_PILOT = 6.0                     # drill to 7.0 (M7 x 0.75)
BUTTON = {"cap": (7.4, 1.7), "stem": (3.4, 3.8), "thread": (6.7, 6.3), "collar": (9.7, 2.4),
          "body": (7.9, 7.0), "lugs": 6.3, "washer": (9.3, 1.1), "nut": (10.1, 1.7)}   # (diameter, length)

# ---- 8 mm addressable LED (Berrybase PL9823-F8): as v4, at the display's middle - 7. Sizes ASSUMED.
LED_Y, LED_Z = BUTTON_Y, OLED_MID_Z - 7.0
LED_D, LED_RIM_D, LED_RIM_T, LED_H = 8.0, 8.9, 1.0, 11.0
LED_PROUD = 1.5
LED_RIM_X = X1 + LED_PROUD - LED_H
LED_PILOT = 7.5                        # drill to 8.0 through the wall and the sleeve

# ---- speaker (3885) under the lid, over the ESP32: as v4
SPK_X, SPK_Y = 6.0, 6.5
SPK_Z = LID_Z - 0.5 - 6.57
SPK_HOLES = [(2.54, 2.54), (27.94, 2.54), (2.54, 34.29), (27.94, 34.29)]
SPK_STANDOFF_D, SPK_PILOT = 4.5, 1.2
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


def teardrop_x(x0, x1, y, z, d):
    """Horizontal hole along x with a 45 degree point on top."""
    r = d / 2
    return rod_x(x0, x1, y, z, d) + Pos((x0 + x1) / 2, y, z + r / math.sqrt(2)) * Rot(45, 0, 0) * Box(x1 - x0, r, r)


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
    x_wall = X1 - WALL

    # double wall along the chamber; it turns at the cross wall behind the display
    (ca0, ca1), (cb0, cb1) = CROSS_WALL
    (ya0, ya1), (yb0, yb1) = DOUBLE_WALL
    base += block(0, ya0, FLOOR, ca1, ya1, LID_Z)              # chamber skin
    base += block(0, yb0, FLOOR, cb1, yb1, LID_Z)              # bay skin
    base += block(ca0, Y_OUTER_IN, FLOOR, ca1, ya1, LID_Z)
    base += block(cb0, Y_OUTER_IN, FLOOR, cb1, yb1, LID_Z)
    hx0, hx1 = CABLE_HOLE_X
    w = hx1 - hx0
    base -= block(hx0, ya0 - 1, FLOOR, hx1, yb1 + 1, FLOOR + CHAMBER_HOLE_H)
    base -= Pos((hx0 + hx1) / 2, (ya0 + yb1) / 2, FLOOR + CHAMBER_HOLE_H) * Rot(0, 45, 0) * Box(
        w / math.sqrt(2), 6, w / math.sqrt(2))

    # lid bosses: back corners as v4; front ones in the cross wall and hanging from the column-j wall
    for i, (x, y, sx, sy) in enumerate(BACK_CORNERS):
        if i in BACK_HANGING_Z:
            z_low = BACK_HANGING_Z[i]
            boss = corner_boss(x, y, sx, sy, z_low, LID_Z)
            for normal in ((-sx, 0, 1), (0, -sy, 1)):
                boss = boss.split(Plane(origin=(x, y, z_low), z_dir=normal), keep=Keep.TOP)
        else:
            boss = corner_boss(x, y, sx, sy, FLOOR, LID_Z)
        base += boss
    (fx, fya), (_, fyj) = FRONT_BOSSES
    base += block(fx - BOSS_R, Y_OUTER_IN, FLOOR, fx + BOSS_R, fya, LID_Z) + rod(fx, fya, FLOOR, LID_Z, 2 * BOSS_R)
    base += block(fx - BOSS_R, fyj, FLOOR, fx + BOSS_R, BAY_W, LID_Z) + rod(fx, fyj, FLOOR, LID_Z, 2 * BOSS_R)
    for bx, by in BOSSES:
        base -= rod(bx, by, LID_Z - BOSS_HOLE_DEPTH, LID_Z + 1, BOSS_PILOT)

    # chamber sensors: standoffs against the double wall
    for _, wall, sx, sz in sensor_screws():
        if wall == "a":
            base += block(sx - STANDOFF_W / 2, BOARD_BACK_Y, FLOOR, sx + STANDOFF_W / 2, DW_OUT, sz + 3)
            base -= teardrop_y(sx, sz, BOARD_BACK_Y - 0.5, BOARD_BACK_Y + STANDOFF_PILOT_DEPTH, STANDOFF_PILOT)

    # column-j compartment: as v4
    post_y0 = J_BACK_Y - J_POST_D
    for name in J_BOARDS:
        posts = [(sx, sz) for n, wall, sx, sz in sensor_screws() if n == name]
        for sx, sz in posts:
            post = block(sx - STANDOFF_W / 2, post_y0, FLOOR, sx + STANDOFF_W / 2, J_BACK_Y, sz + 3)
            over_vent = J_FLOOR_VENT_Y[0] < J_BACK_Y and any(abs(sx - vx) < 0.01 for vx in J_FLOOR_VENT_X)
            if over_vent:                                                # 45 degree underside over the vent
                post = post.split(Plane(origin=(sx, post_y0, FLOOR), z_dir=(0, -1, 1)), keep=Keep.TOP)
            base += post
        for sx, sz in posts:
            base -= teardrop_y(sx, sz, J_BACK_Y - STANDOFF_PILOT_DEPTH, J_BACK_Y + 0.5, STANDOFF_PILOT)
    xs0, xs1 = SPK_X - 0.5, SPK_X + 30.48 + 0.5          # under the speaker
    for (y0, y1), x_end in zip(J_DOUBLE_WALL, (J_CAP_X, J_WALL_END)):
        base += block(0, y0, FLOOR, xs0, y1, LID_Z)
        base += block(xs0, y0, FLOOR, xs1, y1, SPK_Z - 0.5)
        base += block(xs1, y0, FLOOR, x_end, y1, LID_Z)
    base += block(J_CAP_X - J_CAP_T, J_DOUBLE_WALL[0][1], FLOOR, J_CAP_X, J_DOUBLE_WALL[1][0], LID_Z)   # closes the air gap
    hx0, hx1 = J_HOLE_X
    w = hx1 - hx0
    ja, jb = J_DOUBLE_WALL[0][0], J_DOUBLE_WALL[1][1]
    base -= block(hx0, ja - 1, FLOOR, hx1, jb + 1, FLOOR + CABLE_HOLE_H)
    base -= Pos((hx0 + hx1) / 2, (ja + jb) / 2, FLOOR + CABLE_HOLE_H) * Rot(0, 45, 0) * Box(
        w / math.sqrt(2), 6, w / math.sqrt(2))

    # ESP32 posts: 4 the same (v4 cut the 4th flat on the module's side and left it unscrewed)
    neck_z = ESP_ZB - ESP_NECK_H
    for i in range(len(ESP_HOLES)):
        x, y = esp_hole(i)
        base += rod(x, y, FLOOR, neck_z, ESP_POST_D) + rod(x, y, neck_z, ESP_ZB, ESP_NECK_D)
        base -= rod(x, y, ESP_ZB - 5.0, ESP_ZB + 1, ESP_PILOT)

    # display: recess for its glass (open at the top), rail and lip at the header end, and a block
    # behind the lower ear (the lid holds the upper one)
    gy0, gz0, gy1, gz1 = oled_rect(OLED_GLASS)
    base -= block(x_wall - 1, gy0 - 0.2, gz0 - 0.3, x_wall + OLED_RECESS, gy1 + 0.2, LID_Z + 1)
    base -= block(OLED_BACK - 3.3, OLED_Y0 - FIT, OLED_Z0, x_wall - 1, OLED_END_Y + 0.2, FLOOR + 0.1)  # groove: board + lower socket
    # the VEML wall is the rail; its lip runs between the display's two screw heads, the underside
    # rising at 45 degrees from the wall
    x_lip = OLED_BACK - FIT
    (_, z_lo), (_, z_hi) = OLED_SCREWS
    lip = block(x_lip - RAIL_T, OLED_END_Y - RAIL_LIP, z_lo + 2.45, x_lip, VEML_FIN_Y[0], z_hi - 2.75)
    base += lip.split(Plane(origin=(0, VEML_FIN_Y[0], z_lo + 2.45), z_dir=(0, 1, 1)), keep=Keep.TOP)
    # spacers on the front wall for the display's screws (the lower one stands in the groove, the upper
    # one has a 45 degree underside), with the pilots
    w = OLED_SPACER_W / 2
    for sy, sz in OLED_SCREWS:
        if sz < OLED_MID_Z:
            base += block(OLED_FRONT, sy - w, OLED_Z0, x_wall, sy + w, sz + w)
        else:
            sp = block(OLED_FRONT, sy - w, sz - w - (x_wall - OLED_FRONT), x_wall, sy + w, sz + w)
            base += hanging(sp, x_wall, sz - w - (x_wall - OLED_FRONT), -1)
        base -= teardrop_x(OLED_FRONT - 0.5, OLED_PILOT_END, sy, sz, OLED_PILOT)
    base += block(CROSS_WALL[1][1] - 0.5, Y_OUTER_IN, OLED_Z0, x_lip, oled_y(0) - FIT, OLED_Z0 + OLED_EAR - 0.5)

    # light sensor: its wall (from the display's rail back past its header edge) with 2 teardrop screw
    # holes and a pointed hole at the bottom for the bottom socket's cable; two floor posts under its
    # bottom edge; ribs on the front wall beside its sensor (the upper one with a 45 degree underside)
    fy0, fy1 = VEML_FIN_Y
    fin = block(VEML_FIN_X0, fy0, FLOOR, x_wall, fy1, LID_Z - 0.5)
    nx0, nx1 = VEML_NOTCH_X
    w = nx1 - nx0
    fin -= block(nx0, fy0 - 1, FLOOR, nx1, fy1 + 1, FLOOR + CABLE_HOLE_H)
    fin -= Pos((nx0 + nx1) / 2, (fy0 + fy1) / 2, FLOOR + CABLE_HOLE_H) * Rot(0, 45, 0) * Box(
        w / math.sqrt(2), 4, w / math.sqrt(2))
    for sx, sz in VEML_SCREWS:
        fin -= teardrop_y(sx, sz, fy0 - 0.5, fy1 + 0.5, VEML_SCREW_HOLE)
    base += fin
    vp = VEML_PARTS_Y
    for xa, xb in ((VEML_X0 + 0.5, nx0 - 0.5), (VEML_SOCKET_X + 4.5, x_wall - 0.5)):
        base += block(xa, fy0, FLOOR, xb, vp, VEML_Z0)
    s0, s1, sh = VEML_SENSOR
    base += block(x_wall - 2.0, vp + 0.15, FLOOR, x_wall, vp + 1.35, VEML_Z0 + s0 - 1.0)
    upper = block(x_wall - 2.0, vp + 0.15, VEML_Z0 + s1 + 1.0, x_wall, vp + 1.35, VEML_Z0 + VEML_W - 2.0)
    base += hanging(upper, x_wall, VEML_Z0 + s1 + 1.0, -1)
    g = VEML_WINDOW_GAP
    base -= block(x_wall - 0.5, vp - g, VEML_Z0 + s0 - g, X1 + 1, vp + sh + g, VEML_Z0 + s1 + g)

    # LED sleeve: stands on the floor behind the front wall; round hole (the printer's soluble support fills it)
    base += block(LED_RIM_X + LED_RIM_T, LED_Y - 5.0, FLOOR, x_wall, LED_Y + 5.0, LED_Z + LED_D / 2 + 1.5)
    base -= rod_x(LED_RIM_X, X1 + 1, LED_Y, LED_Z, LED_PILOT)

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
    for z0, z1 in SLOT_ROWS:
        for y in J_END_SLOT_Y:
            base -= block(X0 - 1, y - SLOT_W / 2, z0, X0 + WALL + 0.5, y + SLOT_W / 2, z1)
    for x in J_FLOOR_VENT_X:
        base -= block(x - FLOOR_VENT_W / 2, J_FLOOR_VENT_Y[0], -1, x + FLOOR_VENT_W / 2, J_FLOOR_VENT_Y[1], FLOOR + 0.5)
    for x in FLOOR_VENT_X:
        base -= block(x - FLOOR_VENT_W / 2, FLOOR_VENT_Y[0], -1, x + FLOOR_VENT_W / 2, FLOOR_VENT_Y[1], FLOOR + 0.5)
    for x in ESP_FLOOR_VENT_X:
        base -= block(x - FLOOR_VENT_W / 2, ESP_FLOOR_VENT_Y[0], -1, x + FLOOR_VENT_W / 2, ESP_FLOOR_VENT_Y[1], FLOOR + 0.5)

    # USB-C port, display window, button hole
    cut = block(X0 - 1, USB_Y - USB_CUT_W / 2, USB_Z - USB_CUT_H / 2, 1, USB_Y + USB_CUT_W / 2, USB_Z + USB_CUT_H / 2)
    base -= fillet(cut.edges().filter_by(Axis.X), USB_CUT_R)
    y0, z0, y1, z1 = OLED_WIN
    base -= block(x_wall - 1, y0, z0, X1 + 1, y1, z1)
    base -= rod_x(x_wall - 1, X1 + 1, BUTTON_Y, BUTTON_Z, BUTTON_PILOT)

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
    # a tab behind the display's upper ear
    lid += block(FRONT_BOSS_X + BOSS_R + FIT, Y_OUTER_IN + 0.2, OLED_Z0 + OLED_HT - OLED_EAR + 0.5,
                 OLED_BACK - FIT, oled_y(0) - FIT, LID_Z)
    # pocket over the display's STEMMA QT plug and the bend of its cable; pegs on the display's top edge
    ys = oled_y(OLED_SOCKET_BY)
    lid -= block(CROSS_WALL[0][0] + 1.5, ys - 3.6, LID_Z - 1, OLED_BACK + 0.5, ys + 3.6, LID_Z + LID_POCKET)
    for by in OLED_PEG_BY:
        lid += rod((OLED_BACK + OLED_FRONT) / 2, oled_y(by), OLED_Z0 + OLED_HT + 0.2, LID_Z, 2.0)
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


def plug_x(x_open, out, y_mid, z_mid, length=PLUG_OUT):
    """Plug in a socket of an upright board (socket 6.0 along z, 2.96 along y), sticking out along x."""
    x_end = x_open + out * length
    return block(min(x_open, x_end), y_mid - 1.4, z_mid - 2.8, max(x_open, x_end), y_mid + 1.4, z_mid + 2.8)


def plug_z(x_mid, y_mid, z_open, out, length=PLUG_OUT, along_y=False):
    """Plug in a socket on the top or bottom edge of a standing board (6.0 along x, or along y, by 2.96)."""
    z_end = z_open + out * length
    hx, hy = (1.4, 2.8) if along_y else (2.8, 1.4)
    return block(x_mid - hx, y_mid - hy, min(z_open, z_end), x_mid + hx, y_mid + hy, max(z_open, z_end))


def make_sensors():
    """Sensor boards, plugs and cables. Returns parts and the cable lengths."""
    scd = upright(import_step(ADAFRUIT / "5187 SCD-40 C02 Sensor.step"), SCD_X, BOARD_Z)
    sht = upright(import_step(ADAFRUIT / "5665 SHT45 Sensor.step"), SHT_X, BOARD_Z)
    bmp = upright_j(import_step(ADAFRUIT / "4816 BMP390 QT.step"), BMP_X0, J_Z)
    sgp = upright_j(import_step(ADAFRUIT / "4829 SGP40 Sensor.step"), SGP_X0, J_Z)
    # the model's y (header edge 0 -> sensor edge) runs along +x, its x runs up, its parts face +y
    veml = Pos(VEML_X0, VEML_BACK_Y, VEML_Z0) * Rot(-90, 0, 0) * Rot(0, 0, -90) * import_step(
        ADAFRUIT / "5378 VEML7700 Lux Sensor.step")

    y_sock = BOARD_BACK_Y - BOARD_T - 1.48          # middle of a socket of a chamber board
    sht_back = (SHT_X + 0.70, BOARD_Z + 8.89)       # socket faces (from the models)
    sht_front = (SHT_X + 24.70, BOARD_Z + 8.89)
    scd_front = (SCD_X + 25.34, BOARD_Z + 11.43)
    scd_back = (SCD_X + 0.06, BOARD_Z + 11.43)
    y_j = J_FRONT_Y + 1.48                          # middle of a socket of a column-j board
    zj = J_Z + 8.89
    sgp_back, sgp_front = SGP_X0 + 0.06, SGP_X0 + 25.34
    bmp_back, bmp_front = BMP_X0 + 0.19, BMP_X0 + 25.21
    vtop, vbot = VEML_Z0 + VEML_W - 0.19, VEML_Z0 + 0.19          # the light sensor's two sockets
    ox, oy, oz = OLED_BACK - 1.48, oled_y(OLED_SOCKET_BY), oled_z(OLED_SOCKET_IN)   # display, top socket

    sockets = {
        "SHT45 front": ("x", sht_front[0], 1, y_sock, sht_front[1]),
        "SHT45 back": ("x", sht_back[0], -1, y_sock, sht_back[1]),
        "SCD-41 front": ("x", scd_front[0], 1, y_sock, scd_front[1]),
        "SCD-41 back": ("x", scd_back[0], -1, y_sock, scd_back[1]),
        "SGP40 back": ("x", sgp_back, -1, y_j, zj),
        "SGP40 front": ("x", sgp_front, 1, y_j, zj),
        "BMP580 back": ("x", bmp_back, -1, y_j, zj),
        "BMP580 front": ("x", bmp_front, 1, y_j, zj),
        "VEML7700 top": ("z", VEML_SOCKET_X, VEML_SOCKET_Y, vtop, 1, False),
        "VEML7700 bottom": ("z", VEML_SOCKET_X, VEML_SOCKET_Y, vbot, -1, False),
        "display top": ("z", ox, oy, oz, 1, True),
    }

    def socket_block(spec, length):
        kind, *a = spec
        if kind == "x":
            return plug_x(*a, length=length)
        x, y, z, out, along_y = a
        return plug_z(x, y, z, out, length=length, along_y=along_y)

    plugs = [socket_block(s, PLUG_OUT) for s in sockets.values()]
    rooms = {name: socket_block(s, PLUG_ROOM) for name, s in sockets.items()}   # plug + bent cable (measured)

    lengths, groups = {}, {}

    def note(name, parts, edge, wire):
        groups[name] = parts
        zs = [edge.position_at(i / 200).Z for i in range(201)]
        lengths[name] = (edge.length, wire, min(zs) - CABLE_D / 2, max(zs) + CABLE_D / 2)

    # The chain: ESP32 -> SGP40 -> BMP580 -> light sensor -> SHT45 -> SCD-41 -> display. The display is
    # the end: turned, only its top socket takes a plug.

    # chamber: SHT45 back socket -> SCD-41 front socket (50 mm), loop up towards the outer wall
    tube, edge = cable((sht_back[0] - PLUG_OUT, y_sock, sht_back[1]), (scd_front[0] + PLUG_OUT, y_sock, scd_front[1]))
    note("SHT45 - SCD-41 (50 mm)", [tube], edge, CABLE_WIRE)

    # column-j compartment: BMP580 back socket -> SGP40 front socket (50 mm), loop down in the gap
    tube, edge = cable((bmp_back - PLUG_OUT, y_j, zj), (sgp_front + PLUG_OUT, y_j, zj), bulge_y=J_FRONT_Y - 0.5, up=False, lean=0.9)
    note("BMP580 - SGP40 (50 mm, later)", [tube], edge, CABLE_WIRE)

    # BMP580 front socket -> light sensor's top socket (50 mm): under the button's lugs, past the
    # end of the wall, up behind the light sensor and down into its socket
    a = (bmp_front + PLUG_OUT, y_j, zj)
    top = (VEML_SOCKET_X, VEML_SOCKET_Y, vtop + PLUG_OUT + 1.5)
    tube, edge = routed([a, (71.2, y_j - 0.98, 19.0), (70.5, 38.0, 21.5), (69.8, 34.6, 25.0), (68.6, 36.6, 31.5),
                         (70.0, 33.9, 36.6), (72.5, 31.0, 36.8), top], (1, 0, 0), (0, 0, -1))
    note("BMP580 - light sensor (50 mm)", [tube, stub(top, (VEML_SOCKET_X, VEML_SOCKET_Y, vtop + PLUG_OUT))], edge, CABLE_WIRE)

    # light sensor's bottom socket -> through the hole in its wall -> along the floor -> hole in the
    # double wall -> SHT45 front socket (100 mm)
    a = (VEML_SOCKET_X, VEML_SOCKET_Y, vbot - PLUG_OUT)
    end = (sht_front[0] + PLUG_OUT, y_sock, sht_front[1])
    nx = sum(VEML_NOTCH_X) / 2
    tube, edge = routed([a, (VEML_SOCKET_X - 0.4, VEML_SOCKET_Y - 0.8, FLOOR + 0.95), (nx + 0.3, VEML_FIN_Y[1] + 0.6, FLOOR + 0.95),
                         (nx, VEML_FIN_Y[0] - 0.6, FLOOR + 1.0), (70.0, 22.0, FLOOR + 1.0), (68.8, 16.0, FLOOR + 1.0),
                         (68.3, 6.0, FLOOR + 1.0), (68.2, 1.0, FLOOR + 1.3), (68.2, -2.0, FLOOR + 1.6),
                         (68.5, -6.5, 6.5), (69.4, -11.0, 11.5), (end[0] + 2.0, y_sock, end[2])], (0, 0, -1), (-1, 0, 0))
    note("light sensor - SHT45 (100 mm)", [tube, stub((end[0] + 2.0, y_sock, end[2]), end)], edge, 91.0)

    # SCD-41 back socket -> down, along the chamber floor under the boards, through the hole in the
    # double wall, up behind the display -> its top socket, from above (200 mm)
    # (the tight bends at both plugs are drawn as a short straight bit: too tight for a smooth sweep)
    a = (scd_back[0] - PLUG_OUT, y_sock, scd_back[1])
    sb = scd_back[0]
    turn = (sb - 3.84, y_sock, scd_back[1])
    above = (ox, oy, oz + PLUG_OUT + 0.25)
    zf = FLOOR + 1.8
    floor = [(x, -11.3, zf) for x in (sb + 0.71, sb + 3.21, 14.0, 22.0, 30.0, 40.0, 50.0, 58.0, 64.5)]
    tube, edge = routed([turn, (sb - 3.69, -12.4, 13.0), (sb - 3.09, -11.7, 8.0), (sb - 1.59, -11.3, zf + 0.7)] + floor + [(69.0, -8.5, zf), (70.4, -3.0, zf + 0.2), (70.6, 1.5, zf + 0.5),
                         (72.6, 2.6, 12.0), (73.4, 2.4, 30.0), (74.0, 2.2, 37.0), (76.0, 2.0, 39.4), above],
                        (0, 0, -1), (1, 0, 0))
    note("SCD-41 - display (200 mm)", [stub(a, turn), tube, stub(above, (ox, oy, oz + PLUG_OUT))], edge, 191.0)

    # ESP32 pins -> SGP40 back socket: STEMMA QT cable with female sockets, 150 mm. From above the
    # SDA/SCL plugs down beside the ESP32, along the floor to the back, through the hole at the rear end
    # of the double wall and up to the SGP40's back socket. Without the SGP40 it goes on along the
    # compartment floor, under where the SGP40 would be, to the BMP580's back socket.
    top = ESP_ZT + HEADER_H + JUMPER_L
    sx = (pin_x(5) + pin_x(6)) / 2
    end = (sgp_back - PLUG_OUT, y_j, zj)
    jx = sum(J_HOLE_X) / 2
    yd = (ESP_Y0 + ESP_W + J_DOUBLE_WALL[0][0]) / 2     # between the ESP32 and the wall
    zf = FLOOR + 2.2
    tube, edge = routed([(sx, ESP_ROW_Y["P2"], top + 1.0), (sx - 0.4, ESP_ROW_Y["P2"] + 1.0, top + 2.4),
                         (sx - 0.9, yd - 0.4, top + 0.8), (sx - 1.4, yd, top - 4.0), (sx - 1.9, yd, 10.0), (sx - 2.7, yd, zf + 1.5), (sx - 5.0, yd, zf),
                         (20.0, yd, zf), (9.0, yd, zf), (jx + 1.2, yd + 0.6, zf), (jx + 0.2, J_DOUBLE_WALL[0][0] + 1.5, zf),
                         (jx - 0.2, J_DOUBLE_WALL[1][1] + 1.5, zf + 0.3), (jx - 1.0, 38.0, 8.0), (1.8, y_j - 0.88, 13.5),
                         (1.6, y_j - 0.3, zj - 1.0), (end[0] - 1.2, y_j, zj)],
                        (0, 0, 1), (1, 0, 0))
    note("ESP32 - SGP40 (150 mm)", [tube, stub((end[0] - 1.2, y_j, zj), end)], edge, 150.0 - 9.0)

    tubes = [t for parts in groups.values() for t in parts]
    sensors = {"scd41": scd, "sht45": sht, "bmp580": bmp, "sgp40": sgp, "veml7700": veml,
               "plugs": Compound(plugs), "cables": Compound(tubes)}
    return sensors, lengths, groups, rooms


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
            plugs.append(rod(x, y, z + JUMPER_L, z + JUMPER_L + 4.0, 1.4))
    return Compound([pcb] + under + strips + tails), Compound(plugs)


def make_oled():
    """Adafruit 4741, turned 90 degrees: board, ears, holes and glass from its board file. The parts on
    its back are simplified to the two STEMMA QT sockets and one 2 mm block for the rest (ASSUMED)."""
    xb, xf = OLED_BACK, OLED_FRONT
    ye = oled_y(0)
    z0, z1 = OLED_Z0, OLED_Z0 + OLED_HT
    board = block(xb, ye, z0, xf, OLED_END_Y, z1)
    for tz0, tz1 in ((z0, z0 + OLED_EAR), (z1 - OLED_EAR, z1)):
        board += block(xb, OLED_Y0, tz0, xf, ye, tz1)
    for bx, by in OLED_HOLES:
        board -= rod_x(xb - 1, xf + 1, oled_y(by), oled_z(bx), 2.5)
    gy0, gz0, gy1, gz1 = oled_rect(OLED_GLASS)
    glass = block(xf + OLED_TAPE, gy0, gz0, OLED_GLASS_FRONT, gy1, gz1)
    ys = oled_y(18.415)
    back = [block(xb - 2.96, ys - 3.05, oled_z(b1), xb, ys + 3.05, oled_z(b0)) for b0, b1 in ((0.45, 4.7), (30.86, 35.11))]
    back.append(block(xb - 2.0, oled_y(10.0), oled_z(31.0), xb, oled_y(34.0), oled_z(6.0)))
    back += [rod_x(xb - 1.0, xb, oled_y(OLED_HEADER_BY), oled_z(17.78 + k * 2.54), 1.2) for k in (-1.5, -0.5, 0.5, 1.5)]
    return Compound([board, glass] + back)


def make_oled_lit():
    """The lit 128 x 128 area on the glass, to see where the picture is."""
    y0, z0, y1, z1 = oled_rect(OLED_LIT)
    return block(OLED_GLASS_FRONT, y0, z0, OLED_GLASS_FRONT + 0.05, y1, z1)


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
    # half sphere as an intersection: a split sphere gave wrong volumes in the overlap check
    dome = (Pos(body_end, LED_Y, LED_Z) * Sphere(LED_D / 2)) & block(
        body_end, LED_Y - LED_D, LED_Z - LED_D, body_end + LED_D, LED_Y + LED_D, LED_Z + LED_D)
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
    for i in range(len(ESP_HOLES)):                   # M2 x 6 button head (ISO 7380: 3.5 x 1.1)
        x, y = esp_hole(i)
        heads.append(rod(x, y, ESP_ZT, ESP_ZT + 1.1, 3.5))
    for _, wall, sx, sz in sensor_screws():           # M2.5 x 6 socket head, on the parts side
        if wall == "a":
            heads.append(Pos(sx, BOARD_BACK_Y - BOARD_T - 1.25, sz) * Rot(90, 0, 0) * Cylinder(2.25, 2.5))
        else:
            heads.append(Pos(sx, J_FRONT_Y + 1.25, sz) * Rot(90, 0, 0) * Cylinder(2.25, 2.5))
    for sy, sz in OLED_SCREWS:                         # M2.5 x 4 socket head, behind the display
        heads.append(rod_x(OLED_BACK - 2.5, OLED_BACK, sy, sz, 4.5))
    for sx, sz in VEML_SCREWS:                         # M2.5 x 6: head on the bay side of the wall, nut on the board
        heads.append(Pos(sx, VEML_FIN_Y[0] - 1.25, sz) * Rot(90, 0, 0) * Cylinder(2.25, 2.5))
        nut = Pos(sx, VEML_PARTS_Y + 1.0, sz) * Rot(90, 0, 0) * build123d.extrude(
            build123d.RegularPolygon(5.0 / math.sqrt(3), 6), amount=1.0, both=True)
        heads.append(nut - Pos(sx, VEML_PARTS_Y + 1.0, sz) * Rot(90, 0, 0) * Cylinder(1.25, 3.0))
    return Compound(heads)


def make_references():
    esp, jumpers = make_esp()
    button, nut = make_button()
    return {
        "esp32": esp,
        "esp32_plugs": jumpers,
        "oled": make_oled(),
        "oled_lit": make_oled_lit(),
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
    sensors, lengths, groups, rooms = make_sensors()

    print(f"case outside {X1 - X0:.1f} x {Y1 - Y0:.1f} x {LID_Z + LID_T:.2f} mm (v4: 86.3 x 72 x 40.03)")
    y0, z0, y1, z1 = OLED_WIN
    print(f"display window y {y0:.2f}..{y1:.2f}, z {z0:.2f}..{z1:.2f} ({y1 - y0:.2f} x {z1 - z0:.2f}); "
          f"button z {BUTTON_Z:.2f}, LED z {LED_Z:.2f}")
    print(f"column-j double wall y {J_DOUBLE_WALL[0][0]:.2f}..{J_DOUBLE_WALL[1][1]:.2f}, cable hole x "
          f"{J_HOLE_X[0]:.2f}..{J_HOLE_X[1]:.2f}; ESP32 y {ESP_Y0:.2f}..{ESP_Y0 + ESP_W:.2f}; back slots {len(BACK_SLOT_Y)}")
    for name in ("veml7700", "oled"):
        shape = sensors.get(name) or refs.get(name)
        bb = shape.bounding_box()
        print(f"{name}: x {bb.min.X:.2f}..{bb.max.X:.2f}, y {bb.min.Y:.2f}..{bb.max.Y:.2f}, z {bb.min.Z:.2f}..{bb.max.Z:.2f}")
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
    print(f"room for each plug and its bent cable ({PLUG_ROOM:.0f} mm from the socket face; mm3 of something in the way):")
    solid = [("base", base), ("lid", lid)] + [(n, s) for n, s in every.items() if n not in ("plugs", "cables")]
    for name, room in rooms.items():
        hits = [(n, overlap(s, room)) for n, s in solid]
        hits = [f"{n} {v:.2f}" for n, v in hits if v > 0.01]
        print(f"  {name:16} {'free' if not hits else ', '.join(hits)}")
    face = oled_z(OLED_SOCKET_IN)
    print(f"  display plug: socket face z {face:.2f}, bent cable to {face + PLUG_ROOM:.2f}, lid pocket {LID_Z + LID_POCKET:.2f}"
          f" (with the face 0.2 from the edge: {OLED_Z0 + OLED_HT - 0.2 + PLUG_ROOM:.2f})")
    print("cables touching each other (mm3, above 0.05):")
    names_c = list(groups)
    for i, a in enumerate(names_c):
        for b in names_c[i + 1:]:
            v = sum(overlap(sa, Compound(groups[b])) for sa in groups[a])
            if v > 0.05:
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
        ("oled_lit", refs["oled_lit"], "#5b7fa8", 1.0, "ref"),
        ("button", refs["button"], "#cf222e", 1.0, "ref"),
        ("button_nut", refs["button_nut"], "#b8bec6", 1.0, "ref"),
        ("led", refs["led"], "#f0f0f0", 1.0, "ref"),
        ("speaker", refs["speaker"], "#d85a30", 1.0, "ref"),
        ("screws", refs["screws"], "#8c959f", 1.0, "ref"),
        ("rubber_feet", refs["rubber_feet"], "#24292f", 1.0, "ref"),
    ]
    data = {"stamp": f"v5 with the 4741 OLED, {datetime.now():%d %b %Y %H:%M}, build123d {build123d.__version__}", "parts": []}
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
