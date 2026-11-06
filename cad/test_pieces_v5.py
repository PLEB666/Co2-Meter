"""Test pieces for case v5: small slices of the real base, printed before the whole case.

Run from the project folder (after cad\\case_v5.py, or on its own: it builds the base itself):
    cad\\.venv\\Scripts\\python.exe cad\\test_pieces_v5.py

Writes cad/out/v5/test_pieces/*.stl, ready to print floor down, as the base is printed. Each piece is
the base of case_v5.py cut by a box, so it follows every change there; run this again after changing
case_v5.py. The cuts are placed where they cross only plain walls and floor (no slot, post or hole is
cut in half); the check below says so if a cut leaves a loose bit.

The pieces (docs section 13, Printing at school):
- esp32_floor: back wall, floor and the 4 ESP32 posts, 14 mm high. USB-C opening with the real cable,
  the posts against the module and the parts under the board, the screw heads against the jumper plugs
  at the antenna end. Needs the Waveshare board.
- posts_sht45: the SHT45's two posts on a strip of floor, with the double wall's chamber skin behind
  them, cut off 1 mm above the posts. Drill the pilots, screw the board on: do the holes line up, does the
  thread hold? The other sensor boards have the same hole spacing (20.32); their posts are only taller
  (SGP40 and BMP580 by 2 mm, SCD-41 by 5 mm), so this one piece stands for all of them.

Every piece comes twice: as in the case, with pilot holes to drill, and as *_no_drill.stl with the screw
holes printed at their drilled size (DRILLED below), so the screws go straight in. The no-drill pieces
check the fits and the screws, but not the drilling.
"""

from contextlib import contextmanager

import case_v5 as c

OUT = c.OUT / "test_pieces"

ESP_X = 57.5      # past the ESP32's posts at the antenna end (56.23)
ESP_Y = (0.5, 28.5)   # from just past the chamber's double wall to between the board edge and the
                      # column-j double wall (26.86 and 29.0)
ESP_Z = 14.0      # board on its posts at 7.63..9.23, 4.25 mm of back wall over the USB opening

POST_MARGIN = 1.0             # floor and wall left beside the posts and above them
POST_Y = (c.BOARD_BACK_Y,     # the posts' board face (the chamber's floor vents end there)
          c.DOUBLE_WALL[0][1] + 0.8)   # into the double wall's air gap: its chamber skin stays, the floor under the gap too

BIG = 10.0        # boxes reach this far past the case where a side is not cut

# the screw holes at their drilled size (docs, Printing at school: the drill table), for the no-drill pieces
DRILLED = {"ESP_PILOT": 1.6,          # M2 x 6 into the ESP32 posts
           "STANDOFF_PILOT": 2.1}     # M2.5 x 6 into the sensor posts (teardrop, as the pilot)
NO_DRILL = "_no_drill"


@contextmanager
def sizes(**values):
    """Set case_v5's sizes for a while (make_base reads them when it runs)."""
    old = {name: getattr(c, name) for name in values}
    for name, value in values.items():
        setattr(c, name, value)
    try:
        yield
    finally:
        for name, value in old.items():
            setattr(c, name, value)


def box(x0=None, y0=None, z0=None, x1=None, y1=None, z1=None):
    """Box from the given limits; a limit left out lies outside the case."""
    return c.block(c.X0 - BIG if x0 is None else x0, c.Y0 - BIG if y0 is None else y0, -BIG if z0 is None else z0,
                   c.X1 + BIG if x1 is None else x1, c.Y1 + BIG if y1 is None else y1, 60.0 if z1 is None else z1)


def posts_box(board):
    """Box around one chamber board's two posts."""
    posts = [(sx, sz) for name, wall, sx, sz in c.sensor_screws() if name == board and wall == "a"]
    w = c.STANDOFF_W / 2
    x0 = min(sx for sx, _ in posts) - w - POST_MARGIN
    x1 = max(sx for sx, _ in posts) + w
    # the SHT45's front post stands just behind the cable hole at the foot of the double wall: stop at the post
    x1 = x1 + POST_MARGIN if x1 + POST_MARGIN < c.CABLE_HOLE_X[0] else x1
    z1 = max(sz for _, sz in posts) + 3 + POST_MARGIN       # posts end 3 above the screw (case_v5.make_base)
    return box(x0=x0, y0=POST_Y[0], x1=x1, y1=POST_Y[1], z1=z1)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    print("building the base, then again with the screw holes drilled ...")
    bases = {"": c.make_base()}
    with sizes(**DRILLED):
        bases[NO_DRILL] = c.make_base()

    cuts = [
        ("esp32_floor", box(x1=ESP_X, y0=ESP_Y[0], y1=ESP_Y[1], z1=ESP_Z)),
        ("posts_sht45", posts_box("SHT45")),         # the lowest posts: the smallest piece
    ]
    pieces = [(name + suffix, base & cut) for suffix, base in bases.items() for name, cut in cuts]
    for old in OUT.glob("*.stl"):                  # pieces dropped from the list must not linger
        if old.stem not in {name for name, _ in pieces}:
            old.unlink()
    volume = {}
    for name, piece in pieces:
        solids = piece.solids()
        bb = piece.bounding_box()
        volume[name] = piece.volume
        flag = "" if len(solids) == 1 else f"  {len(solids)} SOLIDS: a cut leaves a loose bit"
        print(f"  {name:21} valid={piece.is_valid}  x {bb.min.X:6.2f}..{bb.max.X:6.2f}  y {bb.min.Y:6.2f}..{bb.max.Y:6.2f}"
              f"  z {bb.min.Z:5.2f}..{bb.max.Z:5.2f}  ({bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f})"
              f"  {piece.volume / 1000:5.2f} cm3{flag}")
        c.export_stl(piece, OUT / f"{name}.stl", tolerance=0.02, angular_tolerance=0.2)
    print("plastic the drill takes out of each piece (mm3):")
    for name, _ in cuts:
        print(f"  {name:12} {volume[name] - volume[name + NO_DRILL]:6.2f}")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
