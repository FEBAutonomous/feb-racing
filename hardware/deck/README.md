# Platform deck for the RC car

The RoboRacer platform deck (`Platform_Deck_V3.1 (New Powerboard)` from the official parts
sheet's Google Drive folder) with four holes added for the RPLidar S2, so the lidar mounts
straight on the deck with no adapter plate.

| File | What |
|---|---|
| `Platform_Deck_V3.1_original.dxf` | the official file, untouched, for reference |
| `Platform_Deck_V3.1_S2.dxf` | the same plus the S2 holes: the file to cut |
| `Platform_Deck_V3.1_S2_laser_32x18in.pdf` | the cut file ready for the Jacobs Hall Universal lasers: a 32 x 18 in artboard, 1:1, red hairlines (cut), part 20 mm from the top-left corner |
| `Platform_Deck_V3.1_S2_check.png` | dimensioned check drawing |
| `Platform_Deck_V3.1_S2sq_A.dxf` | **current deck (2026-10-13)**: S2 holes square to the deck, Hokuyo holes removed |
| `Platform_Deck_V3.1_S2sq_B_slots.dxf` | same, but the eight rail-edge holes are 3 mm slots along the rail on a 127 mm outer span (the spacing measured on our BL-2s), with a half-round notch on the rear edge to tell it apart |
| `Platform_Deck_V3.1_S2sq_sheet_32x18in.pdf` | one Jacobs sheet: deck A top-left, deck B below it with a loose triangle beside it, a full-height cut down the middle so the blank right half comes off whole for later, and in the left half's spare space: nine FSAE cone keychains (outline and ring hole cut, stripe engraved), ten 50 x 40 mm team plaques with the FEB logo and FEBAUTO as black fills (raster engrave) and a hanging hole, and sixteen M3 spacer rings (6.35 mm thick, 3.3 mm hole). Red hairlines cut, black fills engrave |
| `make_sheet.py` | generates the three files above from `Platform_Deck_V3.1_S2.dxf` |

## Second cut (2026-10-13)
The first deck (rotated S2) fitted the chassis and the lidar, but the rotated lidar looked wrong,
so the Hokuyo UST/UTM holes are dropped and the S2 pattern is square: holes at (76.15, ±20.85)
and (117.85, ±20.85), 3.4 mm. Lidar yaw offset in the TF is now zero. Deck B exists because the
chassis standoffs measured about 127 mm apart against the deck's 123.85 mm; its slots cannot run
inboard because the cable slot edge is only 3 mm away (y = 63.5), so they run along the rail.
Everything else on the decks is unchanged from V3.1. Cutting the whole sheet in 0.25 in acrylic
takes two passes, about 40 minutes.
| `Powerboard_Adaptor_VX-V10.dxf` / `.stl` | the official "Powerboard Adaptor VX-V10" from the same Drive folder (Adaptor subfolder, 2024-07-30), untouched |
| `Powerboard_Adaptor_VX-V10_laser_32x18in.pdf` | the adaptor as a Jacobs laser file, same artboard and convention as the deck |

## The power board adaptor
Only needed if your power board's holes do not meet the deck's. The V3.1 deck does not take the V10 (Ambimat) power board directly. Its three 3.3 mm
power-board holes, at (-101, 47), (-57, 47) and (-57, -47), are a 44 x 94 mm pattern; the board's
own holes are 66 x 42 mm. The official adaptor bridges them: a 110 x 60 mm plate, 3.13 mm thick,
corner radius 5, with the 94 x 44 pattern (outer four holes) and the 66 x 42 pattern (inner four),
all 3.0 mm. Cut it from 3 mm (1/8 in) acrylic or print the STL flat. Only three of the outer
holes meet deck holes; that is by design ("only three of the power board mounting holes are
used" in the guide).

## The S2 holes
Four 3.4 mm clearance holes (M3) on the S2's 41.7 x 41.7 mm pattern, centred on the deck's lidar
position (97, 0), the square turned 30 degrees so the holes clear the Hokuyo UST-10LX holes
(square to the deck they would overlap them by 1.2 mm). The Hokuyo UST and UTM patterns are kept
for a later lidar. The S2 body therefore sits turned 30 degrees; its cable exits 30 degrees off the
rear axis, and the lidar's yaw offset in the car's TF is 30 degrees.

Screws: 4 x M3 x 10 mm. The S2's holes are blind, 4 mm deep: through a 0.25 in (6.35 mm) deck a
10 mm screw engages 3.6 mm; do not use longer. The S2's scan plane is about 30 mm above the deck.

## Material and cutting (Jacobs Hall, Berkeley)
- 0.25 in cast acrylic, as the original drawing specifies (Jacobs store stock is laser safe).
  The parts sheet's Jetson screw length assumes a 0.12 in deck; at 0.25 in use M2.5 x 10 mm.
- Maker Pass laser training (module, video, quiz), reserve at reserve.jacobshall.org, buddy
  present, never leave the laser running. Acrylic is the flammable one: watch it.
- Print the PDF from Illustrator or Acrobat to the laser; in the Universal Control Panel pick
  the Jacobs store acrylic in the material database and enter the measured thickness. Test-cut
  a small circle in a corner first.
- The VLS 6.60 bed (room 110C) is 32 x 18 in; the PDF matches it. The part is 310 x 145 mm.
