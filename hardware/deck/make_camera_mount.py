"""The RealSense D435if bumper mount for the 1:10 car, laser cut from the 6 mm deck acrylic.

Three bars (A: holes at exactly 3.000 in, as measured; B: slots for 74 to 77 mm, the slack; C: slotted with the bolts behind the camera, 34 mm deep), each on two spacer stacks over the Traxxas front bumper screws (M4; the two holes are slots so
a 74 to 77 mm spacing both fit), the camera sitting flat on the bar held by its 1/4-20 tripod
socket with a short screw from below (the head sits in the 12 mm gap under the bar). Four discs
make the two 12 mm spacers (two per side); M4 x 30 bolts go bar -> spacers -> bumper plate -> the
bumper's catcher, like the original screws.

Outputs: camera_mount.dxf (part geometry, mm) and camera_mount_laser_32x18in.pdf (a 32 x 18 in
artboard, 1:1, red hairlines, parts 20 mm from the top-left corner, Jacobs lasers)."""
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

BAR_W, BAR_H, BAR_R = 96.0, 25.0, 3.0           # mm; the D435 is 90 x 25 mm in plan, the bar matches its depth
SLOT_SPACING = (74.0, 77.0)                      # bar B: bumper screw centres, min and max to cover
FIXED_SPACING = 76.2                             # bar A: exactly 3.000 in, as measured
M4_CLEAR = 4.5                                   # slot and spacer hole width
TRIPOD = 6.6                                     # 1/4-20 clearance
DISC_D, DISCS = 16.0, 4                          # spacer discs, two per side = 12 mm each side
SMALL_D, SMALLS = 10.0, 4                        # the same spacers slimmer, 10 mm outer, if the 16s crowd the bumper

def slot(x, y, r, half):
    return [('L', x-half, y+r, x+half, y+r), ('L', x-half, y-r, x+half, y-r),
            ('A', x+half, y, r, 270, 90), ('A', x-half, y, r, 90, 270)]
def rrect(x0, y0, w, h, r):
    return [('L',x0+r,y0,x0+w-r,y0),('L',x0+w,y0+r,x0+w,y0+h-r),('L',x0+w-r,y0+h,x0+r,y0+h),('L',x0,y0+h-r,x0,y0+r),
            ('A',x0+w-r,y0+r,r,270,360),('A',x0+w-r,y0+h-r,r,0,90),('A',x0+r,y0+h-r,r,90,180),('A',x0+r,y0+r,r,180,270)]
def hole(x, y, d): return [('C', x, y, d/2)]
def place(items, ox, oy):
    out = []
    for it in items:
        if it[0] == 'L': out.append(('L', it[1]+ox, it[2]+oy, it[3]+ox, it[4]+oy))
        else: out.append((it[0], it[1]+ox, it[2]+oy) + tuple(it[3:]))
    return out

# bar A: round M4 holes at exactly the measured spacing; bar B: slots with the slack. Both: the
# camera's 1/4-20 in the middle. Origin at each bar's bottom-left.
cx, cy = BAR_W/2, BAR_H/2
barA = rrect(0, 0, BAR_W, BAR_H, BAR_R) + hole(cx, cy, TRIPOD)
for s in (-1, 1):
    barA += hole(cx + s*FIXED_SPACING/2, cy, M4_CLEAR)
barB = rrect(0, 0, BAR_W, BAR_H, BAR_R) + hole(cx, cy, TRIPOD)
half_len = (SLOT_SPACING[1] - SLOT_SPACING[0]) / 4.0          # each slot covers half the spread
mid = (SLOT_SPACING[0] + SLOT_SPACING[1]) / 4.0                # slot centre from the bar centre
for s in (-1, 1):
    barB += slot(cx + s*mid, cy, M4_CLEAR/2, half_len)
# bar C: the bumper bolts behind the camera instead of under it. 34 mm deep; the camera sits on the
# front 25 mm with its 1/4-20 at y 12.5, the M4 slots run along y 29.5 so bolt heads and washers
# (9 mm) clear the camera body entirely. Slotted like B.
C_H, C_Y = 34.0, 29.5
barC = rrect(0, 0, BAR_W, C_H, BAR_R) + hole(cx, 12.5, TRIPOD)
for s in (-1, 1):
    barC += slot(cx + s*mid, C_Y, M4_CLEAR/2, half_len)
disc = hole(0, 0, DISC_D) + hole(0, 0, M4_CLEAR)

# sheet layout: bar A top-left, bar B under it, discs in a row under that
parts = place(barA, 0, 0)
parts += place(barB, 0, -(BAR_H + 5.0))
parts += place(barC, 0, -(2*BAR_H + 10.0 + (C_H - BAR_H)))
y0 = -(2*BAR_H + 10.0 + (C_H - BAR_H))                         # bottom of bar C
for i in range(DISCS):
    parts += place(disc, DISC_D/2 + i*(DISC_D + 4.0), y0 - (DISC_D/2 + 6.0))
small = hole(0, 0, SMALL_D) + hole(0, 0, M4_CLEAR)
for i in range(SMALLS):
    parts += place(small, SMALL_D/2 + i*(SMALL_D + 4.0), y0 - (DISC_D + 6.0 + SMALL_D/2 + 6.0))

def dxf(items, path):
    o = ['0','SECTION','2','HEADER','9','$INSUNITS','70','4','0','ENDSEC','0','SECTION','2','ENTITIES']
    for it in items:
        if it[0]=='L': o+=['0','LINE','8','CUT','10',f'{it[1]:.4f}','20',f'{it[2]:.4f}','30','0','11',f'{it[3]:.4f}','21',f'{it[4]:.4f}','31','0']
        elif it[0]=='C': o+=['0','CIRCLE','8','CUT','10',f'{it[1]:.4f}','20',f'{it[2]:.4f}','30','0','40',f'{it[3]:.4f}']
        else: o+=['0','ARC','8','CUT','10',f'{it[1]:.4f}','20',f'{it[2]:.4f}','30','0','40',f'{it[3]:.4f}','50',f'{it[4]:.4f}','51',f'{it[5]:.4f}']
    o += ['0','ENDSEC','0','EOF']
    open(path, 'w').write('\n'.join(o) + '\n')

def pdf(items, path, sheet_w_in=12, sheet_h_in=6, margin=20.0):
    W, H = sheet_w_in*25.4, sheet_h_in*25.4
    ys = [v for it in items for v in ((it[2], it[4]) if it[0]=='L' else (it[2]+it[3], it[2]-it[3]))]
    top = max(ys)
    ox, oy = margin, H - margin - top                       # parts 20 mm from the top-left corner
    c = canvas.Canvas(path, pagesize=(W*mm, H*mm))
    c.setStrokeColorRGB(1, 0, 0); c.setLineWidth(0.001)
    for it in place(items, ox, oy):
        if it[0]=='L': c.line(it[1]*mm, it[2]*mm, it[3]*mm, it[4]*mm)
        elif it[0]=='C': c.circle(it[1]*mm, it[2]*mm, it[3]*mm, stroke=1, fill=0)
        else:
            x, y, r, a0, a1 = it[1:]
            c.arc((x-r)*mm, (y-r)*mm, (x+r)*mm, (y+r)*mm, a0, (a1-a0) % 360 or 360)
    c.showPage(); c.save()

dxf(parts, 'camera_mount.dxf')
pdf(parts, 'camera_mount_laser_32x18in.pdf', sheet_w_in=32, sheet_h_in=18)
print("bars %.0f x %.0f mm: A holes %.1f mm apart (%.3f in), B slots covering %.1f to %.1f mm; tripod hole %.1f; %d discs %.0f mm" %
      (BAR_W, BAR_H, FIXED_SPACING, FIXED_SPACING/25.4, 2*(mid-half_len), 2*(mid+half_len), TRIPOD, DISCS, DISC_D))
