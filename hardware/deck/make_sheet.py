"""Build the 32x18 in laser sheet: deck A (S2 only, square), deck B (same + wider standoff
slots, notched), centre cut line, small useful shapes in the left half's leftover space; the
right half is left blank. Outputs PDF + DXFs."""
import re, math, sys
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch, mm

SRC = sys.argv[1]; OUT = sys.argv[2]
IN = 25.4
SHEET_W, SHEET_H = 32*IN, 18*IN

# ---------- read the S2 deck DXF ----------
s = open(SRC, errors='ignore').read()
ent = s.split('\nENTITIES\n')[1].split('\nENDSEC\n')[0]
parts = re.split(r'\n  0\n', '\n' + ent)
def g(p, code):
    m = re.search(r'\n *%s\n([-\d.eE+]+)' % code, p); return float(m.group(1)) if m else None
base = []   # ('L',x1,y1,x2,y2) ('C',x,y,r) ('A',x,y,r,a0,a1)
for p in parts:
    t = p.split('\n')[0].strip()
    if t == 'LINE': base.append(('L', g(p,'10'), g(p,'20'), g(p,'11'), g(p,'21')))
    elif t == 'CIRCLE': base.append(('C', g(p,'10'), g(p,'20'), g(p,'40')))
    elif t == 'ARC': base.append(('A', g(p,'10'), g(p,'20'), g(p,'40'), g(p,'50'), g(p,'51')))

# ---------- deck A: drop Hokuyo (8 x 3.4) and rotated S2 holes, add square S2 holes ----------
HOKUYO = {(70.5,26.5),(70.5,-26.5),(77.0,20.0),(77.0,-20.0),(117.0,20.0),(117.0,-20.0),(123.5,26.5),(123.5,-26.5)}
S2_OLD = {(68.5,7.6),(89.4,-28.5),(104.6,28.5),(125.5,-7.6)}
def is_hole(it, setp):
    return it[0]=='C' and (round(it[1],1), round(it[2],1)) in setp
deckA = [it for it in base if not is_hole(it, HOKUYO|S2_OLD)]
removed = len(base) - len(deckA); assert removed == 12, removed
S2C, S2H = 97.0, 41.7/2
for sx in (-1,1):
    for sy in (-1,1):
        deckA.append(('C', S2C+sx*S2H, sy*S2H, 1.7))   # 3.4 mm, M3 clearance

# ---------- deck B: standoff holes 127 mm apart (measured), each a 3 mm slot along the rail ----------
EDGE_Y = 66.5
edge = [(it[1], it[2]) for it in deckA if it[0]=='C' and abs(abs(it[2])-EDGE_Y) < 0.1]
assert len(edge) == 8, edge
outer = sorted(set(round(x,2) for x,_ in edge)); span = outer[-1]-outer[0]; cx = (outer[-1]+outer[0])/2
scale = 127.0/span
def slot(x, y, r, half):
    """stadium slot of width 2r centred on (x,y), running along x, +-half long."""
    return [('L', x-half, y+r, x+half, y+r), ('L', x-half, y-r, x+half, y-r),
            ('A', x+half, y, r, 270, 90), ('A', x-half, y, r, 90, 270)]
deckB = [it for it in deckA if not (it[0]=='C' and abs(abs(it[2])-EDGE_Y) < 0.1)]
for x, y in edge:
    xn = cx + (x-cx)*scale
    deckB += slot(xn, y, 1.65, 1.5)   # the cable slot edge is at y=63.5: no room to go inboard
# label notch on B: 4 mm half-circle out of the rear (x=-155) edge at y=0: replace the rear edge line
rear = [it for it in deckB if it[0]=='L' and abs(it[1]+155)<0.01 and abs(it[3]+155)<0.01]
assert len(rear)==1, rear
r = rear[0]; deckB.remove(r)
ylo, yhi = sorted([r[2], r[4]])
deckB += [('L',-155,ylo,-155,-4), ('L',-155,4,-155,yhi), ('A',-155,0,4,270,90)]

# ---------- sheet layout (mm, origin bottom-left of sheet) ----------
def place(items, ox, oy):
    out=[]
    for it in items:
        if it[0]=='L': out.append(('L', it[1]+ox, it[2]+oy, it[3]+ox, it[4]+oy))
        elif it[0]=='C': out.append(('C', it[1]+ox, it[2]+oy, it[3]))
        else: out.append(('A', it[1]+ox, it[2]+oy, it[3], it[4], it[5]))
    return out
M = 20.0
sheet = []
# deck A top-left, deck B below it (deck extents x -155..155, y -72.5..72.5)
A_ox, A_oy = M+155, SHEET_H-M-72.5
B_ox, B_oy = M+155, SHEET_H-M-145-M-72.5
sheet += place(deckA, A_ox, A_oy)
sheet += place(deckB, B_ox, B_oy)
# label triangle beside deck B (a loose 12 mm triangle, 'B' marker)
tx, ty = B_ox+155+12, B_oy
sheet += [('L',tx,ty-6,tx+12,ty-6),('L',tx+12,ty-6,tx+6,ty+6),('L',tx+6,ty+6,tx,ty-6)]
# centre cut line, full height
CX = SHEET_W/2
sheet.append(('L', CX, 0, CX, SHEET_H))

# ---------- left half leftovers only; the right half stays blank for later ----------
def hexagon(cx, cy, af):   # across-flats
    R = af/math.sqrt(3); pts=[(cx+R*math.cos(math.radians(60*k+30)), cy+R*math.sin(math.radians(60*k+30))) for k in range(6)]
    return [('L',pts[k][0],pts[k][1],pts[(k+1)%6][0],pts[(k+1)%6][1]) for k in range(6)]
def poly(pts, ox=0, oy=0):
    return [('L',pts[k][0]+ox,pts[k][1]+oy,pts[(k+1)%len(pts)][0]+ox,pts[(k+1)%len(pts)][1]+oy) for k in range(len(pts))]
def rrect(x0,y0,w,h,r):
    return [('L',x0+r,y0,x0+w-r,y0),('L',x0+w,y0+r,x0+w,y0+h-r),('L',x0+w-r,y0+h,x0+r,y0+h),('L',x0,y0+h-r,x0,y0+r),
            ('A',x0+w-r,y0+r,r,270,360),('A',x0+w-r,y0+h-r,r,0,90),('A',x0+r,y0+h-r,r,90,180),('A',x0+r,y0+r,r,180,270)]
def hole(x,y,d): return [('C',x,y,d/2)]

# letters as cut outlines from DejaVu Sans Bold (TrueType, flattened)
from fontTools.ttLib import TTFont
from fontTools.pens.recordingPen import RecordingPen
_font = TTFont('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')
_gs = _font.getGlyphSet(); _upm = _font['head'].unitsPerEm
def glyph(ch, cap_mm, ox, oy):
    """outline of one capital letter, cap height cap_mm, lower-left of its box at (ox,oy)."""
    name = _font.getBestCmap()[ord(ch)]
    pen = RecordingPen(); _gs[name].draw(pen)
    k = cap_mm / (0.729 * _upm)          # DejaVu Sans Bold cap height = 0.729 em
    xmin = min(p[0] for op,args in pen.value for p in args if args) if False else None
    out=[]; cur=None; start=None
    def P(pt): return (ox + pt[0]*k, oy + pt[1]*k)
    for op, args in pen.value:
        if op == 'moveTo': cur = start = P(args[0])
        elif op == 'lineTo': n = P(args[0]); out.append(('L',*cur,*n)); cur = n
        elif op == 'qCurveTo':
            pts = [P(a) for a in args]
            # TrueType: implied on-curve points between consecutive off-curve points
            offs = pts[:-1]; endp = pts[-1]
            ctrls=[]
            for i, c in enumerate(offs):
                if i+1 < len(offs):
                    mid = ((c[0]+offs[i+1][0])/2, (c[1]+offs[i+1][1])/2); ctrls.append((c, mid))
                else: ctrls.append((c, endp))
            for c, e in ctrls:
                for t in [j/8 for j in range(1,9)]:
                    n = ((1-t)**2*cur[0]+2*(1-t)*t*c[0]+t*t*e[0], (1-t)**2*cur[1]+2*(1-t)*t*c[1]+t*t*e[1])
                    out.append(('L',*cur,*n)); cur = n
        elif op in ('closePath','endPath'):
            if cur != start: out.append(('L',*cur,*start))
    return out
def glyph_width(ch, cap_mm):
    name = _font.getBestCmap()[ord(ch)]; return _font['hmtx'][name][0] * cap_mm/(0.729*_upm)

# -- bottom strip under deck B (x 20..386, y 10..117) --
# 1. nose plate for the car: bolts to the deck's two 4.5 mm nose holes (34.2 mm apart), FEB cut out
px, py = 20, 70
sheet += rrect(px, py, 60, 36, 4)
sheet += hole(px+30-17.1, py+27, 4.5) + hole(px+30+17.1, py+27, 4.5)
cap = 11; w = sum(glyph_width(c, cap) for c in 'FEB'); x = px+30-w/2
for ch in 'FEB':
    sheet += glyph(ch, cap, x, py+6); x += glyph_width(ch, cap)
# 2. six cable saddles: tab under any M3 head, zip tie through the two slots
for col in range(2):
    for row in range(3):
        x0, y0 = 20+col*35, 10+row*18
        sheet += rrect(x0, y0, 30, 12, 2) + hole(x0+6, y0+6, 3.3)
        for sx in (16, 23): sheet += rrect(x0+sx, y0+2.5, 2.5, 7, 1.2)
# 3. one hex coaster
AF=90; R=AF/math.sqrt(3); sheet += hexagon(92+R, 10+R, AF)
# 4. three rally-car keyrings, 70 x 26, side view with wheel arches
def car(ox, oy):
    out = []
    body = [(0,9),(1,6),(4,5),(8,5)]
    out += [('L',body[i][0]+ox,body[i][1]+oy,body[i+1][0]+ox,body[i+1][1]+oy) for i in range(len(body)-1)]
    out.append(('A',ox+14,oy+5,6,0,180))                       # front arch (from (20,5) round to (8,5))
    out.append(('L',ox+20,oy+5,ox+50,oy+5))
    out.append(('A',ox+56,oy+5,6,0,180))                       # rear arch
    tail = [(62,5),(66,6),(69,9),(70,13),(68,15),(60,16),(52,16),(44,24),(40,26),(26,26),(18,24),(10,17),(3,15),(0,13),(0,9)]
    out += [('L',tail[i][0]+ox,tail[i][1]+oy,tail[i+1][0]+ox,tail[i+1][1]+oy) for i in range(len(tail)-1)]
    out += hole(ox+33, oy+21, 4)                               # keyring
    out += poly([(20,17),(27,23),(39,23),(43,17)], ox, oy)     # window
    return out
for row in range(3): sheet += car(192, 10+row*34)
# 5. nine cone tags (FSAE cones), 20 x 28, 3 mm hole
cone = [(0,0),(20,0),(20,4),(16,4),(13,28),(7,28),(4,4),(0,4)]
for col in range(3):
    for row in range(3):
        ox, oy = 272+col*26, 10+row*38
        sheet += poly(cone, ox, oy) + hole(ox+10, oy+20, 3)
# 6. M3 spacer rings, 12 mm OD, 3.3 mm hole: six here, six in the column above
for k in range(6): sheet += hole(365, 16+k*18, 12) + hole(365, 16+k*18, 3.3)
# -- column right of the deck noses (x 342..396, y 130..437) --
for k in range(6): sheet += hole(378, 140+k*18, 12) + hole(378, 140+k*18, 3.3)
# big FEB letters, 36 mm tall, as loose pieces
for i, ch in enumerate('BEF'):
    w = glyph_width(ch, 36); sheet += glyph(ch, 36, 369-w/2, 250+i*48)

# ---------- bounds check ----------
def bbox(items):
    xs=[];ys=[]
    for it in items:
        if it[0]=='L': xs+=[it[1],it[3]]; ys+=[it[2],it[4]]
        elif it[0]=='C': xs+=[it[1]-it[3],it[1]+it[3]]; ys+=[it[2]-it[3],it[2]+it[3]]
        else:
            a0,a1=it[4],it[5]
            if a1<a0: a1+=360
            for k in range(0,101):
                a=math.radians(a0+(a1-a0)*k/100); xs.append(it[1]+it[3]*math.cos(a)); ys.append(it[2]+it[3]*math.sin(a))
    return min(xs),max(xs),min(ys),max(ys)
bb=bbox(sheet); assert bb[0]>=0 and bb[1]<=SHEET_W and bb[2]>=0 and bb[3]<=SHEET_H, bb

# ---------- PDF ----------
c = canvas.Canvas(OUT+'.pdf', pagesize=(SHEET_W*mm, SHEET_H*mm))
c.setStrokeColorRGB(1,0,0); c.setLineWidth(0.001)
for it in sheet:
    if it[0]=='L': c.line(it[1]*mm,it[2]*mm,it[3]*mm,it[4]*mm)
    elif it[0]=='C': c.circle(it[1]*mm,it[2]*mm,it[3]*mm)
    else:
        a0,a1=it[4],it[5]
        if a1<a0: a1+=360
        c.arc((it[1]-it[3])*mm,(it[2]-it[3])*mm,(it[1]+it[3])*mm,(it[2]+it[3])*mm,a0,a1-a0)
c.save()

# ---------- DXF (R12, mm) ----------
def dxf(items, path):
    o=['0','SECTION','2','HEADER','9','$INSUNITS','70','4','0','ENDSEC','0','SECTION','2','ENTITIES']
    for it in items:
        if it[0]=='L': o+=['0','LINE','8','CUT','10',f'{it[1]:.4f}','20',f'{it[2]:.4f}','30','0','11',f'{it[3]:.4f}','21',f'{it[4]:.4f}','31','0']
        elif it[0]=='C': o+=['0','CIRCLE','8','CUT','10',f'{it[1]:.4f}','20',f'{it[2]:.4f}','30','0','40',f'{it[3]:.4f}']
        else: o+=['0','ARC','8','CUT','10',f'{it[1]:.4f}','20',f'{it[2]:.4f}','30','0','40',f'{it[3]:.4f}','50',f'{it[4]:.4f}','51',f'{it[5]:.4f}']
    o+=['0','ENDSEC','0','EOF']
    open(path,'w').write('\n'.join(o)+'\n')
dxf(deckA, OUT+'_deckA.dxf'); dxf(deckB, OUT+'_deckB.dxf'); dxf(sheet, OUT+'_sheet.dxf')
print('deck A entities', len(deckA), 'deck B', len(deckB), 'sheet', len(sheet))
print('edge hole x (A):', outer, 'span %.2f'%span, '-> B span 127.00, scale %.4f'%scale)
print('S2 holes:', [(round(S2C+sx*S2H,2), round(sy*S2H,2)) for sx in (-1,1) for sy in (-1,1)])
print('sheet bbox', [round(v,1) for v in bb])
