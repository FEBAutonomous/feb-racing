"""Build the 32x18 in laser sheet: deck A (S2 only, square), deck B (same + wider standoff
slots, notched), centre cut line, small useful shapes in the left half's leftover space; the
right half is left blank. Outputs PDF + DXFs."""
import re, math, sys
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch, mm

SRC = sys.argv[1]; OUT = sys.argv[2]
IN = 25.4
SHEET_W, SHEET_H = 32*IN, 16*IN
CX = 480.0   # centre cut; the blank right half is 332.8 mm (13.1 in) wide

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
# label triangle beside deck B's nose (a loose 12 mm triangle, 'B' marker)
tx, ty = B_ox+155+4, B_oy-25
sheet += [('L',tx,ty-6,tx+12,ty-6),('L',tx+12,ty-6,tx+6,ty+6),('L',tx+6,ty+6,tx,ty-6)]
# centre cut line, full height
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

def glyph_paths(ch, cap_mm, ox, oy):
    """closed contours (lists of points) of one letter, for filling (raster engrave)."""
    name = _font.getBestCmap()[ord(ch)]
    pen = RecordingPen(); _gs[name].draw(pen)
    k = cap_mm / (0.729 * _upm)
    P = lambda pt: (ox + pt[0]*k, oy + pt[1]*k)
    paths=[]; cur=None; pts=[]
    for op, args in pen.value:
        if op == 'moveTo': pts=[P(args[0])]; cur=pts[0]
        elif op == 'lineTo': cur=P(args[0]); pts.append(cur)
        elif op == 'qCurveTo':
            qs=[P(a) for a in args]; offs=qs[:-1]; endp=qs[-1]
            for i,c in enumerate(offs):
                e = ((c[0]+offs[i+1][0])/2,(c[1]+offs[i+1][1])/2) if i+1<len(offs) else endp
                for t in [j/8 for j in range(1,9)]:
                    n=((1-t)**2*cur[0]+2*(1-t)*t*c[0]+t*t*e[0],(1-t)**2*cur[1]+2*(1-t)*t*c[1]+t*t*e[1]); pts.append(n)
                cur=e
        elif op in ('closePath','endPath'):
            if len(pts)>2: paths.append(pts)
            pts=[]
    return paths

# FEB logo: two parallelograms from assets/feb_logo.svg (769 x 579 box), each shrunk 6 % about
# its own centre so a web of acrylic stays between them.
_BLUE=[(360,0),(0,360),(314,360),(674,0)]; _YEL=[(455,219),(95,579),(409,579),(769,219)]
def logo(cx, cy, width, shrink=0.94):
    k = width/769.0; out=[]
    for quad in (_BLUE,_YEL):
        qx=sum(q[0] for q in quad)/4; qy=sum(q[1] for q in quad)/4
        pts=[(cx+(qx+(x-qx)*shrink-769/2)*k, cy-(qy+(y-qy)*shrink-579/2)*k) for x,y in quad]
        out.append(pts)
    return out

# -- bottom strip under deck B (x 20..386, y 10..117) --
# -- bottom strip under deck B (x 20..330, y 10..66) --
# 1. nine FSAE cone keychains, 20 x 28: outline and 3 mm ring hole cut, the cone's stripe engraved
cone = [(0,0),(20,0),(20,4),(16,4),(13,28),(7,28),(4,4),(0,4)]
stripe = [(5.0,12),(15.0,12),(14.375,17),(5.625,17)]      # band between the cone's sloping edges
for k in range(9):
    ox, oy = 20+k*23, 12
    sheet += poly(cone, ox, oy) + hole(ox+10, oy+23, 3)
    sheet.append(('F', [[(x+ox, y+oy) for x, y in stripe]]))
# 2. fifteen M3 spacer rings, 12 mm OD, 3.3 mm hole
for row in range(3):
    for col in range(5):
        sheet += hole(240+col*18, 18+row*18, 12) + hole(240+col*18, 18+row*18, 3.3)
# -- area right of the decks (x 345..470, y 10..396) --
# 3. nine cone keychains with the FEB logo on a taller base, 24 x 30
tall = [(0,0),(24,0),(24,7),(18,7),(14.5,30),(9.5,30),(6,7),(0,7)]
tstripe = [(7.4,16),(16.6,16),(15.9,20.5),(8.1,20.5)]
for col in range(3):
    for row in range(3):
        ox, oy = 345+col*28, 12+row*36
        sheet += poly(tall, ox, oy) + hole(ox+12, oy+25, 3)
        sheet.append(('F', [[(x+ox, y+oy) for x, y in tstripe]]))
        sheet.append(('F', logo(ox+12, oy+3.6, 8)))
# 4. ten team plaques, 50 x 40: outline and hanging hole cut; logo and FEBAUTO raster-engraved
def plaque(x0, y0):
    out = rrect(x0, y0, 50, 40, 3) + hole(x0+5, y0+35, 3.3)
    out.append(('F', logo(x0+25, y0+26, 26)))
    cap=4.2; wtxt=sum(glyph_width(c,cap) for c in 'FEBAUTO'); x=x0+25-wtxt/2
    for ch in 'FEBAUTO':
        out.append(('F', glyph_paths(ch, cap, x, y0+6))); x += glyph_width(ch, cap)
    return out
for col in range(2):
    for row in range(5):
        sheet += plaque(345+col*56, 130+row*46)

# ---------- bounds check ----------
def bbox(items):
    xs=[];ys=[]
    for it in items:
        if it[0]=='F':
            for poly_ in it[1]:
                xs+=[q[0] for q in poly_]; ys+=[q[1] for q in poly_]
            continue
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
# the PDF page is the full 32 x 18 in bed so the print driver does not re-centre a smaller page;
# the 32 x 16 sheet sits in the bed's top-left corner, so content is shifted up by the difference
PAGE_H = 18*IN; DY = PAGE_H - SHEET_H
c = canvas.Canvas(OUT+'.pdf', pagesize=(SHEET_W*mm, PAGE_H*mm))
c.setStrokeColorRGB(1,0,0); c.setLineWidth(0.001)
for it in sheet:
    if it[0]=='F':
        c.saveState(); c.setFillColorRGB(0,0,0)
        pth=c.beginPath()
        for poly_ in it[1]:
            pth.moveTo(poly_[0][0]*mm, (poly_[0][1]+DY)*mm)
            for q in poly_[1:]: pth.lineTo(q[0]*mm, (q[1]+DY)*mm)
            pth.close()
        c.drawPath(pth, stroke=0, fill=1, fillMode=1); c.restoreState(); continue
    if it[0]=='L': c.line(it[1]*mm,(it[2]+DY)*mm,it[3]*mm,(it[4]+DY)*mm)
    elif it[0]=='C': c.circle(it[1]*mm,(it[2]+DY)*mm,it[3]*mm)
    else:
        a0,a1=it[4],it[5]
        if a1<a0: a1+=360
        c.arc((it[1]-it[3])*mm,(it[2]-it[3]+DY)*mm,(it[1]+it[3])*mm,(it[2]+it[3]+DY)*mm,a0,a1-a0)
c.save()

# ---------- DXF (R12, mm) ----------
def dxf(items, path):
    o=['0','SECTION','2','HEADER','9','$INSUNITS','70','4','0','ENDSEC','0','SECTION','2','ENTITIES']
    for it in items:
        if it[0]=='F':
            for poly_ in it[1]:
                for i in range(len(poly_)):
                    a,b=poly_[i],poly_[(i+1)%len(poly_)]
                    o+=['0','LINE','8','ENGRAVE','10',f'{a[0]:.4f}','20',f'{a[1]:.4f}','30','0','11',f'{b[0]:.4f}','21',f'{b[1]:.4f}','31','0']
            continue
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
