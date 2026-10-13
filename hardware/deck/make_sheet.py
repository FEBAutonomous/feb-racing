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
def comb(x0,y0,w,h,n,sw,sd):
    """cable comb: rect with rounded bottom corners and n slots of width sw, depth sd, open at the top."""
    pitch = w/(n+1); r=4; out=[]
    out += [('L',x0+w,y0+r,x0+w,y0+h),('L',x0,y0+h,x0,y0+r),('L',x0+r,y0,x0+w-r,y0),('A',x0+w-r,y0+r,r,270,360),('A',x0+r,y0+r,r,180,270)]
    edge_x=x0
    for k in range(n):
        xc=x0+pitch*(k+1)
        out += [('L',edge_x,y0+h,xc-sw/2,y0+h),('L',xc-sw/2,y0+h,xc-sw/2,y0+h-sd+sw/2),('A',xc,y0+h-sd+sw/2,sw/2,180,360),('L',xc+sw/2,y0+h-sd+sw/2,xc+sw/2,y0+h)]
        edge_x=xc+sw/2
    out.append(('L',edge_x,y0+h,x0+w,y0+h))
    return out
# bottom strip under deck B (y 10..117, x 20..386): three 90 mm hex coasters
AF=90; R=AF/math.sqrt(3)
for col in range(3):
    sheet += hexagon(M+R+col*(AF+10), 10+R, AF)
# beside them: two small cable combs, 64 x 40, three 7 mm slots 22 deep
sheet += comb(322, 12, 64, 40, 3, 7, 22)
sheet += comb(322, 62, 64, 40, 3, 7, 22)
# column right of the deck noses (x 372..384): sixteen M3 spacer rings, 12 mm OD, 3.3 mm hole
for k in range(16):
    cy = 140 + k*18
    sheet += [('C', 378, cy, 6.0), ('C', 378, cy, 1.65)]

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
