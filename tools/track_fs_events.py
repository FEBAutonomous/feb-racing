#!/usr/bin/env python3
"""The two Formula Student fixed-layout events as tracks, to the 2026 rules, in cones and in walls:

    tools/track_fs_events.py            writes tracks/{skidpad,skidpad_walls,acceleration,acceleration_walls}

Skidpad (FS Rules 2026 D 4): two pairs of concentric circles, centres 18.25 m apart, inner
circles 15.25 m and outer circles 21.25 m in diameter, a 3 m path between them, entry and exit
on a 3 m lane tangent to the circles where they meet. 17 cones inside each inner circle, 13
around the outside of each outer circle. The run: enter, one lap right to settle, one lap
right timed, one lap left, one lap left timed, exit. The car is staged 15 m before the line
(D 4.2.4) and must stop within 25 m after it (D 4.2.5). Acceleration (D 5): a 75 m straight at
least 3 m wide, cones every 5 m, the car staged 0.3 m behind the line, stop within 75 m after
the finish (D 5.2.4). Cone colours follow the driverless convention, blue on the left of the
driving direction and yellow on the right; orange marks the lanes and the lines.

The centreline is the whole programme, so the house driver can drive it and a driver can be
checked against it. `timing` holds the lines the harness times from the car's true position
(the simulator's lap counter counts loops, these are not loops). The walls versions use the
same geometry with air-duct walls where the cones stand; the figure of eight's crossing is
left open, as it must be.
"""
import json
import math
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
SPACING = 0.1            # m between centreline points, like every other track
LANE = 1.5               # half of the 3 m path
WALL_D = 0.33

# ---------------------------------------------------------------- geometry helpers


def arc(cx, cy, r, a0, a1, n=None):
    """Points on a circle from angle a0 to a1 (radians, either direction), SPACING apart."""
    n = n or max(8, int(abs(a1 - a0) * r / SPACING))
    a = np.linspace(a0, a1, n, endpoint=False)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])


def line(p, q):
    p, q = np.asarray(p, float), np.asarray(q, float)
    n = max(2, int(np.hypot(*(q - p)) / SPACING))
    t = np.linspace(0, 1, n, endpoint=False)[:, None]
    return p + t * (q - p)


def cones_on(cx, cy, r, phis, colour):
    return [{"x": round(cx + r * math.cos(f), 3), "y": round(cy + r * math.sin(f), 3), "color": colour} for f in phis]


def sweep(path, half):
    """Left and right edges of a path swept sideways: the asphalt the app paints and the site draws."""
    p = np.asarray(path)
    d = np.gradient(p, axis=0)
    d /= np.maximum(np.linalg.norm(d, axis=1), 1e-9)[:, None]
    n = np.column_stack([-d[:, 1], d[:, 0]])
    return p + half * n, p - half * n


def thin_loop(points, offset=0.03):
    """An open polyline as a closed one the app can sweep a tube along: out along the line and
    back along a copy shifted sideways by less than the tube's radius, so it reads as one wall."""
    p = np.asarray(points)
    d = np.gradient(p, axis=0)
    d /= np.maximum(np.linalg.norm(d, axis=1), 1e-9)[:, None]
    n = np.column_stack([-d[:, 1], d[:, 0]])
    return np.vstack([p, (p + offset * n)[::-1]])


def flat(a):
    return [round(float(v), 3) for v in np.asarray(a).reshape(-1)]


def write(name, track, preview):
    folder = ROOT / "tracks" / name
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "track.json").write_text(json.dumps(track, separators=(",", ":")) + "\n")
    preview.save(folder / "preview.png")
    print(f"{name}: {track['length']} m, {len(track['cones'])} cones, {len(track['walls'])} walls -> {folder}")


# ---------------------------------------------------------------- preview

def draw(track, path, scale=10, rotate=False):
    from PIL import Image, ImageDraw
    pts = np.asarray(path)
    extra = np.array([[c["x"], c["y"]] for c in track["cones"]] + [p for w in track["walls"] for p in np.asarray(w["points"]).reshape(-1, 2)] or [[0, 0]])
    allp = np.vstack([pts, extra])
    lo, hi = allp.min(0) - 3, allp.max(0) + 3
    W, H = int((hi[0] - lo[0]) * scale), int((hi[1] - lo[1]) * scale)
    img = Image.new("RGB", (W, H), (40, 40, 40))
    d = ImageDraw.Draw(img)
    px = lambda xy: (int((xy[0] - lo[0]) * scale), int(H - (xy[1] - lo[1]) * scale))
    d.line([px(p) for p in pts], fill=(235, 235, 235), width=int(2 * LANE * scale), joint="curve")   # the asphalt, the path 3 m wide
    d.line([px(p) for p in pts], fill=(253, 181, 21), width=1)
    for w in track["walls"]:
        d.line([px(p) for p in np.asarray(w["points"]).reshape(-1, 2)] + [px(np.asarray(w["points"]).reshape(-1, 2)[0])], fill=(28, 70, 135), width=max(2, int(WALL_D * scale)))
    for cp in track["checkpoints"]:
        n = np.array([-math.sin(cp["yaw"]), math.cos(cp["yaw"])]) * cp["width"] / 2
        d.line([px((cp["x"] - n[0], cp["y"] - n[1])), px((cp["x"] + n[0], cp["y"] + n[1]))], fill=(255, 255, 255), width=2 if cp is track["checkpoints"][0] else 1)
    col = {"blue": (28, 70, 135), "yellow": (253, 181, 21), "orange": (255, 90, 0)}
    for c in track["cones"]:
        r = 4 if c["color"] == "orange" else 3
        x, y = px((c["x"], c["y"]))
        d.ellipse([x - r, y - r, x + r, y + r], fill=col[c["color"]])
    s = track["spawn"]
    x0, y0 = px((s["x"], s["y"])); x1, y1 = px((s["x"] + 0.8 * math.cos(s["yaw"]), s["y"] + 0.8 * math.sin(s["yaw"])))
    d.line([(x0, y0), (x1, y1)], fill=(0, 200, 0), width=3)
    if rotate:
        img = img.rotate(90, expand=True)
    return img


# ---------------------------------------------------------------- skidpad

def skidpad(walls=False):
    R, r_in, r_out, c = 9.125, 7.625, 10.625, 9.125       # path radius, inner, outer, centre offset
    entry_y, exit_y, stop_y = -15.3, 8.0, 33.0           # staging 15 m before the line, finish line, end of the stop zone
    path = [line((0, entry_y), (0, 0))]
    for lap in range(2):                                 # right circle, clockwise, from the crossing heading +y
        path.append(arc(c, 0, R, math.pi, -math.pi))
    for lap in range(2):                                 # left circle, counter-clockwise
        path.append(arc(-c, 0, R, 0, 2 * math.pi))
    path.append(line((0, 0), (0, stop_y + 2)))
    path = np.vstack(path)
    length = round(len(path) * SPACING, 2)

    cones, wall_list = [], []
    # the outer cones span the arc clear of the other circle and the lane: |x| >= LANE
    # the outer cones span the arc clear of the lane: on the right circle the lane edge x = +LANE
    # is at angle +-acos((LANE - c) / r_out) from +x, so the free arc is the rest
    span = 2 * math.acos((LANE - c) / r_out)
    if not walls:
        cones += cones_on(c, 0, r_in, np.linspace(0, 2 * math.pi, 17, endpoint=False), "yellow")      # right circle: inside is on the right
        cones += cones_on(-c, 0, r_in, np.linspace(0, 2 * math.pi, 17, endpoint=False), "blue")       # left circle: inside is on the left
        half = span / 2
        cones += cones_on(c, 0, r_out, np.linspace(-half, half, 13), "blue")                            # right circle outside, away from the crossing
        cones += cones_on(-c, 0, r_out, math.pi + np.linspace(-half, half, 13), "yellow")
        lane_open = math.sqrt(r_out ** 2 - (c - LANE) ** 2)                                             # |y| where the lane meets the outer circles
        for y in np.arange(entry_y + 0.3, -lane_open, 2.5):
            cones += [{"x": -LANE, "y": round(y, 3), "color": "orange"}, {"x": LANE, "y": round(y, 3), "color": "orange"}]
        for y in np.arange(lane_open + 1.0, stop_y + 0.1, 5.0):
            cones += [{"x": -LANE, "y": round(y, 3), "color": "orange"}, {"x": LANE, "y": round(y, 3), "color": "orange"}]
        for sx in (-1, 1):
            for sy in (-1, 1):
                cones.append({"x": sx * 1.9, "y": sy * 0.6, "color": "orange"})                          # the start/finish line's big cones
    else:
        half = span / 2
        lane_open = math.sqrt(r_out ** 2 - (c - LANE) ** 2)
        wall_list.append(arc(c, 0, r_in, 0, 2 * math.pi, 96))
        wall_list.append(arc(-c, 0, r_in, 0, 2 * math.pi, 96))
        wall_list.append(thin_loop(arc(c, 0, r_out, -half, half, 120)))
        wall_list.append(thin_loop(arc(-c, 0, r_out, math.pi - half, math.pi + half, 120)))
        for sx in (-1, 1):
            wall_list.append(thin_loop(line((sx * LANE, entry_y - 1), (sx * LANE, -lane_open))))
            wall_list.append(thin_loop(line((sx * LANE, lane_open), (sx * LANE, stop_y))))

    track = {
        "version": 1, "name": "FS Skidpad" + (" (walls)" if walls else ""), "direction": "cw", "max_cars": 1,
        "qualifying": False, "category": "feb" if walls else "fsae", "difficulty": 3, "length": length,
        "event": "skidpad",
        "centreline": flat(path),
        "checkpoints": [{"x": 0.0, "y": exit_y, "yaw": round(math.pi / 2, 4), "width": 3.0},
                        {"x": 0.0, "y": -8.0, "yaw": round(math.pi / 2, 4), "width": 3.0}],
        "spawn": {"x": 0.0, "y": round(entry_y - 0.3, 3), "yaw": round(math.pi / 2, 4)},
        "walls": [{"points": flat(w)} for w in wall_list], "cones": cones,
        "timing": {"event": "skidpad",
                   "lines": {"centre": [[-1.6, 0.0], [1.6, 0.0]], "exit": [[-1.6, exit_y], [1.6, exit_y]],
                             "stop_end": [[-1.6, stop_y], [1.6, stop_y]]},
                   "circle_x": c},
    }
    if walls:
        track["wall_diameter"], track["wall_color"] = WALL_D, "#9a9a9a"
    left, right = sweep(path, LANE)
    track["paint_left"], track["paint_right"] = flat(left), flat(right)
    return track, draw(track, path)


# ---------------------------------------------------------------- acceleration

def acceleration(walls=False):
    finish_y, stop_y = 75.0, 150.0
    path = line((0, -2.0), (0, stop_y + 2.0))
    length = round(len(path) * SPACING, 2)
    cones, wall_list = [], []
    if not walls:
        for y in np.arange(0.0, finish_y + 0.1, 5.0):
            cones += [{"x": -LANE, "y": round(y, 3), "color": "blue"}, {"x": LANE, "y": round(y, 3), "color": "yellow"}]
        for y in np.arange(finish_y + 10.0, stop_y + 0.1, 10.0):
            cones += [{"x": -LANE, "y": round(y, 3), "color": "orange"}, {"x": LANE, "y": round(y, 3), "color": "orange"}]
        for y in (-0.8, finish_y + 0.8):
            cones += [{"x": -1.9, "y": y, "color": "orange"}, {"x": 1.9, "y": y, "color": "orange"}]
    else:
        for sx in (-1, 1):
            wall_list.append(thin_loop(line((sx * LANE, -3.0), (sx * LANE, stop_y + 2.0))))
    track = {
        "version": 1, "name": "FS Acceleration" + (" (walls)" if walls else ""), "direction": "cw", "max_cars": 1,
        "qualifying": False, "category": "feb" if walls else "fsae", "difficulty": 1, "length": length,
        "event": "acceleration",
        "centreline": flat(path),
        "checkpoints": [{"x": 0.0, "y": finish_y, "yaw": round(math.pi / 2, 4), "width": 3.0},
                        {"x": 0.0, "y": 40.0, "yaw": round(math.pi / 2, 4), "width": 3.0}],
        "spawn": {"x": 0.0, "y": -0.55, "yaw": round(math.pi / 2, 4)},       # the nose 0.3 m behind the line
        "walls": [{"points": flat(w)} for w in wall_list], "cones": cones,
        "timing": {"event": "acceleration",
                   "lines": {"start": [[-1.6, 0.0], [1.6, 0.0]], "finish": [[-1.6, finish_y], [1.6, finish_y]],
                             "stop_end": [[-1.6, stop_y], [1.6, stop_y]]}},
    }
    if walls:
        track["wall_diameter"], track["wall_color"] = WALL_D, "#9a9a9a"
    left, right = sweep(path, LANE)
    track["paint_left"], track["paint_right"] = flat(left), flat(right)
    return track, draw(track, path, rotate=True)


if __name__ == "__main__":
    for name, (track, preview) in (("skidpad", skidpad()), ("skidpad_walls", skidpad(walls=True)),
                                   ("acceleration", acceleration()), ("acceleration_walls", acceleration(walls=True))):
        write(name, track, preview)
