"""Build the static leaderboard site: site/_build/{index.html, data.json, tracks/*.png}.

data.json holds everything the page renders: teams, tracks, events, per-track / per-event
standings and the general boards (walls, cones, autocross: mean pace over every track of the kind). Practice standings prefer verified results; self-reported ones are
shown with an "unverified" badge.
"""
import datetime as dt
import json
import pathlib
import shutil

import yaml

from . import results, rules as rules_mod

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "site" / "_build"


def standings(rows):
    """Best result per team: verified beats unverified, then the rank key."""
    best = {}
    for r in sorted(rows, key=lambda r: (not r.get("verified"), rules_mod.rank_key(r))):
        best.setdefault(r["team"], r)
    return sorted(best.values(), key=lambda r: (not r.get("verified"), rules_mod.rank_key(r)))


def h2h_standings(rows):
    """Teams by the furthest round reached, then by position in their last race."""
    last = {}
    for r in sorted(rows, key=lambda r: r.get("race", "")):
        last[r["team"]] = r
    return sorted(last.values(), key=lambda r: (-int(r.get("race", "r0").split("-")[0].lstrip("r") or 0), r.get("position", 9)))


def autocross_standings(rows):
    """Best first lap per team over every attempt on the track, full runs and autocross runs alike."""
    scored = []
    for r in rows:
        total, hits = rules_mod.autocross(r)
        if total is None or r.get("status") == "dsq":
            continue
        scored.append({"team": r["team"], "id": r["id"], "started_at": r["started_at"], "verified": r.get("verified"),
                       "status": "finished", "total_s": total, "first_lap_s": r["warmup_s"], "collisions": hits,
                       "mode": r.get("mode", "time-attack"), "restricted_subscribers": r.get("restricted_subscribers", {})})
    best = {}
    for r in sorted(scored, key=lambda r: (not r["verified"], r["total_s"])):
        best.setdefault(r["team"], r)
    return sorted(best.values(), key=lambda r: (not r["verified"], r["total_s"]))


# The boards. A discipline is what the car perceives, walls or cones; an event is what is
# measured. Trackdrive and autocross run over every ordinary track of the discipline; skidpad
# and acceleration are one fixed track each; overall turns the four into Formula Student
# Driverless points (FS Rules 2026 D 9, table 3: trackdrive 200, autocross 100, skidpad 75,
# acceleration 75) and sums them.
DISCIPLINES = (("walls", "Walls"), ("cones", "Cones"))
EVENTS = (("trackdrive", "Trackdrive"), ("autocross", "Autocross"), ("acceleration", "Acceleration"), ("skidpad", "Skidpad"))
POINTS = {"trackdrive": 200, "autocross": 100, "skidpad": 75, "acceleration": 75}


def pace_board(legs, key):
    """Mean pace over `legs`: a driver's pace on a track is the fastest finished total there
    divided by their own; unfinished tracks count 0 %; the score is the mean over all legs, so
    breadth and speed both count and track length drops out."""
    best = {}
    for t in legs:
        finished = [r["total_s"] for r in t[key] if r["status"] == "finished"]
        if finished:
            best[t["id"]] = min(finished)
    teams = {}
    for t in legs:
        for r in t[key]:
            if r["status"] == "finished":
                teams.setdefault(r["team"], {})[t["id"]] = r
    rows = []
    for team, per in teams.items():
        pace = {tid: round(100.0 * best[tid] / r["total_s"], 1) for tid, r in per.items()}
        rows.append({"team": team,
                     "score": round(sum(pace.values()) / len(legs), 1) if legs else 0.0,
                     "finished": len(per), "of": len(legs), "pace": pace,
                     "verified": all(r.get("verified") for r in per.values()),
                     "totals": {tid: r["total_s"] for tid, r in per.items()},
                     "best_s": min(r["total_s"] for r in per.values())})
    rows.sort(key=lambda r: (-r["score"], not r["verified"], r["team"]))
    return rows


def board(discipline, event, tracks):
    if event in ("trackdrive", "autocross"):
        legs = [t for t in tracks if t["kind"] == discipline and not t.get("event")]
        key = "standings" if event == "trackdrive" else "autocross"
    else:
        legs = [t for t in tracks if t["kind"] == discipline and t.get("event") == event]
        key = "standings"
    return {"discipline": discipline, "event": event, "name": dict(EVENTS)[event], "kind": discipline,
            "tracks": [t["id"] for t in legs], "featured": [t["id"] for t in legs if t["qualifying"]],
            "standings": pace_board(legs, key) if legs else []}


def points(event, rows):
    """FS Driverless points per driver from a board. Trackdrive and autocross use the time
    formulas on the driver's mean pace (time ratio r = 100 / score); skidpad and acceleration
    use the autonomous ranking formula (D 9.2.2)."""
    pmax = POINTS[event]
    out = {}
    if event in ("skidpad", "acceleration"):
        n = len(rows)
        for rank, r in enumerate(rows, 1):
            out[r["team"]] = round(pmax * (n + 1 - rank) / n, 1)
        return out
    for r in rows:
        if r["score"] <= 0:
            out[r["team"]] = 0.0
            continue
        ratio = 100.0 / r["score"]
        if event == "trackdrive":      # D 9.3.3 with Tmax = 2 Tmin, plus the completed-laps 2.5 % per lap, ten laps, scaled by tracks finished
            pts = 0.75 * pmax * max(0.0, 2.0 / ratio - 1.0) + 0.025 * pmax * 10 * r["finished"] / r["of"]
        else:                          # D 9.3.2 with Tmax = 1.4 Tmin (D 9.1 table 11)
            pts = 0.1 * pmax + 0.9 * pmax * max(0.0, (1.4 - ratio) / 0.4)
        out[r["team"]] = round(pts, 1)
    return out


def overall_board(discipline, boards):
    totals, parts = {}, {}
    for event, _ in EVENTS:
        b = boards.get(event)
        if not b or not b["tracks"]:
            continue
        for team, pts in points(event, b["standings"]).items():
            totals[team] = totals.get(team, 0.0) + pts
            parts.setdefault(team, {})[event] = pts
    rows = [{"team": team, "points": round(pts, 1), "parts": parts[team]} for team, pts in totals.items()]
    rows.sort(key=lambda r: (-r["points"], r["team"]))
    return {"discipline": discipline, "event": "overall", "name": "Overall", "max_points": sum(POINTS[e] for e, _ in EVENTS if boards.get(e) and boards[e]["tracks"]),
            "standings": rows}


def build():
    teams = yaml.safe_load((ROOT / "submissions.yaml").read_text())["teams"]
    events = [yaml.safe_load(p.read_text()) for p in sorted((ROOT / "events").glob("*.yaml"))]
    all_results = results.load_all()

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "tracks").mkdir(exist_ok=True)
    tracks = []
    for folder in sorted((ROOT / "tracks").glob("*/track.json")):
        t = json.loads(folder.read_text())
        name = folder.parent.name
        preview = folder.with_name("preview.png")
        if preview.exists():
            shutil.copy(preview, OUT / "tracks" / f"{name}.png")
        shutil.copy(folder, OUT / "tracks" / f"{name}.json")   # the 3D viewer on the track page reads it
        rows = [r for r in all_results if r["event"] == "practice" and r["track"] == name]
        full = [r for r in rows if r.get("mode") != "autocross"]      # the 10-lap board ignores one-lap runs
        tracks.append({"id": name, "name": t["name"], "length": t["length"], "direction": t["direction"],
                       "walls": bool(t["walls"]), "cones": bool(t["cones"]), "kind": "cones" if t["cones"] else "walls",
                       "event": t.get("event"),
                       "preview": preview.exists(),
                       "qualifying": t.get("qualifying", True),
                       "category": t.get("category", "feb"), "difficulty": int(t.get("difficulty", 0)),
                       "standings": standings(full), "autocross": autocross_standings(rows), "attempts": len(rows)})

    for e in events:
        rows = [r for r in all_results if r["event"] == e["id"]]
        e["standings"] = h2h_standings(rows) if e.get("mode") == "head-to-head" else standings(rows)
        e["races"] = sorted({r["race"] for r in rows if r.get("race")}) if e.get("mode") == "head-to-head" else []
        e["results"] = rows if e.get("mode") == "head-to-head" else []
        e["attempts"] = len(rows)
        e["track"] = pathlib.Path(str(e["track"])).name

    practice = [r for r in all_results if r["event"] == "practice"]
    track_ids = [t["id"] for t in tracks if t["qualifying"] and not t.get("event")]
    for team in teams:
        team["qualification"], team["qualified"] = rules_mod.qualification(team["name"], track_ids, practice)

    matrix = {}
    for d, _ in DISCIPLINES:
        boards = {event: board(d, event, tracks) for event, _ in EVENTS}
        boards["overall"] = overall_board(d, boards)
        matrix[d] = boards

    data = {"teams": teams, "tracks": tracks, "events": events,
            "overall": [board(d, "trackdrive", tracks) for d, _ in DISCIPLINES],
            "matrix": matrix,
            "qualify": {"laps": rules_mod.Rules().laps, "max_collisions": rules_mod.QUALIFY_MAX_COLLISIONS},
            "generated": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")}
    (OUT / "data.json").write_text(json.dumps(data, indent=1, default=str))
    shutil.copy(ROOT / "site" / "index.html", OUT / "index.html")
    shutil.copy(ROOT / "assets" / "feb_logo.svg", OUT / "logo.svg")
    shutil.copy(ROOT / "assets" / "feb_icon_256.png", OUT / "icon.png")   # tab icon for browsers that want a bitmap
    return OUT


if __name__ == "__main__":
    print(build())
