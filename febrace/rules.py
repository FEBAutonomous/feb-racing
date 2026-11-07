"""Scoring rules, the same as the RoboRacer Sim Racing League used them in 2026.

A time-attack attempt is one warm-up lap followed by `laps` timed laps. Total time is the
sum of the timed laps plus `collision_penalty_s` per collision counted after the timer
starts. Attempts that do not complete all laps before `timeout_s` are DNF and rank
after every finisher, by laps completed. More than `max_collisions` collisions is DSQ.

An autocross attempt (`laps=0`) is the warm-up lap alone, the car's first sight of the
track: its total is that lap plus the penalty for collisions on it. Every full attempt
also yields an autocross time from its warm-up lap, so the autocross board takes both.
"""
from dataclasses import asdict, dataclass

# To enter the first competition a team must, on EVERY practice track, complete a full
# attempt (all timed laps) with at most this many collisions. Verified (rerun) results count.
QUALIFY_MAX_COLLISIONS = 0   # zero: a run that touches a wall does not count
QUALIFY_LAPS = 10            # a run scored on fewer timed laps (feb practice --laps N) is a test, never posted


# The Formula Student fixed-layout events, timed on lines from the car's true position rather than
# the lap counter (FS Rules 2026 D 4, D 5, D 10.1.7): a run is one pass through the programme.
EVENT_RULES = {
    "skidpad": dict(laps=0, warmup_laps=1, collision_penalty_s=0.2, max_collisions=10, timeout_s=300.0),
    "acceleration": dict(laps=0, warmup_laps=1, collision_penalty_s=2.0, max_collisions=10, timeout_s=120.0),
}


@dataclass
class Rules:
    laps: int = 10
    warmup_laps: int = 1
    collision_penalty_s: float = 10.0
    max_collisions: int = 10
    timeout_s: float = 300.0
    event: str = "trackdrive"       # trackdrive (laps=0: its warm-up lap alone, autocross) | skidpad | acceleration

    @classmethod
    def for_track(cls, track_json, **overrides):
        """The rules a track asks for: its `event` key picks the FS event rules, else trackdrive."""
        event = (track_json or {}).get("event", "trackdrive")
        if event in EVENT_RULES:
            return cls(event=event, **{**EVENT_RULES[event], **overrides})
        return cls(**overrides)

    @property
    def total_laps(self):
        return self.warmup_laps + self.laps

    @property
    def mode(self):
        if self.event in EVENT_RULES:
            return self.event
        return "autocross" if self.laps == 0 else "time-attack"

    def to_dict(self):
        return asdict(self)


def score(rules, lap_times, collisions):
    """Return (status, timed_laps, penalty_s, total_s, best_lap_s)."""
    timed = [t for t in lap_times[rules.warmup_laps:] if t is not None][: rules.laps]
    penalty = rules.collision_penalty_s * collisions
    best = min(timed) if timed else None
    if collisions > rules.max_collisions:
        return "dsq", timed, penalty, None, best
    if rules.laps == 0:                      # autocross: the warm-up lap is the result
        first = lap_times[0] if lap_times and lap_times[0] is not None else None
        if first is None:
            return "dnf", [], penalty, None, None
        return "finished", [], penalty, round(first + penalty, 3), round(first, 3)
    if len(timed) < rules.laps:
        return "dnf", timed, penalty, None, best
    return "finished", timed, penalty, round(sum(timed) + penalty, 3), best


def score_event(rules, lines, stopped, collisions):
    """The line-timed events. `lines` are the probe's crossings in order: (name, t_sim, side),
    `stopped` whether the car came to rest inside the stop zone (none crossed the stop_end
    line). Returns (status, lap_times, penalty_s, total_s, best_lap_s), lap_times being the
    segment times the event is made of."""
    penalty = rules.collision_penalty_s * collisions
    if collisions > rules.max_collisions:
        return "dsq", [], penalty, None, None
    if rules.event == "acceleration":
        t_start = next((t for n, t, _ in lines if n == "start"), None)
        t_finish = next((t for n, t, _ in lines if n == "finish" and t_start is not None and t > t_start), None)
        if t_start is None or t_finish is None:
            return "dnf", [], penalty, None, None
        run = round(t_finish - t_start, 3)
        if not stopped:
            return "dnf", [run], penalty, None, run          # D 5.2.4: no stop in the zone, no result
        return "finished", [run], penalty, round(run + penalty, 3), run
    if rules.event == "skidpad":
        # five crossings of the centre line: entry, then after each of the four laps; the first
        # two laps must be on the right circle (side > 0), the next two on the left (D 4.2.1)
        xs = [(t, side) for n, t, side in lines if n == "centre"]
        if len(xs) < 5:
            return "dnf", [round(b[0] - a[0], 3) for a, b in zip(xs, xs[1:])], penalty, None, None
        laps = [round(xs[i + 1][0] - xs[i][0], 3) for i in range(4)]
        sides = [xs[i + 1][1] for i in range(4)]
        if not (sides[0] > 0 and sides[1] > 0 and sides[2] < 0 and sides[3] < 0):
            return "dnf", laps, penalty, None, min(laps)       # wrong order of circles
        if not stopped:
            return "dnf", laps, penalty, None, min(laps)
        timed = round((laps[1] + laps[3]) / 2, 3)               # D 4.2.2: the second lap of each circle, averaged
        return "finished", laps, penalty, round(timed + penalty, 3), timed
    raise ValueError(rules.event)


def autocross(result):
    """(first-lap total with its penalty, collisions on that lap) for any attempt, or (None, 0) if
    the warm-up lap was not completed. Attempts record the lap of each collision; older files
    only have wall-clock times, so collisions before the warm-up lap time are counted."""
    first = result.get("warmup_s")
    if first is None:
        return None, 0
    laps = result.get("collision_laps")
    if laps is not None:
        hits = sum(1 for lap in laps if lap == 0)
    else:
        hits = sum(1 for t in result.get("collision_times") or [] if t <= first)
    penalty = (result.get("rules") or {}).get("collision_penalty_s", Rules.collision_penalty_s)
    return round(first + penalty * hits, 3), hits


def rank_key(result):
    """Sort key: finishers by total time, then DNFs by laps completed, then DSQs."""
    order = {"finished": 0, "dnf": 1, "dsq": 2}.get(result.get("status"), 3)
    return (order, result.get("total_s") or float("inf"), -len(result.get("lap_times") or []),
            result.get("best_lap_s") or float("inf"))


def qualifies(result):
    """A practice attempt that meets the qualification bar: finished, at most QUALIFY_MAX_COLLISIONS."""
    return result.get("status") == "finished" and result.get("collisions", 99) <= QUALIFY_MAX_COLLISIONS


def qualification(team, track_ids, practice_results):
    """Per-track status for a team: 'verified', 'unverified' (self-reported only) or None; and
    whether the team is qualified (verified on every track)."""
    status = {}
    for track in track_ids:
        rows = [r for r in practice_results if r["team"] == team and r["track"] == track and qualifies(r)]
        status[track] = "verified" if any(r.get("verified") for r in rows) else ("unverified" if rows else None)
    return status, all(v == "verified" for v in status.values()) and bool(track_ids)
