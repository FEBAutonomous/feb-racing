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


@dataclass
class Rules:
    laps: int = 10
    warmup_laps: int = 1
    collision_penalty_s: float = 10.0
    max_collisions: int = 10
    timeout_s: float = 300.0

    @property
    def total_laps(self):
        return self.warmup_laps + self.laps

    @property
    def mode(self):
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
