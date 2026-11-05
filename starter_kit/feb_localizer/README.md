# feb_localizer: the localisation project

A driver that already drives, maps and remembers, around one file you write: `feb_localizer/localizer.py`.

**Lap one** is reactive (the competitor's law) while every scan is stamped into an occupancy grid at
the pose your `Localizer` reports, and the path driven is recorded. **Lap two** follows that path by
pure pursuit on your pose. The `Localizer` that ships only dead-reckons (`predict`), so by lap two the
pose has drifted and the car follows a path that is no longer where the track is. Your job: make
`update_mapping` (lap one, the grid so far) and `update_racing` (lap two, the finished grid) correct
the pose so lap two works.

```
./feb new my_localizer --from feb_localizer      # your copy
./feb up --driver my_localizer                   # watch lap two go wrong (fallback is off in the simulator on purpose)
./feb up --driver my_localizer --track porto
./feb shell -c "ros2 run my_localizer score bags/loop_laps"    # the number to make go down, no simulator needed
```

`score` replays a recorded lap with ground truth through your class and prints the median and
end-of-lap error for the mapping lap and the racing lap, plus a picture `<bag>_score.png` of truth
versus your estimate. Two bags ship in `bags/`: `loop_laps` and `porto_laps`. Record your own with
`ros2 bag record /scan /odom /autodrive/roboracer_1/ips /autodrive/roboracer_1/imu /autodrive/roboracer_1/lap_count`
while `./feb up` runs any driver.

What the `Localizer` receives: `predict(v, yaw_rate, dt)` from every odometry message; `update_*(points, grid)`
from every scan, `points` an (n, 2) array in the car frame (x forward, y left, metres) and `grid` the
`Grid` in `grid.py` (`grid.fit(points, pose)` scores a candidate pose, `grid.occupied()` is the map).
Set `self.x, self.y, self.yaw` and the driver uses them.

Foxglove shows `/feb/pose`, `/feb/path`, `/feb/map` and `/feb/fit`. `oracle: true` in `config/driver.yaml`
swaps in the simulator's true position so you can see lap two with a perfect localiser first.
The worked example of a scan matcher that does this job is `feb_racer/localizer.py` in the lead's stack.
