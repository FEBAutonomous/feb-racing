# Quickstart

One page. The long version is `devkit.md`.

**The box** is a Linux machine with ROS 2 inside Docker. It sees your `~/FEB` folder, keeps what
you install, and runs the same on the car. The simulator and Foxglove are apps beside it.

## Setup, once

1. Install Docker Desktop (OrbStack on a Mac is fine), VS Code with the Dev Containers extension, and Foxglove.
2. ```
   mkdir ~/FEB && cd ~/FEB
   git clone git@github.com:FEBAutonomous/feb-racing.git && cd feb-racing
   ./feb setup --name "Your Name"
   ```
   Your name is how your results are listed. Get it added to `submissions.yaml` with your GitHub login.
3. `./feb up`. The simulator opens, arrow keys drive, close the window to stop.

Had the old devkit? `git pull` won't work, the history was rewritten. Move the old clone aside,
do steps 2 and 3 above, then copy your old `stack/` contents into the new `stack/`.

## Every day

| | |
|---|---|
| `./feb shell` | a terminal in the box: `ros2`, `colcon`, `pip`, anything. As many as you like. |
| `./feb code` | VS Code inside the box |
| `./feb shell --gui` | plus a desktop in the browser for rviz2, rqt, turtlesim |
| `./feb up --driver my_driver` | your driver races in the simulator, Foxglove opens |
| `./feb up --deadman --driver my_driver` | the same under the car's rules: hold SPACE to drive, M for manual |
| `./feb stop` | ends that; the box stays |
| `./feb practice --driver my_driver` | scored runs, competition rules, every practice track of your driver's kind |
| `./feb submit` | posts the clean runs to the board and your code to the team repo |
| `./feb update` | new code, new image, new simulator |
| `./feb reset` | a fresh box; your files are untouched |

## Your driver

```
cp -r starter_kit/feb_driver stack/my_driver && cd stack/my_driver
mv feb_driver my_driver && mv resource/feb_driver resource/my_driver
grep -rl feb_driver . | xargs sed -i 's/feb_driver/my_driver/g'      # Mac: sed -i '' ..., or do it in ./feb shell
cd ../..
```

A ROS package carries its name in four places, hence the two rename lines. Then edit. One package is a whole driver:

- `driver.yaml`: its name, kind (walls or cones), modes (launch file plus speed cap), and
  `packages:`, the list of your own packages it needs.
- `config/driver.yaml`: every number, one per line with a comment. `config/driver.car.yaml`: only
  what differs on the real car, loaded on top of it there.
- `launch/drive.launch.py` and the node.

Topics in: `/scan`, `/odom`, `/imu`, `/camera/color/image_raw`. Out: `/drive`, a speed in m/s and a
steering angle in rad. Same on the car. A Python edit takes effect on the next run, no rebuild.

Also in the starter kit to copy: `feb_cone_driver` (cones). Shared and already on the car,
don't copy or list them: `feb_mission` (the supervisor's state as go/stop hooks, for a stack with
memory) and `feb_cone_ordering`. Your launch file can start them; they're in the workspace.

Keep `stack/` in a repo of your own; this repo ignores it on purpose.

## To the board, and to the car

`./feb practice --driver my_driver` runs the competition rules on every practice track of your
driver's kind, one attempt each. `./feb submit` then does two things in one go: posts every run
with zero collisions to the scoreboard, and copies your driver's packages to the team repo
`feb-stack` under `drivers/<you>/`, on your own branch, as a pull request. The lead reviews and
merges; merged means car-ready. Nothing of yours is pushed without that command, and a submit
with no changes does nothing.

With the GitHub CLI signed in (`gh auth login`, once) the pull requests open themselves. Without
it, the branches are pushed and you get two links to click.

On the car: `git pull` in `~/FEB/feb-stack`, then `./feb up --car --driver my_driver --mode-name inspection`.
LB held: you drive. RB held: your code drives, capped. Let go: it stops.

## What survives what

| | files in `~/FEB` | apt, pip, bash history in the box |
|---|---|---|
| closing the laptop | yes | yes |
| `feb reset` | yes | no |
| `feb update` with a new image | yes | no |

Put a dependency in your package's `package.xml`, not in the box.

## Stuck

`ros2: command not found`: a laptop terminal, use `./feb shell`. A build error: `./feb logs -n 100`.
Anything weird: `./feb reset`, thirty seconds, nothing of yours is in there.
