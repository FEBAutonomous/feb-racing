# The box: your ROS 2 environment

You get a Linux machine with ROS 2 Humble, our simulator bridge and the car's drivers already
installed. It runs on your laptop inside Docker, keeps whatever you install, and sees one folder
of yours: the FEB folder that holds your clones. The simulator and Foxglove are normal apps that
run beside it. The same box runs on the car's Jetson, so what drives in the simulator drives the
car.

We call it the box. You never type a `docker` command. The one-page version is `quickstart.md`.

## New here: setup in ten minutes

1. Install Docker Desktop (Mac, Windows) or `docker.io` (Linux). On a Mac, OrbStack works too.
   Install VS Code and its Dev Containers extension. Install Foxglove from foxglove.dev/download.
2. Make a folder for all FEB work and clone into it. The folder name does not matter, the
   layout does: the box mounts the folder that holds the clone.
   ```
   mkdir ~/FEB && cd ~/FEB
   git clone git@github.com:FEBAutonomous/feb-racing.git
   cd feb-racing
   ./feb setup --name "Your Name"
   ```
   `setup` pulls the image (a few GB, once), downloads the simulator app for your OS, creates the
   box and points VS Code at it.
3. Try it:
   ```
   ./feb shell          # a terminal in the box. ros2 and colcon work. exit to leave.
   ./feb code           # VS Code attached to the box, every terminal in it is inside
   ./feb up             # the house nodes, the simulator window and Foxglove. Drive with the keys.
   ```

That is the whole install. Windows: do all of this from a WSL 2 (Ubuntu) terminal with Docker
Desktop's WSL integration on, and keep `~/FEB` inside WSL. Mac: keep `~/FEB` under `/Users`.

## Already had the old devkit

```
cd feb-racing && git pull && ./feb setup
```

One condition: the clone must sit in a folder of its own, because the box mounts the folder
that holds it. If `feb-racing` is straight in your home folder, move it first:
`mkdir ~/FEB && mv ~/feb-racing ~/FEB/`. `setup` refuses to go on otherwise.

Your `stack/` folder stays where it is and still works. What changed: the container is no longer
thrown away after every run, so builds are incremental and `pip install` sticks; the launcher is `feb` and `feb-sim run`
is now `feb up`; your whole FEB folder is visible in the box, not
only `stack/`; and the drivers now talk the car's topics, `/scan`, `/odom`, `/drive`, instead of
the simulator's. If you wrote a node against `/autodrive/roboracer_1/...`, see "The contract".

## The commands

| | |
|---|---|
| `./feb shell` | a terminal in the box, in the folder you ran it from. As many as you like. |
| `./feb shell --gui` | the same, plus the box's desktop in a browser tab for rviz2, rqt, turtlesim |
| `./feb shell --gui --apps "turtlesim rqt"` | ...with those windows opened for you |
| `./feb code` | VS Code attached to the box (`--cursor` for Cursor) |
| `./feb up` | house nodes + simulator + Foxglove. Drive by hand. Ctrl-C ends it. |
| `./feb up --track loop --stack "ros2 launch feb_driver drive.launch.py"` | your code drives |
| `./feb up --stack "..." --noise 1` | the same with noisy sensors |
| `./feb up --stack "..." --opponent normal` | a house car to overtake |
| `./feb up --ws ~/FEB/other_ws --stack "..."` | drive from a different workspace |
| `./feb stop` | ends `feb up` from another terminal. The box keeps running. |
| `./feb logs -f` | what `feb up` printed: the build, the house nodes, your stack |
| `./feb reset` | wipes the box. Next command makes a fresh one. Your files are untouched. |
| `./feb update` | git pull, newer image, newer simulator app. A new image replaces the box. |
| `./feb practice --driver NAME` | scored runs under competition rules on every practice track of the driver's kind; `--track T` for one |
| `./feb submit` | post every clean run (finished, no collisions) not yet posted, one pull request; `--dry-run` to look first, `--all` for the rest |
| `./feb up --driver feb_driver --mode-name inspection` | a driver by name and mode, from its `driver.yaml` |

`./feb up --help` lists the rest: `--mode`, `--cars`, `--camera`, `--look`, `--headless`, `--fps`,
`--lidar-hz`, `--no-foxglove`, `--no-sim`, `--car`, `--max-speed`.

## Where things live

```
~/FEB/                      yours. Mounted in the box at /home/feb/FEB. Same bytes, not a copy.
  feb-racing/               the clone
    stack/                  your packages. Built by feb up. Gitignored.
    tracks/  docs/  ...
  some-other-repo/          anything else you clone. Visible in the box the moment it exists.
the box                     Ubuntu 22.04, ROS 2 Humble, the drivers, what you apt/pip install.
                            Kept until feb reset or a new image.
~/.feb-sim/app/             the simulator app, downloaded by setup
```

Edit on the laptop with anything, or in VS Code attached to the box. Both see the same file.
A file you create in the box is yours on the laptop, owned by you.

What does not survive `feb reset` or an image update: apt and pip installs, build folders you put
outside `~/FEB`, bash history. Put a dependency your package needs in its `package.xml` or a
`requirements.txt` so the next person gets it, and tell us if the image should carry it.

## Your code

A package is a folder under `stack/` (or any workspace you point `--ws` at). Make one:

```
./feb shell
cd stack
ros2 pkg create my_driver --build-type ament_python --node-name driver --dependencies rclpy sensor_msgs ackermann_msgs
cd ..
colcon build --symlink-install     # or just: feb up --stack ..., which builds first
source install/setup.bash
ros2 run my_driver driver
```

`--symlink-install` means a Python edit takes effect on the next `ros2 run` with no rebuild.
The template to copy is `starter_kit/feb_driver`, a follow-the-gap driver, with every value
explained in its config file. `feb_cone_driver` (cones, lidar plus camera) is the other one to
copy. Two packages are shared and live once in the team repo, `feb-stack/shared/`: `feb_mission`
(the supervisor's state as go/stop hooks) and `feb_cone_ordering` (the team's cone ordering as a
service, C++). Copy them into your workspace to use them in the simulator, but never list them in
your `packages:`; on the car they're already there, and submit refuses a duplicate.

Keep your workspace in git. `stack/` is gitignored in this repo on purpose: make your own repo
for it (`cd stack && git init`), or clone your repo beside `feb-racing` in `~/FEB` and use
`--ws`. A box reset, a laptop change or a teammate taking over then costs nothing.

## The contract

Your node reads three topics and writes one. They are the RoboRacer car's, so the same node runs
on the Jetson untouched.

| Topic | Type | |
|---|---|---|
| `/scan` | sensor_msgs/LaserScan | the lidar, frame `laser` |
| `/odom` | nav_msgs/Odometry | pose and speed, dead-reckoned from the wheels and the gyro |
| `/imu` | sensor_msgs/Imu | |
| `/camera/color/image_raw` | sensor_msgs/Image | the front camera |
| `/drive` | ackermann_msgs/AckermannDriveStamped | **yours**: speed in m/s, steering angle in rad |

Under you, started by `feb up`: the simulator bridge (or the VESC and lidar drivers on the car),
the adapter that turns the simulator's topics into those, the supervisor and the Foxglove bridge.

The supervisor is the safety layer. It sits between `/drive` and the wheels and publishes its state
on `/feb/state`: IDLE, MANUAL (gamepad driving), AUTONOMOUS (your `/drive` goes through, under a
speed cap) or STOP (lidar silent, collision imminent, stop button). In the simulator it needs no
gamepad; on the car nothing moves unless a dead-man button is held.

The simulator's raw topics are still there (`/autodrive/roboracer_1/...`) if you want the lap
counter or the ground-truth position for debugging. Do not drive from them: the car does not have
them.

## The car's rules, in the simulator

On the car nothing moves unless a button is held, and your stack is running the whole time, also
while a human drives. `feb up --deadman` gives you the same in the simulator, with the keyboard
as the gamepad:

```
./feb up --deadman --driver feb_driver
```

The terminal becomes the pad: hold SPACE and your stack drives, hold M and the arrows drive, X is
the stop button, Q quits. Let go and the car stops within a third of a second, like the real
button. The nodes' output goes to `./feb logs -f` in another terminal. Run it this way at least
once before a driver goes near the car, because of the next paragraph.

**State and your stack.** The supervisor publishes `/feb/state`. Your stack is never started or
stopped by the button; its `/drive` is simply ignored until the button is held. So anything in
your stack that remembers, an MPC warm start, a localiser, an integrator, is stale when the
button goes down. The rule: on AUTONOMOUS, start clean. The starter kit's `feb_mission` package
is a small node that turns the state into two hooks, `on_go` and `on_stop`, and publishes
`/feb/mission/live`; the starter drivers' launch files include it whenever it is in the
workspace. The gap follower needs nothing, it re-plans from every scan. A stack with memory
copies the package and fills in the hooks, or watches the flag from the node that has the memory.

## Looking at things

Foxglove, the app on the laptop, is the everyday tool: `feb up` opens it connected to the box.
By hand: Open connection, Foxglove WebSocket, `ws://localhost:8765`. Scans, markers, plots, bag
playback.

rviz2 and anything else with a window run on the box's desktop, which is a browser tab:
`./feb shell --gui`, then `rviz2 &` or `rqt_graph &` from any box terminal. It is software
rendered, so expect lag; fine for a look, not for watching a race.

Bags: `ros2 bag record -a -o ~/FEB/bags/run1` in a box terminal while driving, then
`ros2 bag play ~/FEB/bags/run1` later with `feb up --no-sim` for the house nodes. Keep bags out of
git.

## Driver packets and modes

A driver is a ROS package with a `driver.yaml` beside its `package.xml`. The file says what the
driver is for and how to start it, so the launcher, the lead and the car all read the same thing:

```yaml
name: feb_driver
kind: walls                 # walls | cones | any
maintainer: your name
modes:
  trackdrive: {launch: drive.launch.py, max_speed: 3.0}
  inspection: {launch: drive.launch.py, max_speed: 0.8}
default_mode: trackdrive
```

Then `./feb up --driver feb_driver` is the same as `--stack "ros2 launch feb_driver drive.launch.py mode:=trackdrive"`
with the supervisor's cap set from the mode. `--mode-name inspection` picks another mode. Your
launch file receives `mode:=` and may ignore it; the cap is applied either way.

The modes every driver should have, so the car can be driven the same way whoever wrote the code:

| Mode | Means | Cap |
|---|---|---|
| `inspection` | the first lap of a new track, the first drive of a new build, a corridor at home. Slow, no heroics. | 0.5 to 0.8 m/s |
| `trackdrive` | the race. The driver's own limits, the cap only as a ceiling. | the driver's |

Add your own below those, `skidpad`, `acceleration`, `corridor`, whatever the code does. A mode
is just a launch file and a cap; the supervisor does not care how many there are. The starter
kit's three packages all carry a `driver.yaml` to copy from.

Two optional keys: `packages:`, the list of your own packages that `feb submit` copies to the team
repo (default: just the driver); and `adapter:`, settings for the sim adapter's throttle law. Leave
`adapter:` out unless your driver runs its own speed controller, as the racer does; then
`adapter: {speed_per_throttle: 25.11, throttle_kp: 0.0, throttle_ki: 0.0}` makes the adapter a
plain pass-through so two loops don't fight.

## Getting a driver onto the car

Team drivers live in `FEBAutonomous/feb-stack`, one folder per person under `drivers/`. Nobody
edits it by hand; `feb submit` fills it. The path from your laptop to the wheels:

1. It laps clean: `./feb practice --driver my_driver`, every practice track of its kind, zero
   collisions.
2. `./feb submit`. Two pull requests from one command: the results to the scoreboard, and your
   driver's code to `feb-stack`. The code half copies the packages listed in your `driver.yaml`
   under `packages:` into `drivers/<your name>/`, commits on a branch named after you, pushes to
   your fork if you can't push to the repo, and opens the pull request or updates the one already
   open. A submit with nothing changed does nothing. `--no-code` posts only results,
   `--no-results` only code, `--dry-run` shows the plan. Without the GitHub CLI installed it
   pushes the branches with git and prints the links to open the pull requests by hand.
   `packages:` lists only your own packages: `feb_mission`, `feb_cone_ordering` and anything else
   every driver may use live once in `feb-stack/shared/`, kept by the lead, and submit refuses a
   name that is already there.
3. The lead reviews the diff, reruns the practice, merges. Merged means car-ready.
4. On the Jetson, which has `feb-stack` cloned beside `feb-racing`:
   ```
   cd ~/FEB/feb-stack && git pull
   cd ~/FEB/feb-racing && ./feb up --car --driver my_driver --mode-name inspection
   ```
   On the car the workspace is `feb-stack` by default, so every merged driver is built and the
   named one launches: the VESC, lidar and gamepad drivers, the supervisor with the dead-man and
   the mode's cap, then your launch file with your car yaml on top. Hold the autonomous button
   and the car drives your code; let go and it stops. Foxglove on a laptop connects to
   `ws://<the jetson's address>:8765`. First drive in `inspection`, then `trackdrive`.

To run a teammate's merged driver in the simulator on a laptop: `./feb up --ws ~/FEB/feb-stack --driver theirs`.

**Numbers that differ on the car.** Same code, different physics: steering lag, lidar height, how
hard it brakes. Keep the shared numbers in your driver's yaml and put only the lines that differ in
`config/<name>.car.yaml` beside it. The starter drivers' launch files load that file on top of the
base one when they run on the Jetson, and ignore it in the simulator. Both starter drivers ship one
with the usual suspects commented out.

Nothing is flashed or copied by hand. The Jetson is a box with a `git pull`: the same image, built
for arm64 by the same CI, created by `./feb setup` with the car's USB devices passed in (it
notices it is on the car by the `/dev/sensors` udev names from the RoboRacer guide). Wiring and
the one-time Jetson setup are in `docs/build-car.md`.

## Ports and overrides

| Variable | Default | |
|---|---|---|
| `FEB_DEVKIT_PORT` | 4567 | the bridge; the simulator connects here |
| `FEB_FOXGLOVE_PORT` | 8765 | the Foxglove WebSocket |
| `FEB_VNC_PORT` | 6080 | the web desktop |
| `FEB_BOX_NAME` | feb-box | the container; change it to run two boxes side by side |
| `FEB_DEVKIT_GUI_IMAGE` | `ghcr.io/febautonomous/feb-devkit:gui` | the box's image |
| `FEB_DEVKIT_IMAGE` | `ghcr.io/febautonomous/feb-devkit:latest` | the league image, used by practice |
| `FEB_SIM_APP` | the installed app | another simulator binary |

## When something is wrong

| Symptom | Fix |
|---|---|
| `ros2: command not found` | you are in a laptop terminal. `./feb shell`. |
| my node is not there | it did not build: `./feb logs -n 100` shows the colcon error, or `colcon build` in a box terminal and read it |
| the simulator says not connected | the bridge is still starting, give it a moment; `./feb logs -f` |
| `port is already allocated` | another box or devkit holds the port: `./feb reset`, or set `FEB_DEVKIT_PORT` |
| `mounts denied` on a Mac | the FEB folder is outside `/Users`; move it, or add it in Docker Desktop, Resources, File Sharing |
| the desktop tab cannot connect | reload it; still nothing, `./feb reset` |
| the box is in a strange state | `./feb reset`. Thirty seconds, nothing of yours is in it. |
| the car runs backwards | swap two motor leads (build-car.md); in the sim, your sign on `/drive` |

## Under the hood, for the curious

The box is one `docker run` of `ghcr.io/febautonomous/feb-devkit:gui` with your FEB folder bind-mounted,
ports 4567, 8765 and 6080 published, user `feb`, and `box.sh` as its process, which starts the
web desktop and sleeps. `feb shell` is `docker exec -it`. `feb up` runs `feb-up` in the box,
which builds the workspace and launches `feb_tools/up.launch.py`, and starts the simulator app with
`--connect 127.0.0.1:4567`. Competition and practice runs use a throwaway container from the
`:latest` image with the league's entrypoint, exactly as the league does; a submission is a
Dockerfile `FROM` that image with your stack copied in (`starter_kit/Dockerfile`).
