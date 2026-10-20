# Building our RC car, step by step

This is the official RoboRacer build (f1tenth.readthedocs.io, "Building the RoboRacer Car")
written out for the exact parts we have, in order, with a check at the end of each step. Where
our car differs from the guide there are only three places, and they are marked **OURS**:
the motor swap, the RPLidar S2 on our own deck, and the Orin Nano in place of the Xavier NX.
Everything else is the guide's step, and the guide's photos still apply.

Nothing here is irreversible. The worst case at any step is undoing a screw.

## What you have

| Part | Where it goes |
|---|---|
| Traxxas Ford Fiesta ST Rally BL-2s (74154-4) | the chassis; its ESC, receiver and motor come out |
| Traxxas Velineon 3500 motor (3351R) | replaces the BL-2s motor |
| VESC 6 MkVI with heat sink | motor controller, on the deck |
| Jetson Orin Nano Super dev kit + NVMe SSD | the computer, on the deck |
| Ambimat RoboRacer power board | 19 V for the Jetson, 5 V and 12 V for the rest |
| RPLidar S2 + its USB adapter | on the deck, front |
| Intel RealSense D435if | front, later (3D-printed mount from the RoboRacer Drive folder) |
| Zeee 3S 6000 mAh batteries, B6AC charger, LiPo bag | under the deck |
| 8BitDo Ultimate 2C gamepad | your dead-man switch |
| Platform deck, our V3.1 with S2 holes, 0.25 in acrylic | `hardware/deck/` |
| Power board adaptor plate, 3 mm acrylic or printed | `hardware/deck/` |
| Standoffs: 4 × 25 mm, 6 × 45 mm (M3); 4 × M2.5 10 mm (14 mm ones unused) | deck and Jetson |
| Screw assortment M2 to M5; antennas; DP dummy plug; cables and adapters | see the steps |

Tools: the hex keys from the Traxxas box, a small Phillips, a T3 Torx for the Jetson, callipers
or a ruler, zip ties, blue thread-locker (optional but worth it).

Three rules for the whole build: battery unplugged unless a step says otherwise; red to red and
black to black every single time; nothing near the driveshaft that runs down the middle.

---

## Session 1: the chassis (about an hour)

### 1.1 Body off
Pull the four body clips, lift the body off.
*Check:* you see the chassis tub with a blue box (steering servo) on one side, the ESC (speed
control) and receiver box, and the motor at the back.

### 1.2 Stock electronics out
Guide: "Removing Traxxas Stock Components". Unplug and unscrew the BL-2s ESC; open the receiver
box, unplug the servo lead and the ESC lead from the receiver, unscrew the box; pull the antenna
tube; remove the plastic battery hold-down. **Keep the servo and its lead**: it is the only
electrical part that stays.
*Check:* nothing electrical on the chassis but the servo, with its three-wire lead loose.

### 1.3 Motor swap  **OURS**
The BL-2s motor has a keyed Traxxas plug the VESC cannot take and is rated for 2S; the Velineon
3500 has three bullet leads and takes 3S. Same size can, same 3 mm shaft, so the small gear moves
across.

1. Take the gear cover off (one middle screw) so you can see the small gear on the motor shaft (the
   **pinion**) meshing with the big gear (the **spur**).
2. Loosen the pinion's set screw (tiny hex) and slide the pinion off the BL-2s shaft.
3. Undo the motor's mounting screws on the blue motor plate; lift the BL-2s motor out. Note which
   way its wires faced.
4. Fit the Velineon in its place, wires towards where the BL-2s wires were, screws loosely.
5. Slide the pinion onto the Velineon's shaft, set screw onto the shaft's flat, tighten.
6. **Gear mesh:** slide the motor so the pinion meets the spur with a strip of paper between the
   teeth; tighten the motor screws; pull the paper out. Turn a wheel by hand: smooth, no clicking,
   no binding.
7. Gear cover back on. Box the BL-2s motor and ESC as spares.

*Check:* pushing the car along the bench, the motor turns freely and quietly.

### 1.4 Standoffs on the chassis
Guide: "Attaching the Standoffs". Remove the nerf bars (four screws each, from underneath; on the
BL-2s they are the clipless body mounts in the same holes). Fit **three 45 mm standoffs** into the
chassis with the removed M3 × 10 screws from below, in the **side-rail holes the nerf bars used**:
two on the motor side, one on the battery side, front. A drop of thread-locker on each.
*Check:* three posts standing up; the two on the motor side 123.85 mm (4 7/8 in) apart, matching
the deck's edge holes. If the deck's holes land 3 mm off, a standoff is in a neighbouring rail
hole: move it, do not bend the deck onto it.

### 1.5 Battery, dry fit only
Place a battery in the compartment opposite the motor, see that the deck will clear it. Do not plug
it in. Read the guide's LiPo warning once, properly: charge only in the bag, never leave one
plugged in, never short the leads, never plug in backwards.

---

## Session 2: the deck (about an hour)

Lay the deck on the bench: lidar end (the rounded lobe with the many small holes) **away** from you.
The deck is not symmetric; the two big M5 holes for the VESC sit off-centre, on the side nearer you.

### 2.1 VESC
Guide: "Mounting the VESC". VESC on top, its battery lead facing away, its three motor leads facing
you. Two **M5 × 20** screws from underneath, diagonal corners, into the case. Fit the heat sink on
top if its kit allows with the deck screws; otherwise leave it off until the VESC has run and you
know its temperature.
*Check:* VESC solid, no wobble, battery lead pointing to the lidar end.

### 2.2 Antenna mount
Guide: "Mounting the Antenna". The 3D-printed antenna mount on two **45 mm standoffs** behind the
VESC, with its two cables. The antennas themselves screw on last.

### 2.3 Jetson Orin Nano  **OURS, but the same as the guide**
The guide's NX photos apply: your Orin Nano dev kit has the same board size and the same four
mounting holes. What differs: it has a fan on top (leave it), and it needs its SSD.
1. Before mounting: fit the NVMe SSD into the **long** M.2 slot underneath (2280, Key M; the short
   slot holds the Wi-Fi card). Unscrew the M2 post screw, slide the SSD in at about 20 degrees,
   press it flat, screw back finger-tight.
2. Take off the plastic antenna bar (two Phillips screws; keep them) and clip the two new antenna
   cables onto the Wi-Fi card's tiny gold connectors (press until they click).
3. Find which deck holes the Jetson's corners meet and use the matching hardware:
   86 × 58 mm pattern, 2.75 mm holes → **M2.5 × 10 standoffs**, M2.5 × 10 from under the deck
   (6 mm does not reach through 0.25 in acrylic), M2.5 × 6 on top;
   42 × 79 mm pattern, 3.3 mm holes → **25 mm M3 standoffs**, M3 × 8 from underneath, M3 on top.
   Never the Jetson flat on the deck: the standoffs are its airflow.
*Check:* Jetson level, fan clear, ports facing where cables can reach, DC jack reachable.

### 2.4 Power board
Guide: "Mounting the Powerboard". Three holes: M3 × 20 screws from underneath into **25 mm M3
standoffs** (the guide's; the four 25 mm are free because the Jetson takes M2.5), board on top with
M3 × 8, green terminal blocks facing the lidar end. Only three of the
board's holes are used; one sits over nothing. There is a gap between it and the VESC.
If your board's holes do **not** meet the deck's (the V10 Ambimat board is 66 × 42 mm, the deck's
pattern 44 × 94 mm), the official `Powerboard Adaptor VX-V10` plate in `hardware/deck/` bridges
them: deck to adaptor as above, board to adaptor with four M3 × 8 and nuts.
*Check:* the board's two switches are reachable with the deck on the car.

### 2.5 RPLidar S2  **OURS**
Our deck has four holes for it (3.4 mm, a 41.7 mm square centred on the lobe; the first deck had
this square turned 30 degrees, the second cut has it straight). S2 on top, cable to the rear; four
**M3 × 10** screws from underneath. Not longer: the S2's holes are 4 mm deep.
*Check:* the S2 sits flat, can be spun by hand without touching anything, and nothing is in its
scan plane (about 30 mm above the deck) except the antennas, which are behind it.

---

## Session 3: deck onto chassis and wiring (about an hour)

### 3.1 Deck onto the chassis
Guide: "Mounting the Upper Level Chassis". VESC towards the back of the car. Pass the servo lead up
through a deck slot. Three **M3 × 10** screws into the three chassis standoffs.
*Check:* deck level, battery can still slide in and out, nothing touches the driveshaft.

### 3.2 Motor to VESC
Guide: "Connecting the Brushless Motor to the VESC". The three 4 mm-to-3.5 mm bullet adapters
onto the motor's three leads, then onto the VESC's leads: **A to white, B to yellow, C to blue**.
If the car later runs backwards, swap white and blue. Push the bullets fully home.

### 3.3 Servo to VESC
Guide: "Attaching the PPM Cable". Three header pins into the servo lead's plug; the Adafruit
JST-PH cable onto the pins, **brown/black to the servo's black (ground)**; JST end into the
VESC's PPM port.
*Check:* polarity. Ground to ground, twice.

### 3.4 Battery to VESC  **OURS**
The battery's big EC5 plug into the **EC5-male-to-XT90-female** adapter, adapter onto the VESC's
XT90. Not yet: this is the step where things start to move. Just see that it reaches.

### 3.5 Power board inputs and outputs  **OURS**
Per the board's own manual (v2024.1):
- Battery in: the battery's small white **balance plug**, through the balance extender, to the
  board's battery socket (J9). The board's switch SW1 set to battery.
- Jetson: the board's **19 V DC jack output (J7)** to the Jetson's DC jack with the male-to-male
  barrel cable you have. Centre pin is positive on both. (The guide's 12 V pigtail route is the old
  way and too weak for an Orin Nano; use J7.)
- S2: its USB adapter plugs into a Jetson USB port; it powers from USB. Nothing to the board.
- SW2 is the board's power switch. Off for now.
*Check:* read the two switch labels; the manual warns they are printed swapped on some boards.

### 3.6 USB
VESC micro-USB to a Jetson USB port. S2 adapter to a Jetson USB port. Gamepad dongle to a third.
Antennas screwed onto the mount. Zip-tie every cable away from the driveshaft and the wheels.
*Check:* turn each wheel and the driveshaft by hand with the cables tied: nothing catches.

---

## Session 4: software (done 2026-10-18 for the Jetson; the VESC and the first drive remain)

The Jetson is flashed and set up. What it is, so whoever picks it up knows:

- **Flashed** from the team PC with Jetson Linux R36.4.3 (the JetPack 6.2 base) onto the NVMe,
  by the script in `~/FEB/jetson-flash/prepare_and_flash.sh` on that PC. No SDK Manager: it
  needs a web login, the plain Jetson Linux downloads do not. To reflash: power off, a jumper
  across pins 9 and 10 of the J14 header while plugging the power in, USB-C to the PC, run the
  script. The Jetson shows up as USB `0955:7523 NVIDIA APX` when it is in recovery mode.
- **Login** `feb`, password `feb`, hostname `feb-car`. Autologin on a monitor.
- **Networks, in the order it tries them:** a known Wi-Fi (home `mandem` is in; add a phone
  hotspot the same way: `nmcli con add type wifi ssid NAME con-name NAME wifi-sec.key-mgmt wpa-psk wifi-sec.psk PASSWORD`);
  if none is found 45 s after boot it becomes its own hotspot, SSID `feb-car`, password
  `febautonomous`, address `10.42.0.1`. It is also on the team tailnet as `feb-car`, so with
  any internet `ssh feb@feb-car` works from the Mac or the PC. Over a USB-C cable to a computer
  it is `192.168.55.1`.
- **Installed:** Docker, `feb` in the docker group; git, curl, PyYAML. `~/FEB/feb-racing` and
  `~/FEB/feb-stack` cloned. The udev rules for the VESC (`/dev/sensors/vesc`) and the RPLidar S2
  (`/dev/sensors/rplidar`) are in place, which is also how the launcher knows it is on a car.
- **The box:** `cd ~/FEB/feb-racing && ./feb setup`, once the arm64 image is published by CI.
  On a car the box gets `/dev` passed in, and `feb up` defaults to the `feb-stack` workspace.

- **First drive done 2026-10-18** (Jacobs, bench and floor): VESC detected in VESC Tool (FOC, Medium
  Outrunner preset, 3S 6000 mAh, motor direction inverted, App = No App, servo output on). Manual
  drive at the 0.8 m/s cap, steering full range. The start-up stutter is the sensorless dead zone.
- **Running it.** `cd ~/FEB/feb-racing && ./feb up --car --no-foxglove --driver feb_driver
  --mode-name inspection --ws ~/FEB/feb-racing/starter_kit`. The `--ws` points at the starter kit
  because `feb-stack`, the car's default workspace, holds no drivers until `feb submit` has run;
  after that the `--ws` goes away. Pad on before `feb up`.
- **Lidar yaw.** The S2 sits turned 90 degrees (cable out the side). `scan_rotate` in `feb_tools`
  re-labels the scan so `/scan`'s zero is the nose; the number is `lidar_yaw` in
  `config/car.yaml`. Check: a hand in front of the nose reads 0 in `/scan`.
- **Gamepad modes.** The 8BitDo Ultimate 2C has two layouts depending on its mode (usb id
  `2dc8:301c`: LB 6, RB 7, minus 10, steer axis 2; `2dc8:310a`: LB 4, RB 5, Back 6, steer axis 3).
  `car.yaml` holds the 301c layout under `supervisor:`. Do not press the X/B/A+Home combos. Turn the
  pad on *before* `feb up`; if it re-pairs while the stack runs, restart the stack.
- **Lidar view.** 315 degrees clear; the Jetson blocks a 45-degree slice straight behind the sensor.

Still to do, on the bench, wheels in the air:

1. **VESC Tool** on a laptop, VESC on USB, battery plugged in (the first time): update
   firmware, enable servo output and write it, load the guide's motor XML, run motor detection,
   then the guide's hysteresis and PID steps. The wheels spin during detection; hold the car.
2. **Pair the gamepad** dongle into the Jetson. `ls /dev/input/js0` shows it.
3. **First `feb up --car`:** `./feb up --car --driver feb_driver --mode-name inspection`.
   Watch the log for the VESC and lidar coming up, then `ros2 topic hz /scan` in `./feb shell`.
   LB held, the left stick turns the front wheels and the right trigger spins the rear wheels;
   let go and they stop. Then, and only then, the floor.

*Check:* on the stand, holding the dead-man, the car answers the gamepad and stops on release.

## First drive
A big empty floor, 1 m/s, the gamepad in your hand the whole time. Drive a few laps by hand. If it
runs backwards: swap the white and blue motor leads. If it steers the wrong way: the servo
direction flag in the driver config. Then `calibrate_camera` and sysid are the next chapters, in
their own documents.

## If you get stuck
Send a photo of the step and say which number you are on. There is nothing in this list that a
photo and five minutes cannot sort out.
