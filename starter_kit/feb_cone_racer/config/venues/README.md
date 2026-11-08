# Venue profiles

One file per place the car drives, applied on top of `racer.yaml` and `racer.car.yaml` with
`./feb up --car --driver feb_cone_racer --profile NAME`. A profile holds only what changes with the
place: the colour ranges under that lighting, the camera lock, sometimes the image band.

Make one on the dash's Tune page (`http://feb-car:8780/tune` while `feb up --car` runs): place a
blue and a yellow cone two metres ahead, drag the sliders until each cone lights up in its own
colour and nothing else does, lock the camera, press Save with the venue's name. The file lands
here; commit it so the team has it. The procedure is in `docs/devkit.md`, "Calibrating at a new venue".
