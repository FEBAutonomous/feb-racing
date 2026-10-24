# Manuals

- `racer.pdf`: `feb_racer`, the walls memory racer (map on a warm-up lap, raceline, pure pursuit): what it does, how to run it, what the scored runs taught.
- `lead.pdf`: the lead's car manual: power, every way to reach the car, running a driver, the pad, the debugging table from what went wrong.
- `member_guide.pdf`: release 2, the whole system for a member: the box, drivers, scoring and submitting, the dead-man, the car. Start here. Build: `tectonic member_guide.tex`.
- `competitor_setup_mac.pdf`, `competitor_setup_windows.pdf`, `competitor_setup_linux.pdf`: setup only, every step from an empty machine to the first autonomous lap, one per system. Shared body `setup_body.tex`, per-system install `setup_install_<os>.tex`, shared appendix `starter_driver.tex`. Build: `tectonic competitor_setup_<os>.tex`.
- `lab_ros_onboarding.pdf`: the ROS 2 onboarding lab (EECS C106A Labs 1 and 2 in our devkit), same content as `docs/lab-ros-onboarding.md`. Build: `tectonic lab_ros_onboarding.tex`.
- `organiser.pdf`: for the organiser, from tracks and events to race day, releases and maintenance.
- `devkit_v2_plan.pdf`: the design of the box, the development environment, and why it is not a VM.

All share `preamble.tex`; the organiser manual also inputs `platform.tex` (the platform and the rules). Rebuild:

```bash
docker run --rm -v "$PWD/../..:/work" -w /work/docs/manual texlive/texlive:latest-medium \
  bash -c 'for m in organiser competitor_setup_linux competitor_setup_mac competitor_setup_windows lab_ros_onboarding; do pdflatex $m.tex && pdflatex $m.tex; done; rm -f *.aux *.log *.out *.toc'
```
