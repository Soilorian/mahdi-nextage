# NEXTAGE first run

This is the small first-run bundle for `/home/mahdi/nextage/mahdi-nextage`. It contains **one hardware Python file** and one small planning-environment template. The robot model, Choreonoid project, and planning example already live in the two lab clones; duplicating them here would make it harder to know which copy is active.

| Need | Existing location |
|---|---|
| Full installation guide | `/home/mahdi/nextage/bpbot-lab/docs/new_pc_setup.en.md` |
| Simulation model | `/home/mahdi/nextage/choreonoid-1.7.0-lab/ext/graspPlugin/RobotModels/HIRO/HIRO_dualarm.yaml` |
| Simulation project | `/home/mahdi/nextage/choreonoid-1.7.0-lab/ext/graspPlugin/BinPicking/project/waypoint_demo.cnoid` |
| Its Python motion program | `/home/mahdi/nextage/bpbot-lab/example/waypoint_demo.py` |
| Real-robot lab procedure | `/home/mahdi/nextage/bpbot-lab/docs/manual_lab_pc.md` |
| One-file direct hardware example | `run_nextage.py` in this directory |
| Planning launcher template for this clone layout | `env.sh.example` in this directory |

The two lab repositories need their **`guides`** branches. Check with `git -C /home/mahdi/nextage/bpbot-lab branch --show-current` and the same command for `choreonoid-1.7.0-lab`; `-C` tells Git which repository to inspect, and `--show-current` prints its active branch. If a clean clone is on another branch, run `git -C PATH checkout guides` for that clone. The Choreonoid clone's `ext/bpbot` link expects a sibling directory called `bpbot`; this checkout has `/home/mahdi/nextage/bpbot -> bpbot-lab`. On another checkout, create that alias with `cd /home/mahdi/nextage && ln -s bpbot-lab bpbot` only if `bpbot` does not already exist. `ln -s` creates a symbolic link, and the last two arguments are its target and name.

## First execution: simulation

1. Finish the installation and build steps in `bpbot-lab/docs/new_pc_setup.en.md`, sections 4-7. Adapt its example folder name `~/fujimoto` to `/home/mahdi/nextage`, and use the existing `bpbot-lab` and `choreonoid-1.7.0-lab` clones. Create a Python 3.8 conda environment named `nextage`, or edit `MY_CONDA_ENV` in this directory's `env.sh.example` to match your environment name. With that environment active, configure and build `choreonoid-1.7.0-lab`. Then copy the template with `cp /home/mahdi/nextage/mahdi-nextage/NextageManual/env.sh.example /home/mahdi/nextage/env.sh`; `cp` copies the named file to the chosen destination. The source clone alone cannot run. In this Ubuntu 18 WSL installation, conda and the lab Choreonoid binary were still missing when this file was written.
2. Create `/home/mahdi/nextage/bpbot-lab/config/run_local.yaml` from `config/run_local.example.yaml` if it does not exist. Set `PLAY: 0`. This tells the waypoint example to preview without sending to the physical robot. `waypoint_demo.py` reads only this setting; it does not need a camera.
3. In a terminal, run:

```bash
source /home/mahdi/nextage/env.sh
cd /home/mahdi/nextage/choreonoid-1.7.0-lab
cnoid ext/graspPlugin/BinPicking/project/waypoint_demo.cnoid
```

`source` activates your configured planning environment in that terminal. `cd` selects the Choreonoid checkout. `cnoid` is the launcher function defined by that environment; its argument is the project that loads the model, table, and `waypoint_demo.py`. In Choreonoid, select/tick the **python script** item and press the **red Execute script** button. The Message view should end with `Waypoint demo finished.` Change the `WAYPOINTS` list in `bpbot-lab/example/waypoint_demo.py` to choose other targets. Keep `PLAY: 0` during simulation.

A Choreonoid Python Script item can run a file with `main()` and import other Python files. It executes the file as `__main__` and adds that file's directory to Python's import path for the run. Put a sibling module next to it and write `from helper import function_name`; then call `main()` under `if __name__ == '__main__':`. A function definition alone does not run. Restart Choreonoid if an edited imported module appears cached. Do **not** load this directory's `run_nextage.py` in Choreonoid: it commands real hardware.

## Later: first physical close and lift

`run_nextage.py` is a standalone Python 2 entry point with no flags and no imports from this repository. On the configured robot-control PC, after the lab's startup procedure has made the robot ready and the left hand is positioned around an object with a clear upward path, run:

```bash
/usr/bin/python2 /home/mahdi/nextage/mahdi-nextage/NextageManual/run_nextage.py
```

`/usr/bin/python2` selects the lab's system Python 2 interpreter. The path is the script to execute. It connects to the OpenRTM name service and `RobotHardware0` configured in bpbot's server, closes the left pneumatic gripper, waits one second, reads the current `LARM_JOINT5` position/orientation, increases its Z by `0.05` metres (5 cm), and asks `setTargetPose` to move over `5.0` seconds. It does not find or approach a ball, turn on servos, or verify a grasp. An accepted pose is not a collision check. The configured lab ROS/OpenRTM modules (`hrpsys` and `nextage_ros_bridge`) and the robot network are required; they are absent from this Ubuntu WSL installation. This file was syntax-checked and run only against a fake client, never against the real robot.

For how to start and stop the robot on the lab PC, follow `bpbot-lab/docs/manual_lab_pc.md`. Its `startrobot` command includes `goInitial()`, which moves the robot. Its `stoprobot` CLI branch calls `goOffPose()` but does not explicitly call `servoOff()` in the checked source, so verify the controller state rather than assuming the servos are off.
