# NEXTAGE first-run motion demo

This directory contains one control script, its mode setting, and an environment template. The demonstration has no object target: it opens the left gripper, closes it, then raises the left wrist by 5 cm from its reported or modeled starting position. It does not approach, hold, or move a ball.

| File | Purpose |
|---|---|
| `run_nextage.py` | One Python entry point for Choreonoid preview and real hardware |
| `run_nextage.json` | Selects `simulation` or `real`; initially `simulation` |
| `env.sh.example` | Template for the Choreonoid planning environment |

The robot model and project remain in the lab clones:
- Model: `/home/mahdi/nextage/choreonoid-1.7.0-lab/ext/graspPlugin/RobotModels/HIRO/HIRO_dualarm.yaml`
- Project: `/home/mahdi/nextage/choreonoid-1.7.0-lab/ext/graspPlugin/BinPicking/project/waypoint_demo.cnoid`
- Installation guide: `/home/mahdi/nextage/bpbot-lab/docs/new_pc_setup.en.md`
- Lab hardware procedure: `/home/mahdi/nextage/bpbot-lab/docs/manual_lab_pc.md`

Both lab clones must be on the `guides` branch. To inspect one, run `git -C /home/mahdi/nextage/bpbot-lab symbolic-ref --short HEAD`. `-C` selects that repository, `symbolic-ref` reads its checked-out branch reference, and `--short` prints just the branch name. Repeat with the Choreonoid clone path.

## Select a mode

Open `/home/mahdi/nextage/mahdi-nextage/NextageManual/run_nextage.json`. It initially contains:

```json
{
  "mode": "simulation"
}
```

The key `mode` chooses one branch of `run_nextage.py`. Keep `simulation` while using Choreonoid. Change only the value to `real` for hardware; there are no command-line flags. A missing or misspelled value stops the script before any robot command.

## Preview in Choreonoid

1. Follow `bpbot-lab/docs/new_pc_setup.en.md` to build the lab Choreonoid and environment. The source clone alone cannot run. In this Ubuntu WSL checkout, the built lab executable and its Python environment were not available for a GUI test.
2. In an Ubuntu terminal run:

```bash
source /home/mahdi/nextage/env.sh
cd /home/mahdi/nextage/choreonoid-1.7.0-lab
cnoid ext/graspPlugin/BinPicking/project/waypoint_demo.cnoid
```

`source` loads the environment into this terminal. `cd` changes to the Choreonoid checkout so the project path resolves. `cnoid` is the launcher defined by `env.sh`; the argument is the project file that loads the HIRO model. The project also contains a different, waypoint-demo Python Script item; untick that item so it cannot execute when you test the new script.

3. In Choreonoid, select the World item, choose **File → Load → Python Script**, and select `/home/mahdi/nextage/mahdi-nextage/NextageManual/run_nextage.py`. Select and tick the new Python Script item, then press the red **Execute script** button. Watch the Message view for the logged opening, closing, IK planning, lift preview, and completion. Save the project from Choreonoid if you want it to remember the added script item.

The simulation mode changes the model's `LHAND_L` and `LHAND_R` sliding joints from open positions (`+0.025` and `-0.025` metres) to closed (`0` and `0` metres). It reads the displayed `LARM_JOINT5` wrist pose, plans a 5 cm upward path through 50 inverse-kinematics steps, and animates that path over 5 seconds. It does not contact the robot or run physics or collision checks. A successful preview shows that the model's inverse kinematics found this path from its current pose; it does not establish that the real workspace is clear.

## Run on the physical NEXTAGE

After the lab's startup procedure has enabled the robot and you have checked the arm has a clear upward path, change `mode` in `run_nextage.json` to `real`. On the configured robot-control PC run:

```bash
/usr/bin/python2 /home/mahdi/nextage/mahdi-nextage/NextageManual/run_nextage.py
```

`/usr/bin/python2` selects the lab's Python 2 interpreter. The path names this script. Real mode imports `hrpsys` and `nextage_ros_bridge`, connects to the OpenRTM name service at `192.168.128.10:15005` and `RobotHardware0`, opens and closes the left pneumatic gripper, reads the reported `LARM_JOINT5` position and orientation, and requests the same 5 cm Z lift over 5 seconds. It waits for left-arm interpolation to finish. This script does not enable servos, home the robot, check for obstacles, or verify that an object was grasped. Hardware mode has only been checked with a fake client here; it has not been run on the physical robot.

The Choreonoid Python Script item does not have a per-run argument field in this version, which is why this example uses `run_nextage.json`. For other experiments, you can read more keys from a config file in the same way.
