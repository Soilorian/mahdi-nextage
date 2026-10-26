# NEXTAGE open, close, and lift demo

This directory has one motion program and one run setting. The program opens the left gripper, closes it, then moves the left wrist 5 cm upward. It does not detect, grab, or move any object.

The script follows the playback structure in `bpbot-lab/example/Fujimoto_with_knotfile.py`: it builds motion rows once, previews them in Choreonoid when `PLAY=0`, and sends them through bpbot's `NxtRobot.playMotion()` when `PLAY=1`. The same model and inverse-kinematics calculation generate both modes' arm angles.

| File | Purpose |
|---|---|
| `run_nextage.py` | Plans and plays the open, close, lift sequence |
| `run_nextage.json` | Selects `PLAY=0` or `PLAY=1` |
| `env.sh.example` | Template for starting the lab Choreonoid environment |

The model and project are already in the lab clones:
- Model: `/home/mahdi/nextage/choreonoid-1.7.0-lab/ext/graspPlugin/RobotModels/HIRO/HIRO_dualarm.yaml`
- Project: `/home/mahdi/nextage/choreonoid-1.7.0-lab/ext/graspPlugin/BinPicking/project/waypoint_demo.cnoid`
- Setup guide: `/home/mahdi/nextage/bpbot-lab/docs/new_pc_setup.en.md`
- Real-robot procedure: `/home/mahdi/nextage/bpbot-lab/docs/manual_lab_pc.md`

## Select preview or hardware playback

Open `/home/mahdi/nextage/mahdi-nextage/NextageManual/run_nextage.json`. Initially it contains:

```json
{
  "PLAY": 0
}
```

`PLAY` is the program's mode value. `0` means preview the model only, with no bpbot robot connection. `1` means send the motion to the real robot through bpbot. The file must contain exactly an integer `0` or `1`; another value stops execution. This is a separate setting from bpbot's `config/run_local.yaml`, so changing it will not change Fujimoto's experiments. There are no command-line flags.

## Preview with PLAY=0

1. Follow the lab setup guide to install the required environment and build Choreonoid. This WSL checkout has the source but no built lab Choreonoid GUI, so GUI execution could not be checked here.
2. In an Ubuntu terminal run:

```bash
source /home/mahdi/nextage/env.sh
cd /home/mahdi/nextage/choreonoid-1.7.0-lab
cnoid ext/graspPlugin/BinPicking/project/waypoint_demo.cnoid
```

`source` loads the Python and plugin environment into this terminal. `cd` selects the Choreonoid checkout. `cnoid` is the launcher defined in `env.sh`; its argument is the project file that loads the HIRO model.

3. The project includes a different Python Script item named `waypoint_demo.py`. Untick it. In Choreonoid select World, use **File → Load → Python Script**, and choose `/home/mahdi/nextage/mahdi-nextage/NextageManual/run_nextage.py`. Select and tick the new item, then press the red **Execute script** button. The Message view shows `PLAY=0`, the planned wrist position and target, and the preview progress. You may save the Choreonoid project to remember this script item.

A successful preview animates the model's two sliding finger joints and then the left arm. It is a kinematic preview: it does not simulate physical contact or check collisions. The model's starting pose is whatever is displayed in Choreonoid when the script begins.

## Hardware with PLAY=1

Use the lab's robot-startup procedure first. The bpbot robot server must already be running on the lab PC; the script uses `NxtRobot(host='[::]:15005')`, exactly as Fujimoto does. That host string addresses bpbot's gRPC server, whose port is `15005`. It is distinct from the robot controller's OpenRTM name service, which the server itself contacts.

Change only `PLAY` in `run_nextage.json` to `1`. Run the **same Choreonoid Python Script item** on the lab PC. The script first calls bpbot's `getJointAngles()`, places those reported command angles into the Choreonoid model, plans all motion rows, then calls `playMotion()`. It does not use `goInitial()`: the lift is relative to the robot's reported starting command pose. `getJointAngles()` is backed by `getActualState().command` in this server, so it does not independently measure encoder position.

Each row is `[seconds, right-hand command, left-hand command, joint angles in degrees]`. The open row uses left-hand command `3`; the close row uses `4`; arm rows use `0`. The Choreonoid preview reads the joint columns. On hardware, bpbot's `playMotion()` reads the hand-command columns and sends joint targets through its robot server. The open and close rows each last one second. Fifty arm rows of 0.1 seconds make the five-second lift.

The script reads the bpbot command state again after playback and compares its left-arm values with the planned final row. This detects a command-state mismatch, but cannot prove physical motion or gripper actuation. Check that the robot is ready and its upward path is clear before `PLAY=1`; this example does not enable servos or perform collision checks.

The Choreonoid Python Script item in this version does not expose a per-run argument field. The JSON file provides the `PLAY` setting without changing the launch command.

Both modes were exercised only with fake Choreonoid and bpbot objects in this workspace. Neither the Choreonoid GUI nor the real robot has been run here.
