For the first run with bpbot-lab and choreonoid-1.7.0-lab, start with [NextageManual/README.md](NextageManual/README.md).

# Real-model ball pickup: Choreonoid 1.7 / Ubuntu 18.04

This scene uses the **unchanged** `/home/mahdi/nextage/model/main_dualarm_regrasp.wrl` robot and its actual `LHAND_L` / `LHAND_R` sliding fingers. It is an offline physics simulation. The robot faces world **+X**. The ball center starts around `(0.50, 0.23, 1.31)` metres: in front of the torso, in the left arm's reachable lane.

## Why this is a 40 mm practice ball

`../model_probe.cpp`, run against the supplied model, measured the open collision-mesh bounds in wrist coordinates. The left finger's inner edge is at about `y=+0.02384 m`, and the right finger's inner edge at about `y=-0.02360 m`. The clear aperture is therefore approximately `0.04744 m` or **47.4 mm**. A regulation baseball is approximately 73–75 mm in diameter. It cannot pass between these fingers. Enlarging the simulated joint travel would silently change the actual robot. This demo uses a 40 mm sphere, 23 g mass, and solid-sphere inertia `2mr²/5 = 0.00000368 kg·m²`. It is **not** a baseball. Do not plan a real-baseball grasp until you have designed and measured a wider physical hand.

## Files and what they do

- `PracticeBall.body`: a freely moving (`jointType: free`) 40 mm sphere. Its mass is near the value obtained by scaling the earlier baseball's mass by the ratio of volumes. The pale color is just a visual property.
- `BallStand.body`: a fixed (`jointType: fixed`) 35 mm square pedestal, 1.28 m high. The script sets its top 20 mm below the ball center, so it supports the ball at its bottom.
- `setup_scene.py`: applies the model YAML's standard arm angles, opens the left fingers to their real joint limits, raises the fixed robot base to 0.97 m, computes the ball location from the left wrist transform, moves the stand beneath it, and saves the initial states.
- `RealModelBallController.cpp`: `grip-only` closes the two real sliders; `pickup` closes and then lifts the wrist by 80 mm using arm inverse kinematics. The fingers receive capped forces; the arm receives position targets. Nothing attaches the ball to the hand.
- `materials.yaml`: contact friction. All these untagged bodies use Choreonoid's `Default` material, so the `Default`–`Default` pair is the relevant row.
- `build.sh`: builds the controller shared library `RealModelBallController.so` for this installed Choreonoid 1.7 build.

## Reproduce in the Ubuntu terminal

The files have already been copied to `/home/mahdi/nextage/RealModelBall`. To resync after editing the Windows workspace, execute these commands **one line at a time**:

```bash
mkdir -p /home/mahdi/nextage/RealModelBall
cp /mnt/c/Users/m1382/OneDrive/Desktop/MOGDH/NextStage/demo/RealModelBall/* /home/mahdi/nextage/RealModelBall/
bash /home/mahdi/nextage/RealModelBall/build.sh
```

`mkdir` creates the directory; `-p` also creates missing parents and tolerates an existing directory. `cp` copies files; `*` matches the files directly in that source folder, and the last path is the destination. `bash` executes the build script. In it, `g++` compiles C++; `-std=c++14` selects the C++ version; `-fPIC` produces position-independent code for a library; `-shared` builds a `.so` rather than a standalone program; each `-I` adds a header search directory; `-L` adds a library search directory; `-Wl,-rpath,...` embeds Choreonoid's runtime library path; `-o` names the output; and each `-l:...` selects the exact installed library file. If the third command prints a C++ error, stop before opening Choreonoid and copy the first error line.

## Assemble and verify the scene

1. Open Choreonoid and create a fresh World. Load `/home/mahdi/nextage/model/main_dualarm_regrasp.wrl` as a Body child of World. Verify **Joint Sliders** show `LHAND_L` and `LHAND_R`. This is the supplied model, not the previous generated `main_with_gripper.wrl`.
2. Load `/home/mahdi/nextage/RealModelBall/PracticeBall.body` and `/home/mahdi/nextage/RealModelBall/BallStand.body` as two separate Body children of the same World. Optionally load the sample floor at `/home/mahdi/nextage/choreonoid-1.7.0-main/choreonoid-1.7.0-main/share/model/misc/floor.body` as another Body child.
3. Select World and set its **Material table** property to `/home/mahdi/nextage/RealModelBall/materials.yaml`. Press Enter so the property is committed. Create an **AISTSimulator** item below World. Set the Time bar's internal frame rate to **1000 frames/s**, giving one physics step every 1 ms (`1/1000 s`). The 1.5 static and 1.2 dynamic friction values are simulation assumptions, not measured finger-pad data.
4. Select the robot Body and add **SimpleController** as its child. Set **Controller module** to `/home/mahdi/nextage/RealModelBall/RealModelBallController.so`. Enter exactly `grip-only` in **Controller options** for the first run.
5. Add a **Python Script** item under World and choose `/home/mahdi/nextage/RealModelBall/setup_scene.py`. Execute that script **once**. It sets positions and finishes; it does not start physics. Check its Message view line: ball x should be greater than `0.40 m`. Visually check the sphere is between the open fingers and rests on the stand. The script calls `storeInitialState()` for all three Bodies; you can also select World and press **Simulation initial status setting**.
6. Select **AISTSimulator** and press the green **Start simulation from the beginning** button, not the Python Script execute button. For `grip-only`, watch the fingers move from `+0.025/-0.025 m` toward zero over roughly 0.25–2.0 s. Stop around 2.2 s and inspect the controller's reported finger positions and the ball's Body/Link position.
7. Return to the initial status. Set the controller option to `pickup`, then start AIST from the beginning. The arm should rise 80 mm over roughly 2.4–4.4 s while the fingers remain in contact. Check the ball's **world Z** position before and after; if it stays at its stand height, the ball was not lifted even if the arm moved. Save the project as a `.cnoid` file only after the grip-only and pickup checkpoints look right.

The item tree should be:

```text
World
├── main_dualarm_regrasp (Body)
│   └── SimpleController
├── PracticeBall (Body)
├── BallStand (Body)
├── Floor (Body, optional)
├── AISTSimulator
└── setup_scene.py (Python Script)
```

## What to check if it fails

- **Controller initialization failed:** inspect the Message view line immediately before the generic failure. Wrong model means missing `LHAND_L/R`; wrong option means use exactly `grip-only` or `pickup`; failed IK means rerun the setup script, save the initial state, then restart.
- **Ball falls before closing:** check its center Z and stand top. The stand top must be ball center Z minus `0.020 m`. Do not move only the visual sphere; use the Body item.
- **Ball shoots away on contact:** pause near first contact. Check the ball's world position and both finger joint displacements. Reduce `forceCap` in the C++ controller or reduce the physics time step if the contact becomes unstable. `forceCap` is 2.0 N per finger in this simulation. After changing C++, rerun `build.sh` and restart Choreonoid so the `.so` is reloaded.
- **Ball remains on stand during lift:** compare finger positions at 2.2 s, inspect contact display, then inspect the ball's Z after 4.4 s. A hand that closes to the zero limit without touching the sphere has an alignment issue. Check `setup_scene.py` ran *after* all Body items were loaded.
- **The model seems side-facing:** the world front direction in this model is +X. The setup prints the ball's world coordinate. The left arm lane has positive Y; this does not mean the object is beside the robot.

`setup_scene.py` uses radians for rotary joints, metres for sliding joints and positions, and NumPy's `wrist.R.dot(local_offset)` to convert the `[-0.120,0,0] m` gripper-center offset from wrist coordinates into world coordinates. It stores the scene after placement so AIST starts from what you inspected. The controller's `blend(a,b,t)` smoothly interpolates from 0 to 1 between times `a` and `b`; `JointPath::calcInverseKinematics` finds arm joint angles for a wrist pose 80 mm higher. The left finger's target goes from +25 mm to zero; the right goes from −25 mm to zero. Their forces are a 200 N/m position-error term minus a 5 N·s/m speed-damping term, clamped to ±2 N. These are demo gains, not real-hand commands.
