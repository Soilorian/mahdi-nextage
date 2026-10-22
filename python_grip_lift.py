"""GUI-only NEXTAGE left-hand close and 80 mm wrist-lift preview.

Load as a Choreonoid Python Script item and execute with AISTSimulator stopped.
Run setup_scene.py first to place the robot and open its fingers. This script
changes BodyItem joint positions directly; it does not simulate ball contact or
send commands to the physical robot.
"""

import time
import numpy as np
from cnoid.Base import RootItem, Timer
from cnoid.Body import getCustomJointPath
from cnoid.BodyPlugin import BodyItem


def smoothstep(value):
    """Ease a 0-to-1 motion so its beginning and end are less abrupt."""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class GripAndLift:
    def __init__(self, robot_item):
        self.item = robot_item
        self.body = robot_item.body
        if self.body.numJoints != 19:
            raise RuntimeError("Expected the supplied 19-joint NEXTAGE model")

        self.left = self.body.link("LHAND_L")
        self.right = self.body.link("LHAND_R")
        chest = self.body.link("CHEST_JOINT0")
        wrist = self.body.link("LARM_JOINT5")
        if not all((self.left, self.right, chest, wrist)):
            raise RuntimeError("The supplied left arm or finger links are missing")

        self.body.calcForwardKinematics()
        path = getCustomJointPath(self.body, chest, wrist)
        if path is None or path.numJoints == 0:
            raise RuntimeError("Could not make a left-arm joint path")
        self.arm_joints = [path.joint(i) for i in range(path.numJoints)]
        self.arm_start = [joint.q for joint in self.arm_joints]

        # This Choreonoid binding exposes Link.T as a 3x4 array, but the
        # JointPath IK binding accepts Position through a full 4x4 matrix.
        # Add the homogeneous last row [0, 0, 0, 1] for that conversion.
        wrist_pose = np.asarray(wrist.T, dtype=float)
        target = np.eye(4, dtype=float)
        if wrist_pose.shape == (3, 4):
            target[:3, :] = wrist_pose
        elif wrist_pose.shape == (4, 4):
            target[:, :] = wrist_pose
        else:
            raise RuntimeError("Unexpected wrist pose shape: " + str(wrist_pose.shape))
        # Keep orientation and X/Y position; add 0.080 m to world Z only.
        target[2, 3] += 0.080
        try:
            if not path.calcInverseKinematics(target):
                raise RuntimeError("The 80 mm upward wrist pose is unreachable")
            self.arm_goal = [joint.q for joint in self.arm_joints]
            for joint, goal in zip(self.arm_joints, self.arm_goal):
                if goal < joint.q_lower - 1e-6 or goal > joint.q_upper + 1e-6:
                    raise RuntimeError("The lift would exceed an arm joint limit")
        finally:
            # IK temporarily changes the model. Restore the start before motion.
            for joint, start in zip(self.arm_joints, self.arm_start):
                joint.q = start
            self.body.calcForwardKinematics()
            self.item.notifyKinematicStateChange()

        self.left_start = self.left.q
        self.right_start = self.right.q
        self.start_time = time.monotonic()
        self.timer = Timer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(40)  # Request a GUI update roughly every 40 ms.
        print("Python preview started: close 0-2 s; lift 2-4 s.")

    def tick(self):
        t = time.monotonic() - self.start_time
        closing = smoothstep(t / 2.0)
        lifting = smoothstep((t - 2.0) / 2.0)

        # Both finger sliders move toward 0 m from their current positions.
        self.left.q = self.left_start * (1.0 - closing)
        self.right.q = self.right_start * (1.0 - closing)
        for joint, start, goal in zip(self.arm_joints, self.arm_start, self.arm_goal):
            joint.q = start + lifting * (goal - start)

        self.body.calcForwardKinematics()
        self.item.notifyKinematicStateChange()
        if t >= 4.0:
            self.timer.stop()
            print("Python preview complete. It did not simulate or lift the ball.")

    def stop(self):
        self.timer.stop()


# Keep the Timer object alive after the Python Script item finishes executing.
# Re-executing this script stops the previous preview first.
if "ball_grip_lift_motion" in globals():
    ball_grip_lift_motion.stop()

matches = [item for item in RootItem.instance.getDescendantItems(BodyItem)
           if item.body.link("LHAND_L")]
if len(matches) != 1:
    raise RuntimeError("Load exactly one supplied NEXTAGE Body under World")
ball_grip_lift_motion = GripAndLift(matches[0])
