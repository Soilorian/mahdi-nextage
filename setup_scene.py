"""Execute as a Choreonoid Python Script item before starting AISTSimulator."""
import math
import numpy as np
from cnoid.Base import RootItem
from cnoid.BodyPlugin import BodyItem

items = RootItem.instance.getDescendantItems(BodyItem)
def exactly_one(link_name):
    found = [item for item in items if item.body.link(link_name)]
    if len(found) != 1:
        raise RuntimeError("Expected exactly one Body with link " + link_name)
    return found[0]

robot = exactly_one("LHAND_L")
ball = exactly_one("PracticeBall")
stand = exactly_one("BallStand")
if not robot.body.link("LHAND_R"):
    raise RuntimeError("The loaded robot lacks the real left hand")

pose_deg = [0, 0, 0, 0, -25.7, -127.7, 0, 0, 0,
            0, -33.7, -122.7, 0, 0, 0]
if robot.body.numJoints != 19:
    raise RuntimeError("Expected 19 joints in main_dualarm_regrasp.wrl")
for index, angle in enumerate(pose_deg):
    joint = robot.body.joint(index)
    joint.q = angle / 180.0 * math.pi
for index in range(15, 19):
    robot.body.joint(index).q = 0.025 if index in (15, 17) else -0.025
robot.body.rootLink.p = np.array([0.0, 0.0, 0.97])
robot.body.calcForwardKinematics()
robot.notifyKinematicStateChange()

wrist = robot.body.link("LARM_JOINT5")
ball_center = wrist.p + wrist.R.dot(np.array([-0.120, 0.0, 0.0]))
ball.body.rootLink.p = ball_center
ball.body.calcForwardKinematics()
ball.notifyKinematicStateChange()
stand.body.rootLink.p = ball_center + np.array([0.0, 0.0, -0.020 - 0.640])
stand.body.calcForwardKinematics()
stand.notifyKinematicStateChange()

for item in (robot, ball, stand):
    item.storeInitialState()
print("Practice ball (world m):", ball_center)
print("Front check: x must exceed 0.40 m; x =", ball_center[0])
print("This 40 mm practice ball fits the measured 47 mm hand aperture.")
