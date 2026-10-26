#!/usr/bin/env python2
"""Close the real NEXTAGE left gripper, then lift the left wrist 5 cm.

Run this file with Python 2 on the configured robot-control PC. It requires the
installed nextage_ros_bridge/hrpsys runtime but imports no other project file.
"""
from __future__ import print_function
import time
from hrpsys import rtm
from nextage_ros_bridge import nextage_client


def main():
    # OpenRTM name service and robot name from bpbot's nxtrobot_server.py.
    rtm.nshost = '192.168.128.10'
    rtm.nsport = '15005'
    robot = nextage_client.NextageClient()
    robot.init(robotname='RobotHardware0', url='')

    # The real hand is pneumatic; the model's finger joints are not commands.
    robot.gripper_l_close()
    time.sleep(1.0)

    # Read the current left-wrist pose, keep X/Y and RPY, and increase Z.
    position = list(robot.getCurrentPosition('LARM_JOINT5'))
    orientation = list(robot.getCurrentRPY('LARM_JOINT5'))
    position[2] += 0.05  # metres = 5 cm

    # The NEXTAGE client solves IK for the target and moves over 5 seconds.
    if not robot.setTargetPose('larm', position, orientation, 5.0):
        raise RuntimeError('NEXTAGE rejected the left-hand lift target')
    robot.waitInterpolationOfGroup('larm')
    print('Left gripper close and 5 cm lift requested and completed.')


if __name__ == '__main__':
    main()
