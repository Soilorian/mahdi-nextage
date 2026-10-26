#!/usr/bin/env python2
"""Close the real NEXTAGE left gripper, then lift the left wrist 5 cm.

Run this file with Python 2 on the configured robot-control PC. It requires the
installed nextage_ros_bridge/hrpsys runtime but imports no other project file.
"""
from __future__ import print_function
import logging
import sys
import time
from hrpsys import rtm
from nextage_ros_bridge import nextage_client


log = logging.getLogger('run_nextage')
log.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
log.addHandler(handler)


def main():
    # OpenRTM name service and robot name from bpbot's nxtrobot_server.py.
    rtm.nshost = '192.168.128.10'
    rtm.nsport = '15005'
    log.info('Connecting to OpenRTM name service %s:%s', rtm.nshost, rtm.nsport)
    robot = nextage_client.NextageClient()
    log.debug('Initializing NEXTAGE client for RobotHardware0')
    robot.init(robotname='RobotHardware0', url='')
    log.info('NEXTAGE client initialized')

    # The real hand is pneumatic; the model's finger joints are not commands.
    log.info('Closing left gripper')
    robot.gripper_l_close()
    log.debug('Close command returned; waiting 1 second for the hand')
    time.sleep(1.0)

    # Read the reported left-wrist pose, keep X/Y and RPY, and increase Z.
    log.debug('Reading current LARM_JOINT5 position and orientation')
    position = list(robot.getCurrentPosition('LARM_JOINT5'))
    orientation = list(robot.getCurrentRPY('LARM_JOINT5'))
    log.info('Reported wrist position [m]: %s', position)
    log.info('Reported wrist orientation [rad]: %s', orientation)
    position[2] += 0.05  # metres = 5 cm
    log.info('Requested wrist target [m]: %s (Z increased by 0.05 m)', position)

    # The NEXTAGE client solves IK for the target and moves over 5 seconds.
    log.info('Requesting 5-second left-arm lift')
    accepted = robot.setTargetPose('larm', position, orientation, 5.0)
    log.debug('setTargetPose returned: %r', accepted)
    if not accepted:
        raise RuntimeError('NEXTAGE rejected the left-hand lift target')
    log.info('Waiting for left-arm interpolation to finish')
    robot.waitInterpolationOfGroup('larm')
    log.info('Left gripper close and 5 cm lift request completed')


if __name__ == '__main__':
    try:
        main()
    except Exception:
        log.exception('NEXTAGE command sequence failed')
        raise
