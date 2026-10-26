#!/usr/bin/env python
"""Open and close the left gripper, then lift the left wrist 5 cm.

Set "mode" in run_nextage.json to "simulation" for a Choreonoid model preview,
or "real" for the physical robot. No command-line flags are required.
"""
from __future__ import print_function
import json
import logging
import os
import sys
import time


log = logging.getLogger('run_nextage')
log.setLevel(logging.DEBUG)
if not log.handlers:
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    log.addHandler(handler)

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'run_nextage.json')
LIFT_METRES = 0.05
LIFT_SECONDS = 5.0


def load_mode():
    with open(CONFIG_FILE, 'r') as config_file:
        config = json.load(config_file)
    mode = config.get('mode')
    if mode not in ('simulation', 'real'):
        raise ValueError('%s must set mode to "simulation" or "real"' % CONFIG_FILE)
    return mode


def run_simulation():
    import numpy as np
    from cnoid.Base import RootItem, MessageView
    from cnoid.Body import getCustomJointPath
    from cnoid.BodyPlugin import BodyItem

    items = RootItem.instance.getDescendantItems(BodyItem)
    robots = [item for item in items
              if item.body.link('LARM_JOINT5') is not None
              and item.body.link('LHAND_L') is not None
              and item.body.link('LHAND_R') is not None]
    if len(robots) != 1:
        raise RuntimeError('Load exactly one HIRO_dualarm robot BodyItem; found %d'
                           % len(robots))
    robot_item = robots[0]
    body = robot_item.body
    left_finger = body.link('LHAND_L')
    right_finger = body.link('LHAND_R')
    wrist = body.link('LARM_JOINT5')
    chest = body.link('CHEST_JOINT0')
    if chest is None:
        raise RuntimeError('The loaded robot lacks CHEST_JOINT0')
    log.info('Simulation: using robot BodyItem %s', robot_item.name)

    # The two modeled fingers slide in metres: +/-0.025 is open; zero is closed.
    def move_fingers(left_target, right_target, description):
        left_start, right_start = left_finger.q, right_finger.q
        log.info('%s modeled left gripper over 1 second', description)
        for frame in range(1, 11):
            fraction = frame / 10.0
            left_finger.q = left_start + (left_target - left_start) * fraction
            right_finger.q = right_start + (right_target - right_start) * fraction
            body.calcForwardKinematics()
            robot_item.notifyKinematicStateChange()
            MessageView.instance.flush()
            time.sleep(0.1)

    log.info('Modeled finger positions [m]: left=%s, right=%s',
             left_finger.q, right_finger.q)
    move_fingers(0.025, -0.025, 'Opening')
    move_fingers(0.0, 0.0, 'Closing')

    # A 4x4 target is required by the Choreonoid Position binding.
    body.calcForwardKinematics()
    start_position = np.array(wrist.p, copy=True)
    start_rotation = np.array(wrist.R, copy=True)
    log.info('Modeled wrist position [m]: %s', start_position.tolist())
    log.info('Requested wrist target [m]: %s',
             (start_position + np.array([0.0, 0.0, LIFT_METRES])).tolist())
    path = getCustomJointPath(body, chest, wrist)
    if path is None or path.numJoints != 6:
        raise RuntimeError('Could not make a six-joint left-arm IK path')
    start_angles = [path.joint(i).q for i in range(6)]
    planned_angles = []

    # Plan the whole lift first so an unreachable target does not produce a
    # partial animated arm move.
    try:
        for frame in range(1, 51):
            target = np.eye(4)
            target[:3, :3] = start_rotation
            target[:3, 3] = start_position
            target[2, 3] += LIFT_METRES * frame / 50.0
            if not path.calcInverseKinematics(target):
                raise RuntimeError('Left-arm IK failed at lift step %d/50' % frame)
            body.calcForwardKinematics()
            planned_angles.append([path.joint(i).q for i in range(6)])
    finally:
        for i, angle in enumerate(start_angles):
            path.joint(i).q = angle
        body.calcForwardKinematics()
        robot_item.notifyKinematicStateChange()

    log.info('IK planned 50 lift steps; previewing over 5 seconds')
    for frame, angles in enumerate(planned_angles, 1):
        for i, angle in enumerate(angles):
            path.joint(i).q = angle
        body.calcForwardKinematics()
        robot_item.notifyKinematicStateChange()
        MessageView.instance.flush()
        time.sleep(LIFT_SECONDS / 50.0)
        if frame % 10 == 0:
            log.debug('Lift preview: %d/50 steps', frame)
    log.info('Modeled wrist ended at [m]: %s', list(wrist.p))
    log.info('Choreonoid kinematic preview completed')


def run_real():
    from hrpsys import rtm
    from nextage_ros_bridge import nextage_client

    # OpenRTM name service and robot name from bpbot's nxtrobot_server.py.
    rtm.nshost = '192.168.128.10'
    rtm.nsport = '15005'
    log.info('Connecting to OpenRTM name service %s:%s', rtm.nshost, rtm.nsport)
    robot = nextage_client.NextageClient()
    log.debug('Initializing NEXTAGE client for RobotHardware0')
    robot.init(robotname='RobotHardware0', url='')
    log.info('NEXTAGE client initialized')

    # The physical left hand uses pneumatic commands, not model finger q.
    log.info('Opening physical left gripper')
    robot.gripper_l_open()
    log.debug('Open command returned; waiting 1 second for the hand')
    time.sleep(1.0)
    log.info('Closing physical left gripper')
    robot.gripper_l_close()
    log.debug('Close command returned; waiting 1 second for the hand')
    time.sleep(1.0)

    log.debug('Reading reported LARM_JOINT5 position and orientation')
    position = list(robot.getCurrentPosition('LARM_JOINT5'))
    orientation = list(robot.getCurrentRPY('LARM_JOINT5'))
    log.info('Reported wrist position [m]: %s', position)
    log.info('Reported wrist orientation [rad]: %s', orientation)
    position[2] += LIFT_METRES
    log.info('Requested wrist target [m]: %s (Z increased by 0.05 m)', position)
    log.info('Requesting 5-second left-arm lift')
    accepted = robot.setTargetPose('larm', position, orientation, LIFT_SECONDS)
    log.debug('setTargetPose returned: %r', accepted)
    if not accepted:
        raise RuntimeError('NEXTAGE rejected the left-hand lift target')
    log.info('Waiting for left-arm interpolation to finish')
    robot.waitInterpolationOfGroup('larm')
    log.info('Physical left gripper open, close, and 5 cm lift request completed')


def main():
    mode = load_mode()
    log.info('Selected mode: %s', mode)
    if mode == 'simulation':
        run_simulation()
    else:
        run_real()


if __name__ == '__main__':
    try:
        main()
    except Exception:
        log.exception('NEXTAGE command sequence failed')
        raise
