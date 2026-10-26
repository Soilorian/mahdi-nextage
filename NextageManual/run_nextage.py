#!/usr/bin/env python3
"""Preview or play one NEXTAGE motion: open left hand, close it, lift 5 cm.

Set PLAY to 0 or 1 in run_nextage.json. Run this file as a Choreonoid Python
Script item with the HIRO_dualarm model loaded. PLAY=1 uses bpbot's NxtRobot
client, the same hardware interface used by Fujimoto_with_knotfile.py.
"""
import json
import logging
import math
import os
import sys
import time

import numpy as np
from cnoid.Base import MessageView, RootItem
from cnoid.Body import getCustomJointPath
from cnoid.BodyPlugin import BodyItem


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
LIFT_STEPS = 50
OPEN_LEFT = 3
CLOSE_LEFT = 4


def read_play():
    with open(CONFIG_FILE, 'r') as config_file:
        config = json.load(config_file)
    play = config.get('PLAY')
    if type(play) is not int or play not in (0, 1):
        raise ValueError('%s must set PLAY to 0 or 1' % CONFIG_FILE)
    return play


def get_robot_item():
    items = RootItem.instance.getDescendantItems(BodyItem)
    robots = [item for item in items
              if item.body.link('LARM_JOINT5') is not None
              and item.body.link('LHAND_L') is not None
              and item.body.link('LHAND_R') is not None]
    if len(robots) != 1:
        raise RuntimeError('Load exactly one HIRO_dualarm robot BodyItem; found %d'
                           % len(robots))
    if robots[0].body.numJoints != 19:
        raise RuntimeError('The bpbot motion interface expects 19 modeled joints')
    return robots[0]


def current_joints(body):
    return [body.joint(i).q for i in range(body.numJoints)]


def set_joints(robot_item, angles):
    body = robot_item.body
    for i, angle in enumerate(angles):
        body.joint(i).q = float(angle)
    body.calcForwardKinematics()
    robot_item.notifyKinematicStateChange()


def motion_row(body, seconds, left_command=0):
    # bpbot rows are [seconds, right command, left command, joints in degrees].
    # The hand q values are also stored in the joint columns for model playback;
    # bpbot's robot server ignores those finger columns and uses left_command.
    return [float(seconds), 0, left_command] + [
        math.degrees(q) for q in current_joints(body)]


def sync_model_to_robot(robot_item, nxt):
    # bpbot's getJointAngles reads getActualState().command, not encoders.
    reported = list(nxt.getJointAngles())
    body = robot_item.body
    if len(reported) not in (15, body.numJoints):
        raise RuntimeError('Robot reported %d joints; expected 15 or %d'
                           % (len(reported), body.numJoints))
    angles = current_joints(body)
    angles[:len(reported)] = [float(q) for q in reported]
    set_joints(robot_item, angles)
    log.info('Model seeded from bpbot reported joint commands: %d joints',
             len(reported))


def build_motion(robot_item):
    body = robot_item.body
    wrist = body.link('LARM_JOINT5')
    chest = body.link('CHEST_JOINT0')
    if chest is None:
        raise RuntimeError('The loaded robot lacks CHEST_JOINT0')
    left_finger = body.link('LHAND_L')
    right_finger = body.link('LHAND_R')
    start_angles = current_joints(body)
    rows = []
    try:
        # Model fingers slide in metres: +/-0.025 open, zero closed.
        left_finger.q = 0.025
        right_finger.q = -0.025
        body.calcForwardKinematics()
        rows.append(motion_row(body, 1.0, OPEN_LEFT))
        log.info('Planned left-hand open row (command 3)')

        left_finger.q = 0.0
        right_finger.q = 0.0
        body.calcForwardKinematics()
        rows.append(motion_row(body, 1.0, CLOSE_LEFT))
        log.info('Planned left-hand close row (command 4)')

        start_position = np.array(wrist.p, copy=True)
        start_rotation = np.array(wrist.R, copy=True)
        log.info('Starting wrist position [m]: %s', start_position.tolist())
        log.info('Target wrist position [m]: %s',
                 (start_position + np.array([0.0, 0.0, LIFT_METRES])).tolist())
        path = getCustomJointPath(body, chest, wrist)
        if path is None or path.numJoints != 6:
            raise RuntimeError('Could not make a six-joint left-arm IK path')
        for frame in range(1, LIFT_STEPS + 1):
            target = np.eye(4)
            target[:3, :3] = start_rotation
            target[:3, 3] = start_position
            target[2, 3] += LIFT_METRES * frame / LIFT_STEPS
            if not path.calcInverseKinematics(target):
                raise RuntimeError('Left-arm IK failed at lift step %d/%d'
                                   % (frame, LIFT_STEPS))
            body.calcForwardKinematics()
            rows.append(motion_row(body, LIFT_SECONDS / LIFT_STEPS))
        log.info('Planned %d motion rows (2 hand rows, %d lift rows)',
                 len(rows), LIFT_STEPS)
        return rows
    finally:
        # Planning must not leave the visible model at the final pose.
        set_joints(robot_item, start_angles)


def preview_motion(robot_item, rows):
    log.info('PLAY=0: previewing the motion on the Choreonoid model')
    previous = current_joints(robot_item.body)
    for index, row in enumerate(rows, 1):
        target = [math.radians(value) for value in row[3:]]
        frames = max(1, int(round(row[0] * 10.0)))
        for frame in range(1, frames + 1):
            fraction = frame / float(frames)
            angles = [a + (b - a) * fraction for a, b in zip(previous, target)]
            set_joints(robot_item, angles)
            MessageView.instance.flush()
            time.sleep(row[0] / frames)
        previous = target
        if index in (1, 2, len(rows)) or index % 10 == 0:
            log.debug('Previewed motion row %d/%d', index, len(rows))
    log.info('Choreonoid preview completed')


def play_on_robot(nxt, rows):
    log.info('PLAY=1: sending %d rows through bpbot NxtRobot.playMotion', len(rows))
    nxt.playMotion(rows)
    # This readback is the controller's command state, not measured encoder motion.
    reported = list(nxt.getJointAngles())
    expected = [math.radians(value) for value in rows[-1][3:]]
    if len(reported) < 15:
        raise RuntimeError('Robot returned too few joint commands after playback')
    error = max(abs(float(reported[i]) - expected[i]) for i in range(9, 15))
    log.info('Largest left-arm command-state difference [rad]: %.6f', error)
    if error > 0.05:
        raise RuntimeError('Robot command state did not reach the planned left-arm target')
    log.info('bpbot playback returned; physical motion was not independently measured')


def main():
    play = read_play()
    log.info('PLAY=%d (%s)', play, 'robot via bpbot' if play else 'Choreonoid preview')
    robot_item = get_robot_item()
    log.info('Using robot BodyItem %s', robot_item.name)
    nxt = None
    if play:
        import bpbot
        # bpbot's generated protobuf modules use absolute imports.
        nxt_dir = os.path.join(os.path.dirname(bpbot.__file__), 'robotcon', 'nxt')
        if nxt_dir not in sys.path:
            sys.path.insert(0, nxt_dir)
        from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
        nxt = NxtRobot(host='[::]:15005')
        sync_model_to_robot(robot_item, nxt)
    rows = build_motion(robot_item)
    if play:
        play_on_robot(nxt, rows)
    else:
        preview_motion(robot_item, rows)


if __name__ == '__main__':
    try:
        main()
    except Exception:
        log.exception('NEXTAGE command sequence failed')
        raise
