"""
Basic functions for developing and testing
"""


import os
import json
import urx
from math import pi
import logging
from pathlib import Path

import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues
logging.basicConfig(level=logging.INFO)


def read_config_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'robot_config.json')
    with open(config_path) as f:
        config = json.load(f)
    return config

def read_data_from_robot(robot):
    j_temp = robot.get_joint_temperature()
    j_voltage = robot.get_joint_voltage()
    j_current = robot.get_joint_current()
    main_voltage = robot.get_main_voltage()
    robot_voltage = robot.get_robot_voltage()
    robot_current = robot.get_robot_current()

    logging.info("Data From the Robot:")
    logging.info("Joint Temperature: %s, Joint Voltage: %s, Joint Current: %s, Main Voltage: %s", j_temp, j_voltage, j_current,  main_voltage)


def move_robot_simple(robot):
    l = 0.05
    v = 0.05
    a = 0.3

    try:
        pose = robot.getl()  # get TCP pose
        logging.info("TCP Pose: %s", pose)

        pose[2] += l

        # absolute move in base coordinate
        robot.movel(pose, acc=a, vel=v)

        # relative move in base coordinate
        robot.translate((0, 0, -l), acc=a, vel=v)

        # relative move back and forth in tool coordinate
        robot.translate_tool((0, 0, -l), acc=a, vel=v)
        robot.translate_tool((0, 0, l), acc=a, vel=v)


    except Exception as e:
        pass
        logging.info("An error occurred while moving the robot:", e)

def move_tcp(robot):
    v = 0.05
    a = 0.3

    t = robot.get_pose()        # Orientation and Vector, Transformation from base to tcp
    t.orient.rotate_zb(pi / 8)  # rotate tcp around base z
    robot.set_pose(t, vel=v, acc=a) # move tcp to point and orientation defined by a transformation
    t.orient.rotate_zb(-pi / 8)  # rotate tcp around base z
    robot.set_pose(t, vel=v, acc=a)  # move tcp to point and orientation defined by a transformation


def move_robot_joint(robot):
    l = 0.05
    v = 0.05
    a = 0.3
    r = 0.01

    try:
        # Initial joint configuration
        initial_joints = robot.getj()

        robot.translate((l, 0, 0), acc=a, vel=v) # Translating in x

        pose = robot.getl()
        pose[2] += l
        robot.movel(pose, acc=a, vel=v, wait=True) # Moving in z

        # moving in tool z
        robot.translate_tool((0, 0, l), vel=v, acc=a)

        # Sending robot back to original position
        robot.movej(initial_joints, acc=0.8, vel=0.2)


    except Exception as e:
        pass
        logging.info("An error occurred while moving the robot:", e)


def move_robot_p(robot):
    l = 0.1
    v = 0.05
    a = 0.1

    try:
        pose = robot.getl()

        pose[2] += l
        robot.movep(pose, acc=a, vel=v, wait=True)
        pose[2] -= l
        robot.movep(pose, acc=a, vel=v, wait=True)
        pose[1] += l
        robot.movep(pose, acc=a, vel=v, wait=True)
        pose[1] -= l
        robot.movep(pose, acc=a, vel=v, wait=True)

    except Exception as e:
        pass
        logging.info("An error occurred while moving the robot:", e)


def test_function(robot):

    try:
        joint_state = robot.getj()  # get joint state in radians
        TCP_pose    = robot.getl()  # get TCP pose (x, y, z, Rx,Ry, Rz), in base coordinate

        print(TCP_pose)

        TCP_pose[0] = 0.1
        robot.movep(TCP_pose, acc=0.8, vel=0.1, relative=False, wait=True) # absolute move wrt base coordinate
        TCP_pose = robot.getl()
        print(TCP_pose)

        #relative move in base coordinate
        #robot.translate((0, 0, -0.2), acc=0.8, vel=0.1)  # move the tool,  # relative move in base coordinate, just move
        robot.translate_tool((0, 0, 0.2), acc=0.8, vel=0.1)

        # t = robot.get_pose()  # Orientation and Vector, Transformation from base to tcp
        # t.orient.rotate_zb(pi / 8)  # rotate tcp around base z
        # robot.set_pose(t, vel=0.1, acc=0.8)  # move tcp to point and orientation defined by a transformation


    except Exception as e:
        pass
        logging.info("An error occurred while moving the robot:", e)



def main():

    config = read_config_file()
    ip_address_robot = config['ip_robot']

    robot = urx.Robot(ip_address_robot, use_rt=True, urFirm=config['urFirm'])
    robot.set_tcp((0, 0, 0, 0, 0, 0)) # Set tool central point
    robot.set_payload(0.5, (0, 0, 0)) # Kg

    test_function(robot)
    # read_data_from_robot(robot=robot)
    # move_robot_simple(robot=robot)

    robot.close()


if __name__ == "__main__":

    main()
