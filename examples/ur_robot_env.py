import os
import json
import urx
import math
import random
from math import pi
import logging
logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues



class Env:
    def __init__(self, robot):
        self.robot = robot
        self.initial_set_up()
        self.vel = 0.9
        self.acc = 1.0

        # ellipse area
        self.h, self.k = (0.14, 0.7)  # central_point in (x, z)
        self.a = 0.40  # Semi-major axis length  x-axis
        self.b = 0.05  # Semi-minor axis length   z-axis

        # wrist range
        self.min_angle_deg  = -50
        self. max_angle_deg = 50

    def initial_set_up(self):
        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.5, (0, 0, 0))  # Kg

    def read_joint_state(self):
        joint_state = self.robot.getj()
        return joint_state


    def home_position(self):
        home_pose = (1.5956498, -1.47229, 1.4730, -1.60006, 4.688987, 0.07035151) # Joint in rad for home position
        self.robot.movej(home_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at home")

    def move_reset_position(self):
        reset_pose = (1.595745682, -1.5322321, 1.4995, -3.11227, 4.724702, 0.0004789)
        self.robot.movej(reset_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at reset pose")

    def test_point_inside(self, x, z):
        ellipse_eq = ((x - self.h) ** 2) / (self.a ** 2) + ((z - self.k) ** 2) / (self.b ** 2)
        if ellipse_eq <= 1:
            pass
        else:
            try:
                raise Exception("Point is outside the boundaries of the ellipse")
            except Exception as e:
                logging.info("out of boundaries", e)


    def working_angle_wrist(self):
        min_angle_rad = math.radians(self.min_angle_deg)
        max_angle_rad = math.radians(self.max_angle_deg)
        random_angle_rad = random.uniform(min_angle_rad, max_angle_rad)
        return random_angle_rad


    def working_area_x_z(self):
        theta = random.uniform(0, 2 * math.pi)  # Generate random angle in radians
        radio = math.sqrt(random.uniform(0, 1))

        x = self.h + self.a * radio * math.cos(theta)
        z = self.k + self.b * radio * math.sin(theta)
        y = -0.5  # keep the Y axis fix
        self.test_point_inside(x, z)
        return x, y, z

    def tool_wrist_3_movement(self):
        desire_point = self.working_area_x_z()  # (x, y, z) w.r.t to the base
        desire_angle = self.working_angle_wrist() # wrist3 angle

        t = self.robot.get_pose()  # Orientation and Vector, Transformation matrix from base to tcp
        t.orient.rotate_yb(desire_angle)  # rotate tcp around base y
        t.pos[:3] = desire_point  # translate tcp in x,y,z  w.r.t base

        self.robot.set_pose(t, vel=self.vel, acc=self.acc)  # move tcp to point and orientation defined by a transformation

    def tool_move_test(self):
        tool_pose = self.robot.getl()
        desire_point = self.working_area_x_z() # (x, y, z) w.r.t to the base
        tool_pose[:3] = desire_point # keep the orientation
        self.robot.movel(tool_pose, acc=self.acc, vel=self.vel)



def read_config_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'robot_config.json')
    with open(config_path) as f:
        config = json.load(f)
    return config

def main():
    config = read_config_file()
    ip_address_robot = config['ip_robot']

    # robot = urx.Robot(ip_address_robot, use_rt=True, urFirm=config['urFirm'])
    robot = urx.Robot(ip_address_robot)


    robot_env = Env(robot)
    #joints = robot_env.read_joint_state()

    #robot_env.home_position()
    robot_env.move_reset_position()

    for _ in range(10):
        #robot_env.tool_move_test()
        robot_env.tool_wrist_3_movement()

    robot_env.move_reset_position()

    robot.close()


if __name__ == "__main__":
    main()
