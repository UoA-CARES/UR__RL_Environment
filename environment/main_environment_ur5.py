import math
import random
import logging
import time

logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues

from tools.util import prepare_point
from tools.camera import Camera
from cares_lib.vision.ArucoDetector import ArucoDetector


class Environment:
    def __init__(self, robot, velocity=0.1, acceleration=0.5, camera_matrix=None, camera_distortion=None):
        self.robot = robot

        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.4, (0, 0, 0))  # Kg

        self.vel = velocity
        self.acc = acceleration

        # Home Position
        self.home_position    = (0.14, -0.50, 0.40) # X, Y , Z
        self.home_orientation = (90.0, 0.0, 0.0)  # Roll, Pith , Yaw  in Degrees

        # Working Area
        self.h, self.k = (self.home_position[0], self.home_position[2])  # central_point in (x, z)
        self.a = 0.30  # Semi-major axis length  x-axis
        self.b = 0.25  # Semi-minor axis length  z-axis

        # wrist 3 range
        self.min_angle_rotation = -80
        self.max_angle_rotation = 80

        # -------- Vision --------- #
        marker_size = 4
        self.camera = Camera(camera_matrix, camera_distortion)
        self.marker_detector = ArucoDetector(marker_size=marker_size)
        self.possible_ids = [7, 8, 9, 10, 11]  # possible IDs in the cube 7-11,
        self.cup_id = 0                        # ID attached to the cup

    def test_position(self, x, y, z):
        ellipse_eq = ((x - self.h) ** 2) / (self.a ** 2) + ((z - self.k) ** 2) / (self.b ** 2)
        if ellipse_eq > 1:
            logging.error("Point X o Z is outside the boundaries of the ellipse")
            return False

        y_tolerance = 0.001
        y_min = self.home_position[1] - y_tolerance
        y_max = self.home_position[1] + y_tolerance
        if not (y_min <= y <= y_max):
            logging.error(f"Y axis is outside the boundary, should be fixed to {self.home_position[1]}")
            return False
        return True

    def test_orientation(self, r, p, y):
        if r != self.home_orientation[0] or p < self.min_angle_rotation or p > self.max_angle_rotation or y != self.home_orientation[2]:
            logging.error("Angle is outside the boundaries for rotation")
            return False
        return True

    def check_point(self, pose):
        valid_position = self.test_position(pose[0], pose[1], pose[2])
        valid_orientation = self.test_orientation(pose[3], pose[4], pose[5])
        if not (valid_position and valid_orientation):
            logging.error("Not valid pose, sending robot to home position")
            move_pose = prepare_point((self.home_position + self.home_orientation))
            return move_pose
        return prepare_point(pose)

    def sample_position(self):
        theta = random.uniform(0, 2 * math.pi)  # Generate random angle in radians
        radio = math.sqrt(random.uniform(0, 1))
        x = self.h + self.a * radio * math.cos(theta)
        z = self.k + self.b * radio * math.sin(theta)
        y = self.home_position[1]  # keep the Y axis fix
        return x, y, z

    def sample_orientation(self):
        roll  = self.home_orientation[0]
        pitch = random.uniform(self.min_angle_rotation, self.max_angle_rotation)
        yaw = self.home_orientation[2]
        return roll, pitch, yaw

    def starting_position(self):
        initial_position = (math.radians(90), math.radians(-90), math.radians(90), math.radians(0), math.radians(90), math.radians(0))  # Joint in rad for home position
        self.robot.movej(initial_position, vel=0.1, acc=1.0, wait=True) # different speed for safety reasons
        logging.info("Robot at initial position")

    def robot_home_position(self):
        desire_pose = prepare_point((self.home_position + self.home_orientation))
        self.robot.movel(desire_pose, vel=0.2, acc=1.0) # different speed for safety reasons
        logging.info("Robot at home position")

    def reset_task(self):
        # self.robot_home_position()
        time.sleep(2)

        # todo, can add if statement when the sensor is mounted, If ball in cup do
        desire_orientation = (self.home_orientation[0], self.home_orientation[1] + 100, self.home_orientation[2])
        rotate_move_pose = prepare_point((self.home_position + desire_orientation))
        self.robot.movel(rotate_move_pose, vel=0.2, acc=1.0)

        self.robot_home_position()

        # this is to reduce the oscillation
        desire_position = (self.home_position[0], self.home_position[1], self.home_position[2] - 0.59)
        touch_ground_move_pose = prepare_point((desire_position + self.home_orientation))
        self.robot.movel(touch_ground_move_pose, vel=0.2, acc=1.0)
        time.sleep(2)

        self.robot_home_position()

    def get_sample_pose(self):
        desire_position = self.sample_position()  # (x, y, z) w.r.t to the base
        desire_orientation = self.sample_orientation() # r p y)
        desire_pose = self.check_point((desire_position + desire_orientation))
        return desire_pose

    def tool_move_pose_test(self):
        desire_tool_pose   =  self.get_sample_pose()
        self.robot.movel(desire_tool_pose, acc=self.acc, vel=self.vel)
        logging.info("Move completed")

    def hard_code_solution(self):

        # pose 1
        desire_position_1 = (self.home_position[0]+0.27, self.home_position[1], self.home_position[2]+0.10)
        desire_orientation_1 = (self.home_orientation[0], self.home_orientation[1], self.home_orientation[2])
        desire_pose_1 = self.check_point((desire_position_1 + desire_orientation_1))
        self.robot.movel(desire_pose_1, acc=self.acc, vel=self.vel)

        # pose 2
        desire_position_2 = (self.home_position[0]-0.27, self.home_position[1], self.home_position[2]-0.10)
        desire_orientation_2 = (self.home_orientation[0], self.home_orientation[1]+45, self.home_orientation[2])
        desire_pose_2 = self.check_point((desire_position_2 + desire_orientation_2))
        self.robot.movel(desire_pose_2, acc=self.acc, vel=self.vel)

        # pose 3
        desire_position_3 = (self.home_position[0]+0.10, self.home_position[1], self.home_position[2])
        desire_orientation_3 = (self.home_orientation[0], self.home_orientation[1]+40, self.home_orientation[2])
        desire_pose_3 = self.check_point((desire_position_3 + desire_orientation_3))
        self.robot.movel(desire_pose_3, acc=self.acc, vel=self.vel)

        # pose 4
        desire_position_4 = (self.home_position[0]+0.10, self.home_position[1], self.home_position[2])
        desire_orientation_4 = (self.home_orientation[0], self.home_orientation[1], self.home_orientation[2])
        desire_pose_4 = self.check_point((desire_position_4 + desire_orientation_4))
        self.robot.movel(desire_pose_4, acc=self.acc, vel=self.vel)

        #self.robot.movels([desire_pose_1, desire_pose_2, desire_pose_3], vel=self.vel, acc=self.acc, radius=0.01)


    def get_marker_poses(self):
        while True:
            frame = self.camera.get_frame()
            marker_poses = self.marker_detector.get_marker_poses(frame,
                                                                 self.camera.camera_matrix,
                                                                 self.camera.camera_distortion,
                                                                 display=True)

            detected_ids = [ids for ids in marker_poses]

            detected_possible_ids = any(ids in detected_ids for ids in self.possible_ids)
            detected_cup_id = self.cup_id in detected_ids

            if detected_possible_ids and detected_cup_id:
                logging.info("Detected IDs: %s", detected_ids)
                break
            logging.info("no marker detected")

        return detected_ids, marker_poses

    def aruco_state_space(self):
        state = []
        marker_ids, marker_poses = self.get_marker_poses()

        cup_marker_pose = marker_poses[self.cup_id]
        cup_position = cup_marker_pose["position"]

        deltas = {"dx": [], "dy": [], "dz": []}

        for id in marker_ids:
            if id != self.cup_id:  # Exclude the cup ID itself
                marker_pose = marker_poses[id]
                marker_position = marker_pose["position"]

                # Calculate the delta distance between the marker and the cup for each axis
                dx =  marker_position[0] - cup_position[0]
                dy =  marker_position[1] - cup_position[1]
                dz =  marker_position[2] - cup_position[2]

                deltas["dx"].append(dx)
                deltas["dy"].append(dy)
                deltas["dz"].append(dz)

        # Calculate the average delta for each axis
        average_dx = sum(deltas["dx"]) / len(deltas["dx"])
        average_dy = sum(deltas["dy"]) / len(deltas["dy"])
        average_dz = sum(deltas["dz"]) / len(deltas["dz"])

        state.extend([average_dx, average_dy, average_dz])
        return state


    def robot_state_space(self):
        state = []
        robot_pose = self.robot.getl()  # end effector pose
        state.append(robot_pose[0])
        state.append(robot_pose[1])
        state.append(robot_pose[2])
        return state


    def get_state(self):
        aruco_state_space = self.aruco_state_space()  # the "ball" position wrt to the cup mark
        robot_state_space = self.robot_state_space()  # the "tool" position
        state = robot_state_space + aruco_state_space
        logging.info("State: %s", state)
        return state

    def get_reward(self):
        pass



