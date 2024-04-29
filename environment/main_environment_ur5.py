import numpy as np
import math
import random
import logging
import time
from collections import deque


logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues

from tools.util import prepare_point
from tools.camera import Camera
from cares_lib.vision.ArucoDetector import ArucoDetector


class Environment:
    def __init__(self, robot, velocity=0.1, acceleration=0.5, camera_matrix=None, camera_distortion=None):

        self.sensor_in_cup = 0
        self.robot = robot

        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.4, (0, 0, 0))  # Kg

        self.vel = velocity
        self.acc = acceleration

        # Home Position
        self.home_position    = (0.14, -0.50, 0.40) # X, Y , Z
        self.home_orientation = (90.0, 0.0, 0.0)  # Roll, Pith , Yaw --> in Degrees

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

        #-----
        self.max_tries = 0
        self.step_counter = 0
        self.episode_horizon = 10

        # to be use later
        self.state_stack_size  = 3
        self.aruco_state_stack = deque(maxlen=self.state_stack_size)


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
        validate_position    = self.test_position(pose[0], pose[1], pose[2])
        validate_orientation = self.test_orientation(pose[3], pose[4], pose[5])
        if not (validate_position and validate_orientation):
            logging.error("Not valid pose, sending robot to home position")
            input()
            move_pose = prepare_point((self.home_position + self.home_orientation))
            return move_pose
        return prepare_point(pose)

    def starting_position(self):
        # This is just to make sure that the robot start in same position as expected and mounted
        initial_position = (math.radians(90), math.radians(-90), math.radians(90),
                            math.radians(0), math.radians(90), math.radians(0))
        self.robot.movej(initial_position, vel=0.1, acc=1.0, wait=True) # different speed for safety reasons
        logging.info("Robot at initial position")

    def robot_home_position(self):
        desire_pose = prepare_point((self.home_position + self.home_orientation))
        self.robot.movel(desire_pose, vel=0.2, acc=1.0) # different speed for safety reasons
        logging.info("Robot at home position")

    def read_sensor(self):
        # this will be reading the sensor inside the cube
        #sensor_read = self.robot.get_digital_in(0)  # 0 off , 1 on, For any reason this line did not work
        digital_in = 0
        sensor_read = self.robot.get_digital_in_bits()
        if sensor_read == 65536:
            digital_in = 1
        return digital_in

    def environment_reset(self):
        """
        Resets the environment for a new episode.
        """
        self.step_counter = 0
        time.sleep(2)
        self.robot_home_position()  # move robot to home position



        untangled_attempts = 4 # Number of attempts to untangle before human intervention is needed
        distance_string = 33 # cm
        self.sensor_in_cup = self.read_sensor()  # 0 if no ball, 1 if ball is in the cup

        if self.sensor_in_cup == 1:
            # Case 1: the ball in the cup
            self.reset_ball_in_cup()
            self.reset_oscillating_ball()
            self.aruco_state_stack.clear()  # just clear the deque that stack 3 arucos
            state, distance = self.get_state()
        else:
            _, distance = self.get_state()
            if distance >= distance_string - 5:
                # Case 2: The ball is not in the cup but oscillating
                self.reset_oscillating_ball()
                self.aruco_state_stack.clear()  # just clear the deque that stack 3 arucos
                state, _ = self.get_state()

            else:
                # Case 3: the string is tangled up in the robot
                self.reset_tangled_string(untangled_attempts, distance)
                self.aruco_state_stack.clear()  # just clear the deque that stack 3 arucos
                state, _ = self.get_state()

        return state


    def reset_ball_in_cup(self):
        """
        Reset the scenario when the ball is already in the cup.
        """
        logging.info("Resetting with ball already in cup")
        desire_orientation = (self.home_orientation[0], self.home_orientation[1] + 100, self.home_orientation[2])
        rotate_move_pose = prepare_point((self.home_position + desire_orientation))
        self.robot.movel(rotate_move_pose, vel=0.2, acc=1.0)
        self.robot_home_position()

    def reset_oscillating_ball(self):
        """
        Reset the scenario when the ball is not in the cup but oscillating.
        """
        logging.info("Resetting with oscillating ball")
        desire_position = (self.home_position[0], self.home_position[1], self.home_position[2] - 0.60)
        touch_ground_move_pose = prepare_point((desire_position + self.home_orientation))
        self.robot.movel(touch_ground_move_pose, vel=0.2, acc=1.0)
        time.sleep(2)
        self.robot_home_position()

    def reset_tangled_string(self, untangled_attempts, original_distance):
        """
        Reset the scenario when the string is tangled up in the robot.
        """
        logging.info("Resetting with tangled string")
        untangled_direction = False
        rotation_angle = -179

        for i in range(untangled_attempts):
            untangled_direction = self.attempt_to_untangle(rotation_angle, original_distance)
            if untangled_direction:
                break
            rotation_angle *= -1  # Change direction for the next attempt

        if not untangled_direction:
            logging.error("Unable to untangle string. Human intervention required.")
            logging.info("Please untangle the string and press Enter to continue")
            input()  # Wait for user to press Enter
            time.sleep(2)
            self.robot_home_position()

    def attempt_to_untangle(self, rotation_angle, original_distance):
        """
        Attempt to untangle the string by rotating the robot.
        Returns True if successful, False otherwise.
        """
        logging.info(f"Attempting to untangle string with rotation angle")
        desire_orientation = (self.home_orientation[0], self.home_orientation[1] + rotation_angle, self.home_orientation[2])
        untangle_move_pose = prepare_point((self.home_position + desire_orientation))
        self.robot.movel(untangle_move_pose, vel=0.2, acc=1.0)

        _, new_distance = self.get_state()
        if new_distance > original_distance:
            logging.info("untangled direction correctly")
            self.untangle_move(rotation_angle)
            self.reset_oscillating_ball()
            return True
        else:
            self.robot_home_position()
            logging.info("Untangle attempt unsuccessful")
            return False

    def untangle_move(self, rotation_angle):
        desire_orientation = (self.home_orientation[0]-90, self.home_orientation[1] + rotation_angle, self.home_orientation[2])
        untangle_move_pose = prepare_point((self.home_position + desire_orientation))
        self.robot.movel(untangle_move_pose, vel=0.2, acc=1.0)

        desire_orientation = (self.home_orientation[0]-90, self.home_orientation[1] + rotation_angle, self.home_orientation[2]-rotation_angle)
        untangle_move_pose = prepare_point((self.home_position + desire_orientation))
        self.robot.movel(untangle_move_pose, vel=0.2, acc=1.0)
        self.robot_home_position()

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

        distances = []
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

                # Calculate the distance between the marker and the cup
                distance = ((cup_position[0] - marker_position[0]) ** 2 +
                            (cup_position[1] - marker_position[1]) ** 2 +
                            (cup_position[2] - marker_position[2]) ** 2) ** 0.5
                distances.append(distance)

        # Calculate the average delta for each axis
        average_dx = sum(deltas["dx"]) / len(deltas["dx"])
        average_dy = sum(deltas["dy"]) / len(deltas["dy"])
        average_dz = sum(deltas["dz"]) / len(deltas["dz"])

        # Calculate the average final distance
        average_distance = sum(distances) / len(distances)

        state.extend([average_dx, average_dy, average_dz])
        return state, average_distance

    def robot_state_space(self):
        state = []
        robot_pose = self.robot.getl()  # end effector pose
        state.append(robot_pose[0])
        state.append(robot_pose[1])
        state.append(robot_pose[2])
        return state

    def get_state(self):

        self.sensor_in_cup = self.read_sensor()

        if self.sensor_in_cup == 1:
            # if the sensor is on, means the ball is in the cup, therefore I can not see the ball aruco marker,
            # so the dx, dy, dz and the distance are zero
            aruco_state_space, distance = [0, 0, 0], 0

        else:
            # [[dx, dy, dz], distance]
            aruco_state_space, distance = self.aruco_state_space()  # the "ball" position wrt to the cup mark

        # stack 3 consecutive aruco_state_space so the state will be:
        # state = [ Frame1 | Frame2 | Frame 3 | Px, Py, Pz ]
        # state = [ dx, dy, dz| dx, dy, dz | dx, dy, dz,| Px, Py, Pz ]

        if len(self.aruco_state_stack) == 0:
            # this means this is coming after a reset state
            for _ in range(self.state_stack_size):
                self.aruco_state_stack.append(aruco_state_space)
        else:
            self.aruco_state_stack.append(aruco_state_space)


        # Stack the current Aruco marker state with previous Aruco marker states
        stacked_aruco_state = []
        for state in self.aruco_state_stack:
            stacked_aruco_state.extend(state)

        robot_state_space = self.robot_state_space()  # the "tool" position
        state = robot_state_space + stacked_aruco_state
        return state, distance

    def get_reward(self, state, distance):

        dy = state[4] # Change in height axis: positive if cube is under the cup
        done = False
        reward = 0
        scaling_factor = 0.5
        threshold_distance = 15  # cm

        # Penalize if cube is under the cup
        if dy >= 0:
            reward = - scaling_factor * dy  # Penalize proportional to the distance below the cup
        else:
            # Encourage if cube is close to the cup
            if distance < threshold_distance:
                reward = 0.5
                done = True

        if self.sensor_in_cup == 1:
            reward = 1.0  # Reward for successfully placing the ball in the cup
            done = True

        return reward, done


    def tool_move_pose(self, tool_pose):
        self.robot.movel(tool_pose, acc=self.acc, vel=self.vel)
        logging.info("Move completed")

    def step(self, action):

        self.step_counter += 1
        self.tool_move_pose(action)
        input()
        state, distance = self.get_state()
        reward, done    = self.get_reward(state, distance)
        truncated = self.step_counter >= self.episode_horizon

        return state, reward, done, truncated
