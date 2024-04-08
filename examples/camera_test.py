import os
import sys
import logging
logging.basicConfig(level=logging.INFO)

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.append(parent_dir)

from tools.camera import Camera
from cares_lib.vision.ArucoDetector import ArucoDetector


class Environment:
    def __init__(self):


        camera_matrix     = parent_dir + '/config/matrix.txt'
        camera_distortion = parent_dir + '/config/distortion.txt'

        marker_size = 4
        self.camera = Camera(camera_matrix, camera_distortion)
        self.marker_detector = ArucoDetector(marker_size=marker_size)
        self.possible_ids = [7, 8, 9, 10, 11]  # possible IDs in the cube 7-11,
        self.cup_id = 0                        # ID attached to the cup


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


    def get_aruco_values(self):
        aruco_values = []

        marker_ids, marker_poses = self.get_marker_poses()

        for id in marker_ids:
            marker_pose = marker_poses[id]
            position = marker_pose["position"]
            aruco_values.append(position[0])  # X
            aruco_values.append(position[1])  # Y
            aruco_values.append(position[2])  # Z
        return aruco_values

    def get_aruco_distance(self):

        marker_ids, marker_poses = self.get_marker_poses()

        cup_marker_pose = marker_poses[self.cup_id]
        cup_position = cup_marker_pose["position"]

        distances = []
        for id in marker_ids:
            if id != self.cup_id:  # Exclude the cup ID itself
                marker_pose = marker_poses[id]
                marker_position = marker_pose["position"]

                # Calculate the distance between the marker and the cup
                distance = ((cup_position[0] - marker_position[0]) ** 2 +
                            (cup_position[1] - marker_position[1]) ** 2 +
                            (cup_position[2] - marker_position[2]) ** 2) ** 0.5
                distances.append(distance)

        if distances:  # Check if distances list is not empty
            # Calculate the average final distance
            average_distance = sum(distances) / len(distances)
            logging.info("Average Distance to the Cup: %s", average_distance)
            return average_distance





env = Environment()
while True:
    distance = env.get_aruco_distance()
