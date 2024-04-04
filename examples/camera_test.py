
from tools.camera import Camera
from cares_lib.vision.ArucoDetector import ArucoDetector


class Environment:
    def __init__(self):
        marker_size = 4

        camera_matrix     = '/home/david_lab/UoA_Repository/UR_camera_system/config/matrix.txt'
        camera_distortion = '/home/david_lab/UoA_Repository/UR_camera_system/config/distortion.txt'

        self.camera = Camera(camera_matrix, camera_distortion)
        self.marker_detector = ArucoDetector(marker_size=marker_size)


    def get_aruco_state_space(self):
        state = []

        marker_ids, marker_poses = self.get_marker_poses()

        for id in marker_ids:
            marker_pose = marker_poses[id]
            position = marker_pose["position"]
            state.append(position[0])  # X
            state.append(position[1])  # Y
            state.append(position[2])  # Z
        return state

    def get_marker_poses(self):
        possible_ids = [0, 1, 2, 3, 4, 5, 6, 7]  # possible ids

        while True:
            frame = self.camera.get_frame()
            marker_poses = self.marker_detector.get_marker_poses(frame, self.camera.camera_matrix,  self.camera.camera_distortion, display=True)

            detected_ids = [ids for ids in marker_poses]

            if any(ids in detected_ids for ids in possible_ids):
                print("Detected IDs:", detected_ids)
                break
            print("no marker detected")

        return detected_ids, marker_poses


a= Environment()
a.get_aruco_state_space()

while True:
    state = a.get_aruco_state_space()
    print(state)
