import numpy as np
from pypylon import pylon

class Camera:

    def __init__(self, camera_matrix_path, camera_distortion_path):

        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

        self.camera_matrix     = np.loadtxt(camera_matrix_path)
        self.camera_distortion = np.loadtxt(camera_distortion_path)

        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.converter = pylon.ImageFormatConverter()

        # converting to opencv bgr format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def get_frame(self):
        frame = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if frame.GrabSucceeded():
            image = self.converter.Convert(frame)
            image = image.GetArray()
        else:
            raise IOError("Camera did not return a frame")
        frame.Release()
        return image
