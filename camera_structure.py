from enum import Enum
from data_type import *

# This class defines a collection of cameras in different directions. 
class Cameras:
    class _CameraInfo:
        def __init__(
            self, 
            origin_SV, origin_CV, 
            roll, pitch, yaw,
            R_SV, R_CV, 
            cam_dir : CamDirection, cam_type : CamType,
            f_x, f_y, c_x, c_y,
            k1, k2, k3, k4
        ):
            # The original location of the camera in SV and openCV coordinates.
            self.origin_SV = origin_SV
            self.origin_CV = origin_CV
            
            # The rotation angle in SV coordinate.
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

            # The rotational matrices in SV and openCV coordinates.
            self.R_SV = R_SV
            self.R_CV = R_CV
            
            # The direction and type of the camera.
            self.cam_dir = cam_dir
            self.cam_type = cam_type
            
            # The camera calibration parameters including focal lengths and the principal point coordinates.
            self.f_x = f_x
            self.f_y = f_y
            self.c_x = c_x
            self.c_y = c_y
            
            # The distortion coefficients.
            self.k1 = k1
            self.k2 = k2
            self.k3 = k3
            self.k4 = k4

            # A dictionary to store road lines. Initialized with keys based on LinePos Enum and values set to None.
            # road_lines = {'Left':None, 'Right':None, 'NextLeft':None, ...}
            self.road_lines = dict.fromkeys([pos.name for pos in LinePos])
            
            # A dictionary to store epipoles. Initialized with keys based on CamDirection Enum and values set to None.
            # epilines = {'from_F': None, 'from_B': None...}
            self.epipoles = dict.fromkeys([f'from_{cam_dir.name}' for cam_dir in CamDirection])
            
            # A dictionary to store epilines. Initialized with keys based on CamDirection Enum and values set to None.
            # epilines = {'from_F': None, 'from_B': None...}
            self.epilines = dict.fromkeys([f'from_{cam_dir.name}' for cam_dir in CamDirection])

    def __init__(self, 
                 origin_SV_F, origin_CV_F, roll_F, pitch_F, yaw_F, R_F, R_F_CV, cam_type_F,
                 origin_SV_B, origin_CV_B, roll_B, pitch_B, yaw_B, R_B, R_B_CV, cam_type_B,
                 origin_SV_L, origin_CV_L, roll_L, pitch_L, yaw_L, R_L, R_L_CV, cam_type_L,
                 origin_SV_R, origin_CV_R, roll_R, pitch_R, yaw_R, R_R, R_R_CV, cam_type_R,
                 f_x, f_y, c_x, c_y,
                 k1, k2, k3, k4
                 ):
        # Initialization of cameras in different directions.
        self.F = self._CameraInfo(origin_SV_F, origin_CV_F, roll_F, pitch_F, yaw_F, R_F, R_F_CV, CamDirection.F, cam_type_F, f_x, f_y, c_x, c_y, k1, k2, k3, k4)
        self.B = self._CameraInfo(origin_SV_B, origin_CV_B, roll_B, pitch_B, yaw_B, R_B, R_B_CV, CamDirection.B, cam_type_B, f_x, f_y, c_x, c_y, k1, k2, k3, k4)
        self.L = self._CameraInfo(origin_SV_L, origin_CV_L, roll_L, pitch_L, yaw_L, R_L, R_L_CV, CamDirection.L, cam_type_L, f_x, f_y, c_x, c_y, k1, k2, k3, k4)
        self.R = self._CameraInfo(origin_SV_R, origin_CV_R, roll_R, pitch_R, yaw_R, R_R, R_R_CV, CamDirection.R, cam_type_R, f_x, f_y, c_x, c_y, k1, k2, k3, k4)