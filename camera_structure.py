from enum import Enum
from data_type import *

# This class defines a collection of cameras in different directions. 
class Cameras:
    class _CameraInfo:
        def __init__(
            self, 
            origin_SV, origin_CV, 
            R_SV, R_CV, 
            cam_dir : CamDirection, cam_type : CamType,
            f_x, f_y, c_x, c_y,
            k1, k2, k3, k4
        ):
            # The original location of the camera in SV and openCV coordinates.
            self.origin_SV = origin_SV
            self.origin_CV = origin_CV
            
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
                 F_origin_SV, F_origin_CV, R_F, R_F_CV, F_cam_type,
                 B_origin_SV, B_origin_CV, R_B, R_B_CV, B_cam_type,
                 L_origin_SV, L_origin_CV, R_L, R_L_CV, L_cam_type,
                 R_origin_SV, R_origin_CV, R_R, R_R_CV, R_cam_type,
                 f_x, f_y, c_x, c_y,
                 k1, k2, k3, k4
                 ):
        # Initialization of cameras in different directions.
        self.F = self._CameraInfo(F_origin_SV, F_origin_CV, R_F, R_F_CV, CamDirection.F, F_cam_type, f_x, f_y, c_x, c_y, k1, k2, k3, k4)
        self.B = self._CameraInfo(B_origin_SV, B_origin_CV, R_B, R_B_CV, CamDirection.B, B_cam_type, f_x, f_y, c_x, c_y, k1, k2, k3, k4)
        self.L = self._CameraInfo(L_origin_SV, L_origin_CV, R_L, R_L_CV, CamDirection.L, L_cam_type, f_x, f_y, c_x, c_y, k1, k2, k3, k4)
        self.R = self._CameraInfo(R_origin_SV, R_origin_CV, R_R, R_R_CV, CamDirection.R, R_cam_type, f_x, f_y, c_x, c_y, k1, k2, k3, k4)