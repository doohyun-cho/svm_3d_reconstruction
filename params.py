import numpy as np
from util import Util
from camera_structure import *

# extrinsic parameters refered from
# https://stradvision.atlassian.net/wiki/spaces/ParkingPost/pages/47091358842/SVM+Offline+calibration+tool

w_img, h_img = 1280, 960
f_x, f_y = 385, 385
c_x, c_y = 640, 480
K = np.array([
    [f_x, 0, c_x],
    [0, f_y, c_y],
    [0, 0, 1]
])

theta_deg_threshold = 90
rad2deg = 180 / np.pi
deg2rad = 1 / rad2deg
k1 = 0.04
k2 = 0.001
k3 = 0
k4 = 0
# k3 = 0.0008
# k4 = -0.0012

# don't worry about image distortion - we're considering only ideal case

cam_type = CamType.Fisheye
# cam_type = CamType.Rectilinear

# extrinsic pitch, roll, and yaw for each camera (degree)
flag_angle_nonzero = True
front_cam_pitch = 12.0 if flag_angle_nonzero else 0
front_cam_roll  = 3.0 if flag_angle_nonzero else 0
front_cam_yaw   = 2.0 if flag_angle_nonzero else 0
front_tx = 3.5
front_ty = 0
front_tz = 1.2

rear_cam_pitch  = 24.0 if flag_angle_nonzero else 0
rear_cam_roll   = -5.0 if flag_angle_nonzero else 0
rear_cam_yaw    = -1.0 if flag_angle_nonzero else 0
rear_tx = -1
rear_ty = 0
rear_tz = 1.2


side_pitch = 22
side_tz = 0.95
left_cam_pitch  = side_pitch if flag_angle_nonzero else 0
left_cam_roll   = -5.0 if flag_angle_nonzero else 0
left_cam_yaw    = 2.0 if flag_angle_nonzero else 0
left_tx = 2.0
left_ty = 1.0
left_tz = side_tz

right_cam_pitch = side_pitch if flag_angle_nonzero else 0
right_cam_roll  = 0.0 if flag_angle_nonzero else 0
right_cam_yaw   = -3.0 if flag_angle_nonzero else 0
right_tx = 2.0
right_ty = -1.0
right_tz = side_tz

vehicle_height = 1.4
vehicle_z_bottom = 0.2
bonnet_x = 1.0
tire_width = 0.3
tire_height = 0.4

util = Util()
R_front = util.rotation_matrix_SV(front_cam_roll, front_cam_pitch, front_cam_yaw)
R_rear = util.rotation_matrix_SV(rear_cam_roll, rear_cam_pitch, rear_cam_yaw + 180)
R_left = util.rotation_matrix_SV(left_cam_roll, left_cam_pitch, left_cam_yaw + 90)
R_right = util.rotation_matrix_SV(right_cam_roll, right_cam_pitch, right_cam_yaw + 270)
R_front_CV = util.rotation_matrix_CV(front_cam_roll, front_cam_pitch, front_cam_yaw)
R_rear_CV = util.rotation_matrix_CV(rear_cam_roll, rear_cam_pitch, rear_cam_yaw + 180)
R_left_CV = util.rotation_matrix_CV(left_cam_roll, left_cam_pitch, left_cam_yaw + 90)
R_right_CV = util.rotation_matrix_CV(right_cam_roll, right_cam_pitch, right_cam_yaw + 270)

front_origin = np.array([front_tx, front_ty, front_tz])
rear_origin = np.array([rear_tx, rear_ty, rear_tz])
left_origin = np.array([left_tx, left_ty, left_tz])
right_origin = np.array([right_tx, right_ty, right_tz])
front_origin_CV = util.XYZ_SV2CV(front_origin)
left_origin_CV = util.XYZ_SV2CV(rear_origin)
right_origin_CV = util.XYZ_SV2CV(left_origin)
rear_origin_CV = util.XYZ_SV2CV(right_origin)



cameras = Cameras(
    F_origin_SV=front_origin,   F_origin_CV=front_origin_CV,    R_F=R_front,    R_F_CV=R_front_CV,  F_cam_type=cam_type,
    L_origin_SV=left_origin,    B_origin_CV=left_origin_CV,     R_L=R_left,     R_B_CV=R_rear_CV,   B_cam_type=cam_type,
    R_origin_SV=right_origin,   L_origin_CV=right_origin_CV,    R_R=R_right,    R_L_CV=R_left_CV,   L_cam_type=cam_type,
    B_origin_SV=rear_origin,    R_origin_CV=rear_origin_CV,     R_B=R_rear,     R_R_CV=R_right_CV,  R_cam_type=cam_type,
    f_x=f_x, f_y=f_y, c_x=c_x, c_y=c_y,
    k1=k1, k2=k2, k3=k3, k4=k4
)

fov_h = util.calculate_fov(w_img, f_x)
fov_v = util.calculate_fov(h_img, f_y)