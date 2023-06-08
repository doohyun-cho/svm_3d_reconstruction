from params import *
import numpy as np
from matplotlib import pyplot as plt
from plot_handler import PlotHandler
from svm_algorithm import SVMAlgorithm

def main():
    svm = SVMAlgorithm()
    plot_handler = PlotHandler()    

    plot_handler.draw_SV_axes()
    plot_handler.draw_SV_ground_plane()

    plot_handler.draw_SV_camera_pyramids()
    plot_handler.draw_SV_vehicle()

    Y_line = 1.2
    pointA_line1 = np.array([-5, Y_line, 0])
    pointB_line1 = np.array([10, Y_line, 0])
    line1_pos = LinePos.Left
    pointA_line2 = np.array([-5, -Y_line, 0])
    pointB_line2 = np.array([10, -Y_line, 0])
    line2_pos = LinePos.Right

    XYZs_line1, xys_F_line1, xys_B_line1, xys_L_line1, xys_R_line1, \
        vis_FL_line1, vis_FR_line1, vis_BL_line1, vis_BR_line1, \
        idx_first_vis_FL_line1, idx_first_vis_FR_line1, idx_last_vis_BL_line1, idx_last_vis_BR_line1 = \
        plot_handler.draw_line_info(pointA_line1, pointB_line1, line1_pos)
    XYZs_line2, xys_F_line2, xys_B_line2, xys_L_line2, xys_R_line2, \
        vis_FL_line2, vis_FR_line2, vis_BL_line2, vis_BR_line2, \
        idx_first_vis_FL_line2, idx_first_vis_FR_line2, idx_last_vis_BL_line2, idx_last_vis_BR_line2 = \
        plot_handler.draw_line_info(pointA_line2, pointB_line2, line2_pos)

    if idx_first_vis_FL_line1 is not None:
        plot_handler.handle_essential_info(cameras.F, cameras.L, xys_F_line1, vis_FL_line1, line1_pos)
        plot_handler.handle_essential_info(cameras.L, cameras.F, xys_L_line1, vis_FL_line1, line1_pos)
    if idx_first_vis_FR_line2 is not None:
        plot_handler.handle_essential_info(cameras.F, cameras.R, xys_F_line2, vis_FR_line2, line2_pos)
        plot_handler.handle_essential_info(cameras.R, cameras.F, xys_R_line2, vis_FR_line2, line2_pos)
    if idx_last_vis_BL_line1 is not None:
        plot_handler.handle_essential_info(cameras.B, cameras.L, xys_B_line1, vis_BL_line1, line1_pos)
        plot_handler.handle_essential_info(cameras.L, cameras.B, xys_L_line1, vis_BL_line1, line1_pos)
    if idx_last_vis_BR_line2 is not None:
        plot_handler.handle_essential_info(cameras.B, cameras.R, xys_B_line2, vis_BR_line2, line2_pos)
        plot_handler.handle_essential_info(cameras.R, cameras.B, xys_R_line2, vis_BR_line2, line2_pos)

    plot_handler.set_plots()

    idx_F = 1
    idx_B = -2
    XYZ_F_line1_GT = svm.XYZ_SV2CV(XYZs_line1[:,vis_FL_line1[idx_F]])
    XYZ_F_line2_GT = svm.XYZ_SV2CV(XYZs_line2[:,vis_FR_line2[idx_F]])
    XYZ_B_line1_GT = svm.XYZ_SV2CV(XYZs_line1[:,vis_BL_line1[idx_B]])
    XYZ_B_line2_GT = svm.XYZ_SV2CV(XYZs_line2[:,vis_BR_line2[idx_B]])

    XYZ_FL_CV = svm.reconstruct_3D_point(cameras.F, cameras.L, LinePos.Left, vis_FL_line1[idx_F])
    XYZ_LF_CV = svm.reconstruct_3D_point(cameras.L, cameras.F, LinePos.Left, vis_FL_line1[idx_F])
    XYZ_FR_CV = svm.reconstruct_3D_point(cameras.F, cameras.R, LinePos.Right, vis_FR_line2[idx_F])
    XYZ_RF_CV = svm.reconstruct_3D_point(cameras.R, cameras.F, LinePos.Right, vis_FR_line2[idx_F])
    XYZ_BL_CV = svm.reconstruct_3D_point(cameras.B, cameras.L, LinePos.Left, vis_BL_line1[idx_B])
    XYZ_LB_CV = svm.reconstruct_3D_point(cameras.L, cameras.B, LinePos.Left, vis_BL_line1[idx_B])
    XYZ_BR_CV = svm.reconstruct_3D_point(cameras.B, cameras.R, LinePos.Right, vis_BR_line2[idx_B])
    XYZ_RB_CV = svm.reconstruct_3D_point(cameras.R, cameras.B, LinePos.Right, vis_BR_line2[idx_B])

    print(np.round(np.c_[XYZ_F_line1_GT, XYZ_FL_CV, XYZ_LF_CV], 3))  # front left
    print(np.round(np.c_[XYZ_F_line2_GT, XYZ_FR_CV, XYZ_RF_CV], 3))  # front right
    print(np.round(np.c_[XYZ_B_line1_GT, XYZ_BL_CV, XYZ_LB_CV], 3))  # back left
    print(np.round(np.c_[XYZ_B_line2_GT, XYZ_BR_CV, XYZ_RB_CV], 3))  # back right

    delta_rpy_F = (0, 10.0, 0)
    delta_rpy_L = (0, 0, 0)
    XYZ_FL_CV_F_error = svm.reconstruct_3D_point(cameras.F, cameras.L, LinePos.Left, vis_FL_line1[idx_F], delta_rpy_F, delta_rpy_L)
    XYZ_LF_CV_F_error = svm.reconstruct_3D_point(cameras.L, cameras.F, LinePos.Left, vis_FL_line1[idx_F], delta_rpy_L, delta_rpy_F)
    print(np.round(np.c_[XYZ_F_line1_GT, XYZ_FL_CV_F_error, XYZ_LF_CV_F_error], 3))  # front left
    
    xy_gt_from_XYZ = svm.worldXYZ_SV2CAMxy(XYZs_line1[:,vis_FL_line1[idx_F]].reshape(-1,1), cameras.F, K)
    xy_gt_from_recon = svm.worldXYZ_CV2CAMxy(XYZ_FL_CV, cameras.F, K=K)
    xy_error = svm.worldXYZ_CV2CAMxy(XYZ_FL_CV_F_error, cameras.F, K=K)

    camL_delta_xy = np.r_[3,0]
    XYZ_LF_CV_xy_error = svm.reconstruct_3D_point(cameras.L, cameras.F, LinePos.Left, vis_FL_line1[idx_F], (0,0,0), (0,0,0), cam1_delta_xy=camL_delta_xy)
    print(np.round(np.c_[XYZ_F_line1_GT, XYZ_LF_CV_xy_error], 3))  # front left

    df_camFL_CV = svm.calculate_bundle_info(cameras.F, cameras.L, LinePos.Left, vis_FL_line1, XYZs_line1)
    df_camLF_CV = svm.calculate_bundle_info(cameras.L, cameras.F, LinePos.Left, vis_FL_line1, XYZs_line1)
    df_camFR_CV = svm.calculate_bundle_info(cameras.F, cameras.R, LinePos.Right, vis_FR_line2, XYZs_line2)
    df_camRF_CV = svm.calculate_bundle_info(cameras.R, cameras.F, LinePos.Right, vis_FR_line2, XYZs_line2)
    df_camBL_CV = svm.calculate_bundle_info(cameras.B, cameras.L, LinePos.Left, vis_BL_line1, XYZs_line1)
    df_camLB_CV = svm.calculate_bundle_info(cameras.L, cameras.B, LinePos.Left, vis_BL_line1, XYZs_line1)
    df_camBR_CV = svm.calculate_bundle_info(cameras.B, cameras.R, LinePos.Right, vis_BR_line2, XYZs_line2)
    df_camRB_CV = svm.calculate_bundle_info(cameras.R, cameras.B, LinePos.Right, vis_BR_line2, XYZs_line2)

    df_camFL_CV_cam_angle_err = svm.calculate_bundle_info(cameras.F, cameras.L, LinePos.Left, vis_FL_line1, XYZs_line1, cam0_delta_rpy=delta_rpy_F)
    # df_camFL_CV_cam_angle_err['XYZ_err'].apply(np.linalg.norm)
    # df_camFL_CV_cam_angle_err['xy_err'].apply(np.linalg.norm)

    print(1)

if __name__ == "__main__":
	main()