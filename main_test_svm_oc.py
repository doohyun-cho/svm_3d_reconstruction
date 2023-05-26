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

    Y_line = 1.6
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
        plot_handler.handle_essential_info(cameras.F, cameras.L, xys_F_line1[:,idx_first_vis_FL_line1], line1_pos)
        plot_handler.handle_essential_info(cameras.L, cameras.F, xys_L_line1[:,idx_first_vis_FL_line1], line1_pos)
    if idx_first_vis_FR_line2 is not None:
        plot_handler.handle_essential_info(cameras.F, cameras.R, xys_F_line2[:,idx_first_vis_FR_line2], line2_pos)
        plot_handler.handle_essential_info(cameras.R, cameras.F, xys_R_line2[:,idx_first_vis_FR_line2], line2_pos)
    if idx_last_vis_BL_line1 is not None:
        plot_handler.handle_essential_info(cameras.B, cameras.L, xys_B_line1[:,idx_last_vis_BL_line1], line1_pos)
        plot_handler.handle_essential_info(cameras.L, cameras.B, xys_L_line1[:,idx_last_vis_BL_line1], line1_pos)
    if idx_last_vis_BR_line2 is not None:
        plot_handler.handle_essential_info(cameras.B, cameras.R, xys_B_line2[:,idx_last_vis_BR_line2], line2_pos)
        plot_handler.handle_essential_info(cameras.R, cameras.B, xys_R_line2[:,idx_last_vis_BR_line2], line2_pos)

    plot_handler.set_plots()

    XYZ_F_line1_GT = svm.XYZ_SV2CV(XYZs_line1[:,idx_first_vis_FL_line1])
    XYZ_F_line2_GT = svm.XYZ_SV2CV(XYZs_line2[:,idx_first_vis_FR_line2])
    XYZ_B_line1_GT = svm.XYZ_SV2CV(XYZs_line1[:,idx_last_vis_BL_line1])
    XYZ_B_line2_GT = svm.XYZ_SV2CV(XYZs_line2[:,idx_last_vis_BR_line2])

    XYZ_FL_CV = svm.reconstruct_3D_point(cameras.F, cameras.L, LinePos.Left)
    XYZ_LF_CV = svm.reconstruct_3D_point(cameras.L, cameras.F, LinePos.Left)
    XYZ_BL_CV = svm.reconstruct_3D_point(cameras.B, cameras.L, LinePos.Left)
    XYZ_LB_CV = svm.reconstruct_3D_point(cameras.L, cameras.B, LinePos.Left)
    XYZ_FR_CV = svm.reconstruct_3D_point(cameras.F, cameras.R, LinePos.Right)
    XYZ_RF_CV = svm.reconstruct_3D_point(cameras.R, cameras.F, LinePos.Right)
    XYZ_BR_CV = svm.reconstruct_3D_point(cameras.B, cameras.R, LinePos.Right)
    XYZ_RB_CV = svm.reconstruct_3D_point(cameras.R, cameras.B, LinePos.Right)

    print(np.round(np.c_[XYZ_F_line1_GT, XYZ_FL_CV, XYZ_LF_CV], 3))  # front left
    print(np.round(np.c_[XYZ_F_line2_GT, XYZ_FR_CV, XYZ_RF_CV], 3))  # front right
    print(np.round(np.c_[XYZ_B_line1_GT, XYZ_BL_CV, XYZ_LB_CV], 3))  # back left
    print(np.round(np.c_[XYZ_B_line2_GT, XYZ_BR_CV, XYZ_RB_CV], 3))  # back right





    

    plt.figure()
    num = 100
    xys = np.zeros((2,num))
    xys[0] = np.linspace(50, 1240, num)
    xys[1] = xys[0] * 0.7 + 200
    xys_dist = svm.distort_xy(xys, cameras.F)
    xys_undist = svm.undistort_xy(xys_dist, cameras.F)
    plt.scatter(xys[0], xys[1], label='original')
    plt.scatter(xys_dist[0], xys_dist[1], label='dist')
    plt.scatter(xys_undist[0], xys_undist[1], label='undist')

    plt.plot([0, w_img, w_img, 0, 0], [0, 0, h_img, h_img, 0], 'k')
    plt.plot([0, w_img], [c_y, c_y], 'k-.')
    plt.plot([c_x, c_x], [0, h_img], 'k-.')    
    plt.xlim(0, w_img)
    plt.ylim(h_img, 0)
    plt.grid()
    plt.legend()
    plt.pause(.1)
    
    pointP = np.r_[5,2,0].reshape(-1,1)
    pointP_CV = svm.XYZ_SV2CV(pointP)
    x_F = svm.worldXYZ_SV2camXYZ_CV(pointP, cameras.F)
    x_L = svm.worldXYZ_SV2camXYZ_CV(pointP, cameras.L)
    y_F = svm.worldXYZ_SV2CAMuv(pointP, cameras.F)
    y_L = svm.worldXYZ_SV2CAMuv(pointP, cameras.L)
    
    R = cameras.L.R_CV @ cameras.F.R_CV.T
    t = cameras.F.R_CV @ (cameras.L.origin_CV - cameras.F.origin_CV)
    R @ (x_F - t)

    # x_L == R @ (x_F - t)

    svm.worldXYZ_CV2camXYZ_CV(pointP_CV, cameras.F)

    T_F = svm.get_transformation_matrix_CV(cameras.F)
    T_L = svm.get_transformation_matrix_CV(cameras.L)
    P_F = K @ T_F[:3]
    P_L = K @ T_L[:3]
    
    xy_homo_F = P_F @ pointP_CV
    xy_F = (xy_homo_F / xy_homo_F[-1])[:2]



    print(1)

    




if __name__ == "__main__":
	main()