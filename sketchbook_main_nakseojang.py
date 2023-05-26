

# plt.figure()
# num = 100
# xys = np.zeros((2,num))
# xys[0] = np.linspace(50, 1240, num)
# xys[1] = xys[0] * 0.7 + 200
# xys_dist = svm.distort_xy(xys, cameras.F)
# xys_undist = svm.undistort_xy(xys_dist, cameras.F)
# plt.scatter(xys[0], xys[1], label='original')
# plt.scatter(xys_dist[0], xys_dist[1], label='dist')
# plt.scatter(xys_undist[0], xys_undist[1], label='undist')

# plt.plot([0, w_img, w_img, 0, 0], [0, 0, h_img, h_img, 0], 'k')
# plt.plot([0, w_img], [c_y, c_y], 'k-.')
# plt.plot([c_x, c_x], [0, h_img], 'k-.')    
# plt.xlim(0, w_img)
# plt.ylim(h_img, 0)
# plt.grid()
# plt.legend()
# plt.pause(.1)

# pointP = np.r_[5,2,0].reshape(-1,1)
# pointP_CV = svm.XYZ_SV2CV(pointP)
# x_F = svm.worldXYZ_SV2camXYZ_CV(pointP, cameras.F)
# x_L = svm.worldXYZ_SV2camXYZ_CV(pointP, cameras.L)
# y_F = svm.worldXYZ_SV2CAMuv(pointP, cameras.F)
# y_L = svm.worldXYZ_SV2CAMuv(pointP, cameras.L)

# R = cameras.L.R_CV @ cameras.F.R_CV.T
# t = cameras.F.R_CV @ (cameras.L.origin_CV - cameras.F.origin_CV)
# R @ (x_F - t)

# # x_L == R @ (x_F - t)

# svm.worldXYZ_CV2camXYZ_CV(pointP_CV, cameras.F)

# T_F = svm.get_transformation_matrix_CV(cameras.F)
# T_L = svm.get_transformation_matrix_CV(cameras.L)
# P_F = K @ T_F[:3]
# P_L = K @ T_L[:3]

# xy_homo_F = P_F @ pointP_CV
# xy_F = (xy_homo_F / xy_homo_F[-1])[:2]