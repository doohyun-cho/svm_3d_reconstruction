from params import *
from svm_algorithm import SVMAlgorithm
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class PlotHandler:
    """
    3D World View and 4-directional Camera images
    """
    
    def __init__(self):
        
        self.svm = SVMAlgorithm()

        # Create a figure
        self.fig = plt.figure(constrained_layout=True, figsize=(15, 10))

        # Create a GridSpec for the figure
        gs = GridSpec(2, 3, figure=self.fig, width_ratios=[7, 5, 5], height_ratios=[10, 10])

        # Create a 3D plot in the first position
        self.axs = {}
        self.axs[0] = self.fig.add_subplot(gs[:, 0], projection='3d')

        # Create the other subplots
        self.axs[1] = self.fig.add_subplot(gs[0, 1])
        self.axs[2] = self.fig.add_subplot(gs[0, 2])
        self.axs[3] = self.fig.add_subplot(gs[1, 1])
        self.axs[4] = self.fig.add_subplot(gs[1, 2])

    def draw_SV_axes(self):
        ax = self.axs[0]

        axis_length = 0.5
        origin = np.r_[0, 0, 0]
        O_CV = origin
        dX = np.r_[1,0,0] * axis_length
        dY = np.r_[0,1,0] * axis_length
        dZ = np.r_[0,0,1] * axis_length

        ax.quiver(O_CV[0], O_CV[1], O_CV[2], \
                dX[0], dX[1], dX[2], \
                color='r', linewidth=2, arrow_length_ratio=0.2)
        ax.quiver(O_CV[0], O_CV[1], O_CV[2], \
                dY[0], dY[1], dY[2], \
                color='g', linewidth=2, arrow_length_ratio=0.2)
        ax.quiver(O_CV[0], O_CV[1], O_CV[2], \
                dZ[0], dZ[1], dZ[2], \
                color='b', linewidth=2, arrow_length_ratio=0.2)
        
    def draw_SV_ground_plane(self):
        ax = self.axs[0]

        # Define the rectangle vertices
        rect_points = np.array([[-5, -3, 0],
                            [-5, 3, 0],
                            [10, 3, 0],
                            [10, -3, 0]])
        
        for i in range(4):
            start_point = rect_points[i]
            end_point = rect_points[(i + 1) % 4]
            ax.plot([start_point[0], end_point[0]],
                    [start_point[1], end_point[1]],
                    [start_point[2], end_point[2]], color='k')
        
        # Create a Poly3DCollection for the rectangle and set its color
        rect = Poly3DCollection([rect_points], alpha=0.1)
        rect.set_facecolor('orange')

        # Add the rectangle to the plot
        ax.add_collection3d(rect)
        
    # Function to draw lines between vertices
    def draw_edges_of_vertices(self, vertices, pairs):
        ax = self.axs[0]
        for p1, p2 in pairs:
            ax.plot(*zip(*[vertices[p1], vertices[p2]]), color='k')

    # Function to draw a cylinder (approximated as a polyhedron)
    def draw_tire(self, x_center, y_center, z_center, resolution=20):
        ax = self.axs[0]
        alpha = 0.4

        t = np.linspace(0, 2.*np.pi, resolution)
        y = np.linspace(y_center - tire_width/2, y_center + tire_width/2, resolution)
        y_cylinder = np.outer(np.ones_like(t), y)
        x_cylinder = x_center + tire_height * np.outer(np.cos(t), np.ones_like(y))
        z_cylinder = z_center + tire_height * np.outer(np.sin(t), np.ones_like(y))  # 0.5 is half of the tire width
            
        # Draw the side surface
        ax.plot_surface(x_cylinder, y_cylinder, z_cylinder, color='k', alpha=alpha)

        # Draw the left and right faces
        for y_val in [y_center - tire_width/2, y_center + tire_width/2]:
            x_circle = x_center + tire_height * np.cos(t)  # Y-coordinates of a circle at the current x
            z_circle = z_center + tire_height * np.sin(t)   # Z-coordinates of a circle at the current x
            y_circle = np.full_like(x_circle, y_val)  
            ax.plot(x_circle, y_circle, z_circle, color='k', alpha=alpha)

            theta, radius = np.linspace(0, 2.*np.pi, resolution), np.linspace(0, tire_height, 3)
            theta, radius = np.meshgrid(theta, radius)

            x_circle = x_center + radius * np.cos(theta)  
            y_circle = np.full_like(x_circle, y_val)  
            z_circle = z_center + radius * np.sin(theta)
            ax.plot_surface(x_circle, y_circle, z_circle, color='k', alpha=alpha)

    # Create vehicle cube
    def draw_SV_vehicle_cube(self):
        ax = self.axs[0]

        def get_cube_info(cube_vertices):
            cube_faces = [
                [cube_vertices[0], cube_vertices[1], cube_vertices[5], cube_vertices[4]],
                [cube_vertices[7], cube_vertices[6], cube_vertices[2], cube_vertices[3]],
                [cube_vertices[0], cube_vertices[3], cube_vertices[7], cube_vertices[4]],
                [cube_vertices[1], cube_vertices[2], cube_vertices[6], cube_vertices[5]],
                [cube_vertices[0], cube_vertices[1], cube_vertices[2], cube_vertices[3]],
                [cube_vertices[4], cube_vertices[5], cube_vertices[6], cube_vertices[7]],
            ]

            # Draw lines on the cube
            cube_line_pairs = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]
            
            cube = Poly3DCollection(cube_faces, alpha=.25)
            return cube_line_pairs, cube
        
        cube_body_vertices = np.array([
            [rear_tx, right_ty, vehicle_z_bottom],
            [front_tx, right_ty, vehicle_z_bottom],
            [front_tx, left_ty, vehicle_z_bottom],
            [rear_tx, left_ty, vehicle_z_bottom],
            [rear_tx, right_ty, vehicle_height],
            [front_tx, right_ty, vehicle_height],
            [front_tx, left_ty, vehicle_height],
            [rear_tx, left_ty, vehicle_height],
        ])
        cube_body_line_pairs, cube_body = get_cube_info(cube_body_vertices)

        # Add cube faces to the plot
        ax.add_collection3d(cube_body)
        self.draw_edges_of_vertices(cube_body_vertices, cube_body_line_pairs)        

        # add bonnet
        cube_bonnet_vertices = np.array([
            [front_tx, right_ty, vehicle_z_bottom],
            [front_tx+bonnet_x, right_ty, vehicle_z_bottom],
            [front_tx+bonnet_x, left_ty, vehicle_z_bottom],
            [front_tx, left_ty, vehicle_z_bottom],
            [front_tx, right_ty, vehicle_height*0.6],
            [front_tx+bonnet_x, right_ty, vehicle_height/2],
            [front_tx+bonnet_x, left_ty, vehicle_height/2],
            [front_tx, left_ty, vehicle_height*0.6],
        ])
        cube_bonnet_line_pairs, cube_bonnet = get_cube_info(cube_bonnet_vertices)

        # Add cube faces to the plot
        ax.add_collection3d(cube_bonnet)
        self.draw_edges_of_vertices(cube_bonnet_vertices, cube_bonnet_line_pairs)    
            

    def draw_SV_vehicle(self):
        self.draw_SV_vehicle_cube()
        
        # Draw tires
        self.draw_tire(0, right_ty + tire_width/2, tire_height/2)
        self.draw_tire(0, left_ty - tire_width/2, tire_height/2)
        self.draw_tire(front_tx - 0.1, right_ty + tire_width/2, tire_height/2)
        self.draw_tire(front_tx - 0.1, left_ty - tire_width/2, tire_height/2)
    
    def draw_cam_CV_axes(self, CV_axes_points):
        ax = self.axs[0]

        (O_CV, X_CV, Y_CV, Z_CV) = CV_axes_points
        
        CV_X_axis_delta = X_CV - O_CV
        CV_Y_axis_delta = Y_CV - O_CV
        CV_Z_axis_delta = Z_CV - O_CV

        ax.quiver(O_CV[0], O_CV[1], O_CV[2], \
                CV_X_axis_delta[0], CV_X_axis_delta[1], CV_X_axis_delta[2], \
                color='r', linewidth=2, arrow_length_ratio=0.2)
        ax.quiver(O_CV[0], O_CV[1], O_CV[2], \
                CV_Y_axis_delta[0], CV_Y_axis_delta[1], CV_Y_axis_delta[2], \
                color='g', linewidth=2, arrow_length_ratio=0.2)
        ax.quiver(O_CV[0], O_CV[1], O_CV[2], \
                CV_Z_axis_delta[0], CV_Z_axis_delta[1], CV_Z_axis_delta[2], \
                color='b', linewidth=2, arrow_length_ratio=0.2)
        
    def create_pyramid(self, origin, direction_x, direction_y, fov_h, fov_v, size):
        wd = size * (f_x * fov_h / w_img)
        direction_z = -np.cross(direction_x, direction_y)

        dX = direction_x * wd
        dY = direction_y * size / 2
        dZ = direction_z * size * (h_img / w_img) / 2

        # tip = origin - dX
        tip = origin

        dX_CV = direction_y * size
        dY_CV = -direction_z * size
        dZ_CV = direction_x * size

        O_CV = origin
        X_CV = O_CV + dX_CV
        Y_CV = O_CV + dY_CV
        Z_CV = O_CV + dZ_CV
        CV_axes_points = (O_CV, X_CV, Y_CV, Z_CV)

        vertices = [
            tip,
            tip + dX - dY - dZ,
            tip + dX + dY - dZ,
            tip + dX + dY + dZ,
            tip + dX - dY + dZ,
        ]

        pyramid = Poly3DCollection([
            [vertices[0], vertices[1], vertices[2]],
            [vertices[0], vertices[2], vertices[3]],
            [vertices[0], vertices[3], vertices[4]],
            [vertices[0], vertices[4], vertices[1]],
            [vertices[1], vertices[2], vertices[3], vertices[4]],
        ], alpha=.25)

        return vertices, pyramid, CV_axes_points
    
    def draw_SV_camera_pyramids(self):
        ax = self.axs[0]
        
        direction_x = np.r_[1, 0, 0]
        direction_y = np.r_[0, -1, 0]
        size = 0.4

        front_direction_x = R_front @ direction_x
        front_direction_y = R_front @ direction_y
        rear_direction_x = R_rear @ direction_x
        rear_direction_y = R_rear @ direction_y
        left_direction_x = R_left @ direction_x
        left_direction_y = R_left @ direction_y
        right_direction_x = R_right @ direction_x
        right_direction_y = R_right @ direction_y

        # Create pyramids
        front_vertices, front_pyramid, front_CV_axes_points \
            = self.create_pyramid(front_origin, front_direction_x, front_direction_y, fov_h, fov_v, size)
        rear_vertices, rear_pyramid, rear_CV_axes_points \
            = self.create_pyramid(rear_origin, rear_direction_x, rear_direction_y, fov_h, fov_v, size)
        left_vertices, left_pyramid, left_CV_axes_points \
            = self.create_pyramid(left_origin, left_direction_x, left_direction_y, fov_h, fov_v, size)
        right_vertices, right_pyramid, right_CV_axes_points \
            = self.create_pyramid(right_origin, right_direction_x, right_direction_y, fov_h, fov_v, size)
        
        self.front_vertices = front_vertices
        self.rear_vertices = rear_vertices
        self.left_vertices = left_vertices
        self.right_vertices = right_vertices

        # draw pyramids
        pyramid_line_pairs = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
        ax.add_collection3d(front_pyramid)
        self.draw_edges_of_vertices(front_vertices, pyramid_line_pairs)
        self.draw_cam_CV_axes(front_CV_axes_points)

        ax.add_collection3d(rear_pyramid)
        self.draw_edges_of_vertices(rear_vertices, pyramid_line_pairs)
        self.draw_cam_CV_axes(rear_CV_axes_points)

        ax.add_collection3d(left_pyramid)
        self.draw_edges_of_vertices(left_vertices, pyramid_line_pairs)
        self.draw_cam_CV_axes(left_CV_axes_points)

        ax.add_collection3d(right_pyramid)
        self.draw_edges_of_vertices(right_vertices, pyramid_line_pairs)
        self.draw_cam_CV_axes(right_CV_axes_points)

    def draw_3D_both_visible_pts_on_ground(
            self, 
            XYZs, 
            cam_info0 : Cameras._CameraInfo, 
            cam_info1 : Cameras._CameraInfo
        ):
        ax = self.axs[0]

        is_cam0_visible = self.svm.is_point_visible(XYZs, cam_info0)
        is_cam1_visible = self.svm.is_point_visible(XYZs, cam_info1)
        is_visible = is_cam0_visible & is_cam1_visible
        
        ax.scatter(XYZs[0,is_cam0_visible], 
                XYZs[1,is_cam0_visible], 
                XYZs[2,is_cam0_visible], 
                marker='o', s=10, alpha=0.3)
        
        ax.scatter(XYZs[0,is_cam1_visible], 
                XYZs[1,is_cam1_visible], 
                XYZs[2,is_cam1_visible], 
                marker='o', s=10, alpha=0.3)

        ax.scatter(XYZs[0,is_visible], 
                XYZs[1,is_visible], 
                XYZs[2,is_visible], 
                c='k', marker='o', s=30)
        
        return is_visible
    
    def draw_3D_triangle(self, points_XYZs):
        ax = self.axs[0]

        P1, P2, P3 = points_XYZs
        
        # Create a list of the triangle's vertices
        verts = [[list(P1), list(P2), list(P3)]]

        # Create a 3D polygon and add it to the subplot
        tri = Poly3DCollection(verts, alpha=0.5)
        tri.set_facecolor('black')
        ax.add_collection3d(tri)
        ax.plot(*zip(P1, P2), 'k')
        ax.plot(*zip(P2, P3), 'k')
        ax.plot(*zip(P3, P1), 'k')


    def draw_XYZs2xy_cam(self, XYZs, cam_info : Cameras._CameraInfo, line_pos : LinePos.Undefined, K=K):
        def append_road_line(line_pos, xys_vis):
            if line_pos == LinePos.Undefined:
                if cam_info.road_lines['Undefined'] is None:
                    cam_info.road_lines['Undefined'] = [xys_vis]
                else:
                    cam_info.road_lines['Undefined'].append(xys_vis)
                return
            cam_info.road_lines[line_pos.name] = xys_vis

        ax = self.axs[cam_info.cam_dir.value]
        
        xys = self.svm.world2CAMxy(XYZs, cam_info, K)
        vis = self.svm.is_point_visible(XYZs, cam_info)

        if np.sum(vis) > 0:
            xys_vis = xys[:,vis]
            ax.scatter(xys_vis[0], xys_vis[1], s=10, c='k')
            
            if np.sum(vis) < 4:
                append_road_line(LinePos.Undefined, xys_vis)
            else:
                append_road_line(line_pos, xys_vis)
        
        return xys
    
    def draw_2D_both_first_visible_point(self, xys, idx_first_vis, cam_info : Cameras._CameraInfo, label_type_name=None):
        if idx_first_vis is not None:
            ax = self.axs[cam_info.cam_dir.value]
            ax.scatter(xys[0,idx_first_vis], xys[1,idx_first_vis], s=50, marker='^', label=f'both_visible_{label_type_name}')        
    
    def draw_line_info(self, pointA, pointB, line_pos : LinePos):
        XYZs_line = self.svm.get_line_from_points_3D(pointA, pointB)
        
        vis_FL = self.draw_3D_both_visible_pts_on_ground(XYZs_line, cameras.F, cameras.L)
        vis_FR = self.draw_3D_both_visible_pts_on_ground(XYZs_line, cameras.F, cameras.R)
        vis_BL = self.draw_3D_both_visible_pts_on_ground(XYZs_line, cameras.B, cameras.L)
        vis_BR = self.draw_3D_both_visible_pts_on_ground(XYZs_line, cameras.B, cameras.R)
        idx_first_vis_FL = np.where(vis_FL)[0][1] if np.any(vis_FL) else None
        idx_first_vis_FR = np.where(vis_FR)[0][1] if np.any(vis_FR) else None
        idx_last_vis_BL = np.where(vis_BL)[0][-2] if np.any(vis_BL) else None
        idx_last_vis_BR = np.where(vis_BR)[0][-2] if np.any(vis_BR) else None
        
        xys_F = self.draw_XYZs2xy_cam(XYZs_line, cameras.F, line_pos=line_pos)
        xys_B = self.draw_XYZs2xy_cam(XYZs_line, cameras.B, line_pos=line_pos)
        xys_L = self.draw_XYZs2xy_cam(XYZs_line, cameras.L, line_pos=line_pos)
        xys_R = self.draw_XYZs2xy_cam(XYZs_line, cameras.R, line_pos=line_pos)
        
        self.draw_2D_both_first_visible_point(xys_F, idx_first_vis_FL, cameras.F, 'FL')
        self.draw_2D_both_first_visible_point(xys_L, idx_first_vis_FL, cameras.L, 'FL')
        self.draw_2D_both_first_visible_point(xys_F, idx_first_vis_FR, cameras.F, 'FR')
        self.draw_2D_both_first_visible_point(xys_R, idx_first_vis_FR, cameras.R, 'FR')
        self.draw_2D_both_first_visible_point(xys_B, idx_last_vis_BL, cameras.B, 'BL')
        self.draw_2D_both_first_visible_point(xys_L, idx_last_vis_BL, cameras.L, 'BL')
        self.draw_2D_both_first_visible_point(xys_B, idx_last_vis_BR, cameras.B, 'BR')
        self.draw_2D_both_first_visible_point(xys_R, idx_last_vis_BR, cameras.R, 'BR')
    
        return XYZs_line, xys_F, xys_B, xys_L, xys_R, vis_FL, vis_FR, vis_BL, vis_BR, \
            idx_first_vis_FL, idx_first_vis_FR, idx_last_vis_BL, idx_last_vis_BR
    
    def draw_epipole(self, cam_info : Cameras._CameraInfo, F, label_type_name=None):
        ax = self.axs[cam_info.cam_dir.value]
        
        # u, s, vt = np.linalg.svd(F) 
        # e0 = vt[2]
        # e0 /= e0[2] 
        # ax.scatter(e0[0], e0[1], s=50)

        u, s, vt = np.linalg.svd(F.T) 
        xy_epipole = vt[2]
        xy_epipole /= xy_epipole[2] 
        if cam_info.cam_type is CamType.Fisheye:
            xy_epipole = self.svm.distort_xy(xy_epipole, cam_info)
        ax.scatter(xy_epipole[0], xy_epipole[1], s=50, label=f'epipole_{label_type_name}')
        return xy_epipole[:2]
    
    def draw_epiline(self, cam_info : Cameras._CameraInfo, line):
        ax = self.axs[cam_info.cam_dir.value]

        def calc_y(line, x):
            a, b, c = line  # ax + by + c = 0
            return -(a*x + c)/b
        
        xs_undist = np.linspace(cam_info.c_x - 10*w_img, cam_info.c_x + 12*w_img, 100)
        ys_undist = calc_y(line, xs_undist)
        xys_epiline = np.r_[[xs_undist], 
                    [ys_undist]]
        
        # distort if Fisheye model
        if cam_info.cam_type is CamType.Fisheye:
            xys_epiline = self.svm.distort_xy(xys_epiline, cam_info)
            
        ax.plot(xys_epiline[0], xys_epiline[1])
        return xys_epiline

    def handle_essential_info(
        self,            
        cam_info0 : Cameras._CameraInfo,    # source camera
        cam_info1 : Cameras._CameraInfo,    # target camera
        xy_cam0,
        line_pos : LinePos  # xy_cam0's line position
    ):
        
        E, F = self.svm.compute_essential_matrix(cam_info0, cam_info1, flag_F=True)
        label_type_name = cam_info0.cam_dir.name + '2' + cam_info1.cam_dir.name

        # if fisheye, undistortion needed
        # since we calculate epiline based on undistorted image point
        if cam_info0.cam_type == CamType.Fisheye:
            xy_cam0 = self.svm.undistort_xy(xy_cam0, cam_info0)

        epipole = self.draw_epipole(cam_info1, F, label_type_name)
        epiline_eq = self.svm.compute_epipolar_line_eq(F, xy_cam0)
        epiline = self.draw_epiline(cam_info1, epiline_eq)
        
        cam_info1.epipoles[f'from_{cam_info0.cam_dir.name}'] = epipole 
        cam_info1.epilines[f'from_{cam_info0.cam_dir.name}'] = {'base_point_undist':xy_cam0,   # source point in cam0 of epiline
                                                                'line_pos':line_pos,    # # xy_cam0's line position
                                                                'epiline':epiline}      # epiline

    def set_plots(self):
        ax = self.axs[0]

        # Set axes limits
        ax.set_xlim(-2, 10)
        ax.set_ylim(-3, 3)
        ax.set_zlim(-0.2, 2)

        # Set axes labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Set the axis scale equal
        ax.set_box_aspect((np.ptp(ax.get_xlim()),
                        np.ptp(ax.get_ylim()),
                        np.ptp(ax.get_zlim())))

        for i in range(1, 5):
            ax = self.axs[i]
            ax.plot([0, w_img, w_img, 0, 0], [0, 0, h_img, h_img, 0], 'k')
            ax.plot([0, w_img], [c_y, c_y], 'k-.')
            ax.plot([c_x, c_x], [0, h_img], 'k-.')
            
            ax.set_xlim(0, w_img)
            ax.set_ylim(h_img, 0)
            ax.grid()
        self.axs[1].set_title(f'Front\npitch {front_cam_pitch:.1f}º, roll {front_cam_roll:.1f}º, yaw {front_cam_yaw:.1f}º', fontsize=15)
        self.axs[2].set_title(f'Rear\npitch {rear_cam_pitch:.1f}º, roll {rear_cam_roll:.1f}º, yaw {rear_cam_yaw:.1f}º', fontsize=15)
        self.axs[3].set_title(f'Left\npitch {left_cam_pitch:.1f}º, roll {left_cam_roll:.1f}º, yaw {left_cam_yaw:.1f}º', fontsize=15)
        self.axs[4].set_title(f'Right\npitch {right_cam_pitch:.1f}º, roll {right_cam_roll:.1f}º, yaw {right_cam_yaw:.1f}º', fontsize=15)
        self.axs[1].legend()
        self.axs[2].legend()
        self.axs[3].legend()
        self.axs[4].legend()
        self.axs[1].set_aspect('equal', adjustable='box')
        self.axs[2].set_aspect('equal', adjustable='box')
        self.axs[3].set_aspect('equal', adjustable='box')
        self.axs[4].set_aspect('equal', adjustable='box')
        
        self.fig.tight_layout()

        # Show the plot
        plt.pause(0.1)
        print(1)

