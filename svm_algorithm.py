import numpy as np
from params import *
from scipy.interpolate import splprep, splev
from scipy.optimize import fsolve
from shapely.geometry import LineString

class SVMAlgorithm(Util):
    def __init__(self) -> None:
        super().__init__()

    def is_point_visible(self, XYZs, cam_info):
        """
        Checks if a given point is visible in the image.
        
        Args:
            XYZs: 3D world coordinates
            cam_info: Camera information

        Returns:
            visibility (numpy boolean array): True if point is visible
        """
        # Convert world coordinates to camera coordinates
        XYZs_cam_CV = self.worldXYZ_SV2camXYZ_CV(XYZs, cam_info)
        
        # Apply camera intrinsic matrix
        xys = K @ XYZs_cam_CV
        xys = xys[:2,:] / xys[2,:]

        # Apply distortion if camera type is Fisheye
        if cam_info.cam_type is CamType.Fisheye:
            xys = self.distort_xy(xys, cam_info)

        # Check if the projected points are within the image bounds
        visibility = (xys[0,:] >= 0) & (xys[0,:] < w_img) & \
                    (xys[1,:] >= 0) & (xys[1,:] < h_img) & \
                    (XYZs_cam_CV[2,:] > 0)

        return visibility

    def get_line_from_points_3D(self, P0, P1, num_points_=100):
        """
        Generate a line in 3D space from two given points.
        
        Args:
            P0: 3D starting point
            P1: 3D end point
            num_points_: Number of points on the line

        Returns:
            line_points_3D: 3D coordinates of the line
        """
        # Create linspace for each coordinate
        dist = np.linalg.norm(P1 - P0)
        dL = 0.1

        num_points = min(int(dist / dL), num_points_)
        Xs = np.linspace(P0[0], P1[0], num_points)
        Ys = np.linspace(P0[1], P1[1], num_points)
        Zs = np.linspace(P0[2], P1[2], num_points)

        # Stack the coordinates together
        line_points_3D = np.r_[[Xs], [Ys], [Zs]]
        
        return line_points_3D
    
    def compute_essential_matrix(
            self, 
            cam0_info : Cameras._CameraInfo, 
            cam1_info : Cameras._CameraInfo,
            flag_F=False
        ):
        """
        Compute the essential matrix.
        
        Args:
            cam0_info: Information about camera 0
            cam1_info: Information about camera 1
            flag_F: Boolean flag, if true returns the Fundamental matrix

        Returns:
            E: Essential matrix
            F: Fundamental matrix (if flag_F is true)
        """
        def skew(x):
            return np.array([[0, -x[2], x[1]],
                    [x[2], 0, -x[0]],
                    [-x[1], x[0], 0]])
        
        if False:
            R_CV = cam1_info.R_CV @ cam0_info.R_CV.T         # R not happy, not converted to CV yet
            t_CV = self.XYZ_SV2CV(cam1_info.R_SV @ (cam1_info.origin_SV - cam0_info.origin_SV))   # t happy, converted to CV frame
            E = skew(np.ravel(t_CV)) @ R_CV
        else:
            # Compute the Essential matrix
            R_CV = cam1_info.R_CV @ cam0_info.R_CV.T
            t_CV = cam0_info.R_CV @ (cam1_info.origin_CV - cam0_info.origin_CV)
            E = R_CV @ skew(np.ravel(t_CV))
        
        if flag_F:
            F = np.linalg.inv(K.T) @ E @ np.linalg.inv(K)
            F /= F[-1,-1]
            return E, F
        return E
    
    def compute_epipolar_line_eq(self, F, xy_cam0):
        """
        With fundamental matrix F from cam0 to cam1 and arbitrary xy point im cam0,
        the resulting l2 is a 3-element vector that represents the 
        coefficients of the equation of the epipolar line 
        in camera 2's image plane: 
            line[0] * x + line[1] * y + line[2] = 0.
        
        Args:
            F (numpy.ndarray): Fundamental matrix from cam0 to cam1 (3x3)
            xy_cam0 (numpy.ndarray): source point in cam0 image plane (2x1)

        Returns:
            numpy.ndarray: epipolar line vector in cam1 (3x1)
        """
        return F @ np.r_[xy_cam0[0], xy_cam0[1], 1].reshape(-1,1)
    
    def calculate_spline(self, xys):
        """
        Calculates a 2D spline from a set of points.
        
        Args:
            xys: Input points to generate spline

        Returns:
            tck: Tuple (t, c, k) where t is array of knots, c is b-spline coefficients, and k is the degree of the spline.
            fp: Weighted sum of squared residuals of the spline approximation
        """
        tck, fp = splprep(xys, s=0)
        return tck, fp
    
    
    def find_intersections(self, tck1, fp1, tck2, fp2):
        """
        Find the intersections of two splines.
        
        Args:
            tck1, tck2: Spline representation
            fp1, fp2: Residuals for each spline

        Returns:
            numpy.ndarray: Intersection points if any, None otherwise
        """
        x1, y1 = splev(fp1, tck1)
        x2, y2 = splev(fp2, tck2)

        # convert to shapely LineStrings
        line1 = LineString(np.column_stack([x1, y1]))
        line2 = LineString(np.column_stack([x2, y2]))
        
        # calculate the intersection points and return
        intersection = line1.intersection(line2)
        
        if intersection.is_empty:
            return None
        else:
            return np.array(intersection.xy).T

    def reconstruct_3D_point(
        self,
        cam0_info : Cameras._CameraInfo, 
        cam1_info : Cameras._CameraInfo,
        line_pos : LinePos,
        cam0_delta_rpy=None,
        cam1_delta_rpy=None,
        cam1_delta_xy=None
    ):
        """
        Calculate 3D point from cam0's epiline base point 'xy_cam0' and epiline from cam0 -> cam1 

        Parameters
        ----------
        cam0_info : Cameras._CameraInfo
            Camera0 information
        cam1_info : Cameras._CameraInfo
            Camera1 information
        line_pos : LinePos
            Line position (e.g., Left, Right, NextLeft, ...)
        cam0_delta_rpy (Optional)
            Camera0 default setup angle error (degree)
        cam1_delta_rpy (Optional)
            Camera1 default setup angle error (degree)
        cam1_delta_xy (Optional, numpy.ndarray)
            intersection point error in Camera1 image plane (pixel)            

        Returns
        -------
        XYZ : numpy.ndarray (3x1)
            Reconstructed 3D point in world openCV coordinate system
        """
        # epiline   from cam0 -> cam1
        # road line from cam1
        cam0_name = f'from_{cam0_info.cam_dir.name}'
        xy_cam0_undist = cam1_info.epilines[cam0_name]['base_point_undist']
        xys_epiline = cam1_info.epilines[cam0_name]['epiline']
        xys_road_line = cam1_info.road_lines[line_pos.name]

        # generate spline info
        tck_epiline, fp_epiline = self.calculate_spline(xys_epiline)
        tck_road_line, fp_road_line = self.calculate_spline(xys_road_line)

        # find intersections
        # I believe there'd be only a single intersection point in this simulation...
        intersections = self.find_intersections(tck_epiline, fp_epiline, tck_road_line, fp_road_line)
        xy_cam1 = intersections[0]
        if cam1_delta_xy is not None:
            xy_cam1 += cam1_delta_xy
        xy_cam1_undist = self.undistort_xy(xy_cam1, cam1_info)

        K_inv = np.linalg.inv(K)
        uv0 = K_inv @ np.vstack((xy_cam0_undist,1))
        uv1 = K_inv @ np.vstack((xy_cam1_undist,1))

        # triangulate using points from each camera
        T0 = self.get_transformation_matrix_CV(cam0_info, cam0_delta_rpy)
        T1 = self.get_transformation_matrix_CV(cam1_info, cam1_delta_rpy)
        P0 = T0[:3]
        P1 = T1[:3]

        A = np.r_[[uv0[0]*P0[2] - P0[0]],
                  [uv0[1]*P0[2] - P0[1]],
                  [uv1[0]*P1[2] - P1[0]],
                  [uv1[1]*P1[2] - P1[1]]]
        u, s, vt = np.linalg.svd(A)
        
        # the last column of V gives the null space
        XYZ_homo = vt.T[:, -1]
        XYZ = (XYZ_homo / XYZ_homo[-1])[:3].reshape(-1,1)

        return XYZ