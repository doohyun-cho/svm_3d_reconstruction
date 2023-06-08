import numpy as np
from camera_structure import *
from scipy.optimize import fmin, minimize

class struct():
    pass

class Util:
    """
    Initialize the Util class
    """
    def __init__(self) -> None:
        self.flag_undistort_fmin = True     # whether to use scipy fmin for point undistortion

    def calculate_fov(self, image_size, focal_length):
        """
        Calculate the field of view (FOV)

        Args:
            image_size (float): The size of the image
            focal_length (float): The focal length of the camera lens

        Returns:
            float: The field of view
        """
        return 2 * np.arctan(image_size / (2 * focal_length))

    def rotation_matrix_SV(self, roll_deg, pitch_deg, yaw_deg):
        """
        Calculate the rotation matrix in SV coordinate system

        Args:
            roll_deg (float): Roll angle in degrees
            pitch_deg (float): Pitch angle in degrees
            yaw_deg (float): Yaw angle in degrees

        Returns:
            numpy.ndarray: Rotation matrix in SV coordinate system (3x3)
        """
        # Convert degrees to radians
        roll_rad = np.radians(roll_deg)
        pitch_rad = np.radians(pitch_deg)
        yaw_rad = np.radians(yaw_deg)

        # Calculate the individual rotation matrices
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)],
        ])

        R_pitch = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)],
        ])

        R_yaw = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1],
        ])

        R = R_yaw @ R_pitch @ R_roll

        return R    
    
    def rotation_matrix_CV(self, roll_deg, pitch_deg, yaw_deg):
        """
        Calculate the rotation matrix in openCV coordinate system

        Args:
            roll_deg (float): Roll angle in degrees
            pitch_deg (float): Pitch angle in degrees
            yaw_deg (float): Yaw angle in degrees

        Returns:
            numpy.ndarray: Rotation matrix in openCV coordinate system (3x3)
        """
        # Convert degrees to radians
        roll_rad = np.radians(roll_deg)
        pitch_rad = np.radians(pitch_deg)
        yaw_rad = np.radians(yaw_deg)

        # Calculate the individual rotation matrices
        R_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
            [0, np.sin(pitch_rad), np.cos(pitch_rad)],
        ])
        R_yaw = np.array([
            [np.cos(yaw_rad), 0, np.sin(yaw_rad)],
            [0, 1, 0],
            [-np.sin(yaw_rad), 0, np.cos(yaw_rad)],
        ])
        R_roll = np.array([
            [np.cos(roll_rad), -np.sin(roll_rad), 0],
            [np.sin(roll_rad), np.cos(roll_rad), 0],
            [0, 0, 1],
        ]).T
        
        R = R_roll @ R_pitch @ R_yaw

        return R    
    
    def worldXYZ_SV2camXYZ_SV(self, XYZs_SV, cam_info : Cameras._CameraInfo):
        """
        Convert world coordinates in SV coordinate system to camera coordinates in SV coordinate system

        Args:
            XYZs_SV (numpy.ndarray): 3D world points in SV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: 3D points in camera coordinates in SV coordinate system (3xn)
        """
        origin_cam_SV, R_SV = cam_info.origin_SV, cam_info.R_SV

        # Compute the extrinsic matrix
        if origin_cam_SV.shape != (3,1):
            origin_cam_SV = np.array(origin_cam_SV).reshape(3, 1)

        # Project the world points to the image plane
        # Both ways are available
        if True:
            T = np.eye(4)
            T[:3,:3] = R_SV.T
            T[:3,3] = R_SV.T @ (-origin_cam_SV)[:,0]
            XYZs_cam_SV = T @ np.r_[XYZs_SV, 
                                    [[1]*XYZs_SV.shape[1]]]
            return XYZs_cam_SV[:3]
        else:
            XYZs_cam_SV = R_SV.T @ (XYZs_SV - origin_cam_SV)
            return XYZs_cam_SV
    
    def XYZ_SV2CV(self, XYZs_SV):
        """
        Convert coordinates from SV coordinate system to openCV coordinate system

        Args:
            XYZs_SV (numpy.ndarray): 3D points in SV coordinate system (3xn)

        Returns:
            numpy.ndarray: 3D points in CV coordinate system (3xn)
        """
        if len(XYZs_SV.shape) == 1:
            XYZs_SV = XYZs_SV.reshape(-1,1)
        T_SV2CV = np.r_[[[0,-1,0]],
                        [[0,0,-1]],
                        [[1,0,0]]]
        XYZs_CV = T_SV2CV @ XYZs_SV
        
        return XYZs_CV
    
    def get_transformation_matrix_CV(self, cam_info : Cameras._CameraInfo, delta_rpy=None):
        """
        Get the transformation matrix in openCV coordinate system

        Args:
            cam_info (Cameras._CameraInfo): Camera information
            delta_rpy (tuple with three elements - roll, pitch, and yaw in degree)
                camera setup error angle (for error analysis)
                if None, use default rotation matrix

        Returns:
            numpy.ndarray: Transformation matrix (4x4)
        """
        origin_cam_CV = cam_info.origin_CV
        if delta_rpy is None:
            R_CV = cam_info.R_CV
        else:
            delta_roll, delta_pitch, delta_yaw = delta_rpy        
            roll = cam_info.roll + delta_roll
            pitch = cam_info.pitch + delta_pitch
            yaw = cam_info.yaw + delta_yaw
            R_CV = self.rotation_matrix_CV(roll, pitch, yaw)

        T = np.eye(4)
        T[:3,:3] = R_CV
        T[:3,3] = R_CV @ (-origin_cam_CV)[:,0]
        
        return T

    def worldXYZ_SV2camXYZ_CV(self, worldXYZs_SV, cam_info : Cameras._CameraInfo):
        """
        Convert world coordinates in SV coordinate system to camera coordinates in openCV coordinate system

        Args:
            worldXYZs_SV (numpy.ndarray): 3D world points in SV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: 3D points in camera coordinates in openCV coordinate system (3xn)
        """
        XYZs_cam_SV = self.worldXYZ_SV2camXYZ_SV(worldXYZs_SV, cam_info)
        XYZs_cam_CV = self.XYZ_SV2CV(XYZs_cam_SV)
        
        return XYZs_cam_CV

    def worldXYZ_CV2camXYZ_CV(self, worldXYZs_CV, cam_info : Cameras._CameraInfo):
        """
        Convert world coordinates in CV coordinate system to camera coordinates in openCV coordinate system

        Args:
            worldXYZs_CV (numpy.ndarray): 3D world points in CV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: 3D points in camera coordinates in openCV coordinate system (3xn)
        """
        
        origin_cam_CV, R_CV = cam_info.origin_CV, cam_info.R_CV

        # Compute the extrinsic matrix
        if origin_cam_CV.shape != (3,1):
            origin_cam_CV = np.array(origin_cam_CV).reshape(3, 1)

        # Project the world points to the image plane
        T = np.eye(4)
        T[:3,:3] = R_CV
        T[:3,3] = R_CV @ (-origin_cam_CV)[:,0]
        XYZs_cam_CV = T @ np.r_[worldXYZs_CV, 
                                [[1]*worldXYZs_CV.shape[1]]]
        return XYZs_cam_CV[:3]
    
    def camXYZ_CV2xy(self, XYZs_CV, cam_info, K):
        """
        Convert camera XYZ coordinates to camera image coordinates

        Args:
            XYZs_CV (numpy.ndarray): 3D points in camera CV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information
            K (numpy.ndarray): Camera calibration matrix (3x3)

        Returns:
            numpy.ndarray: 2D image coordinates in camera coordinate system (2xn)
        """
        xys = K @ XYZs_CV
        xys = xys[:2,:] / xys[2,:]

        # distort if Fisheye model
        if cam_info.cam_type is CamType.Fisheye:
            xys_dist = self.distort_xy(xys, cam_info)
            xys_undist = self.undistort_xy(xys, cam_info)
            return xys_dist[:2]
        
        return xys[:2]
    
    def worldXYZ_SV2CAMxy(self, XYZs_SV, cam_info : Cameras._CameraInfo, K):
        """
        Convert world coordinates to camera image coordinates

        Args:
            XYZs_SV (numpy.ndarray): 3D world points in SV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information
            K (numpy.ndarray): Camera calibration matrix (3x3)

        Returns:
            numpy.ndarray: 2D image coordinates in camera coordinate system (2xn)
        """
        XYZs_CV = self.worldXYZ_SV2camXYZ_CV(XYZs_SV, cam_info)
        xys = self.camXYZ_CV2xy(XYZs_CV, cam_info, K)
        return xys
    
    def worldXYZ_CV2CAMxy(self, worldXYZs_CV, cam_info : Cameras._CameraInfo, K):
        """
        Convert world coordinates to camera image coordinates

        Args:
            XYZs_CV (numpy.ndarray): 3D world points in CV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information
            K (numpy.ndarray): Camera calibration matrix (3x3)

        Returns:
            numpy.ndarray: 2D image coordinates in camera coordinate system (2xn)
        """
        XYZs_CV = self.worldXYZ_CV2camXYZ_CV(worldXYZs_CV, cam_info)
        xys = self.camXYZ_CV2xy(XYZs_CV, cam_info, K)
        return xys

    def worldXYZ_SV2CAMuv(self, XYZs_SV, cam_info : Cameras._CameraInfo):
        """
        Convert world coordinates in SV coordinate system to camera image coordinates

        Args:
            XYZs_SV (numpy.ndarray): 3D world points in SV coordinate system (3xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: 2D image coordinates in camera coordinate system (2xn)
        """
        XYZs_CV = self.worldXYZ_SV2camXYZ_CV(XYZs_SV, cam_info)
        points_uv = (XYZs_CV / XYZs_CV[2])
        
        return points_uv
    
    def xy2uv(self, xys, cam_info : Cameras._CameraInfo):
        """
        Convert image coordinates (xy) to normalized image coordinates (uv)

        Args:
            xys (numpy.ndarray): 2D image coordinates in camera coordinate system (2xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: Normalized image coordinates (2xn)
        """
        us = (xys[0] - cam_info.c_x) / cam_info.f_x
        vs = (xys[1] - cam_info.c_y) / cam_info.f_y
        uvs = np.r_[[us], [vs]]        
        return uvs
    
    def uv2xy(self, uvs, cam_info : Cameras._CameraInfo):
        """
        Convert normalized image coordinates (uv) to image coordinates (xy)

        Args:
            uvs (numpy.ndarray): Normalized image coordinates (2xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: 2D image coordinates in camera coordinate system (2xn)
        """
        xs = uvs[0]*cam_info.f_x + cam_info.c_x
        ys = uvs[1]*cam_info.f_y + cam_info.c_y
        xys = np.r_[[xs], [ys]]
        return xys
    
    def distort_xy(self, xys_undist, cam_info : Cameras._CameraInfo):
        """
        Apply distortion to image coordinates (xy) in camera coordinate system

        Args:
            xys_undist (numpy.ndarray): Undistorted image coordinates (xy) in camera coordinate system (2xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: Distorted image coordinates (xy) in camera coordinate
        """
        xs_undist = xys_undist[0]
        ys_undist = xys_undist[1]
        r = np.sqrt((xs_undist-cam_info.c_x)**2 + (ys_undist-cam_info.c_y)**2)
        inv_f = 1 / np.sqrt(cam_info.f_x * cam_info.f_y)
        theta = np.arctan(r * inv_f)
        if np.any(theta >= np.pi/2):
            print('bad')
        # Avoid division by zero
        radial = np.where(r != 0, (theta + cam_info.k1 * theta**3 + 
                               cam_info.k2 * theta**5 + 
                               cam_info.k3 * theta**7 + 
                               cam_info.k4 * theta**9) / r, 
                      theta)
        xs_dist = (radial * cam_info.f_x * (xs_undist - cam_info.c_x)) + cam_info.c_x
        ys_dist = (radial * cam_info.f_y * (ys_undist - cam_info.c_y)) + cam_info.c_y
        
        xys_dist = np.r_[[xs_dist], 
                         [ys_dist]]
        return xys_dist
    
    def distort_uv(self, uvs_undist, cam_info : Cameras._CameraInfo):
        """
        Apply distortion to normalized image coordinates (uv) in camera coordinate system

        Args:
            uvs_undist (numpy.ndarray): Undistorted normalized image coordinates (uv) in camera coordinate system (2xn)
            cam_info (Cameras._CameraInfo): Camera information

        Returns:
            numpy.ndarray: Distorted normalized image coordinates (uv) in camera coordinate system (2xn)
        """
        us_undist = uvs_undist[0]
        vs_undist = uvs_undist[1]
        r = np.sqrt(us_undist**2 + vs_undist**2)
        theta = np.arctan(r)
        if np.any(theta >= np.pi/2):
            print('bad')
        # Avoid division by zero
        radial = np.where(r != 0, (theta + cam_info.k1 * theta**3 + 
                               cam_info.k2 * theta**5 + 
                               cam_info.k3 * theta**7 + 
                               cam_info.k4 * theta**9) / r, 
                      theta)
        us_dist = radial * us_undist
        vs_dist = radial * vs_undist
        
        uvs_dist = np.r_[[us_dist], 
                         [vs_dist]]
        return uvs_dist
    
    # from pinhole model to fisheye
    def undistort_xy(self, 
                     xys_dist, 
                     cam_info : Cameras._CameraInfo, 
                     err_max_iter = 100,
                     err_threshold = 1e-6,
                    ):
        """
        Undistort distorted image coordinates (xy) in camera coordinate system

        Args:
            xys_dist (numpy.ndarray): Distorted image coordinates (xy) in camera coordinate system (2xn)
            cam_info (Cameras._CameraInfo): Camera information
            err_max_iter (int): Maximum number of iterations for iterative undistortion (default: 100)
            err_threshold (float): Threshold for convergence in iterative undistortion (default: 1e-6)

        Returns:
            numpy.ndarray: Undistorted image coordinates (xy) in camera coordinate system (2xn)
        """

        if len(xys_dist.shape) == 1:
            xys_dist = xys_dist.reshape(-1,1)
        uvs_dist = self.xy2uv(xys_dist, cam_info)
        uvs_undist = np.copy(uvs_dist)

        for i in range(uvs_dist.shape[1]):
            uv_undist = np.copy(uvs_dist[:,i])

            if self.flag_undistort_fmin:
                # define the cost function
                def cost_func(uv_undist):
                    uv_dist_guess = self.distort_uv(uv_undist, cam_info)
                    duv = uvs_dist[:,i] - uv_dist_guess
                    return np.linalg.norm(duv)**2
                
                # use fmin to find the best uv_undist
                # uv_undist = fmin(cost_func, uv_undist, )
                uv_undist = minimize(cost_func, uv_undist).x
            else:
                flag_converge=False
                r_distort_orig = np.linalg.norm(uv_undist)
                r_undistort = np.copy(r_distort_orig)
                for iter in range(err_max_iter):
                    # Calculate what our guess would look like if it were distorted
                    uv_dist_guess = self.distort_uv(uv_undist, cam_info)

                    # Calculate the difference from the actual distorted point
                    duv = uvs_dist[:,i] - uv_dist_guess
                    
                    # If our guess is close enough to the actual point, we can stop
                    err_uv = np.linalg.norm(duv)
                    if np.abs(err_uv) < err_threshold:
                        flag_converge = True
                        break

                    # Update our guess
                    uv_undist += duv

                if flag_converge==False:
                    print('sad')
            
            uvs_undist[:,i] = uv_undist
        xys_undist = self.uv2xy(uvs_undist, cam_info)

        return xys_undist
