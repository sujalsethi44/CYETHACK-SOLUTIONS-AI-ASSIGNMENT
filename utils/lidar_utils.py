import numpy as np

def load_velodyne_points(velodyne_path):
    """Load LiDAR points from KITTI .bin file"""
    points = np.fromfile(velodyne_path, dtype=np.float32).reshape(-1, 4)
    return points[:, :3]  # Return only XYZ coordinates

def filter_points_behind_camera(points):
    """Remove points behind the camera (negative X in camera coordinates)"""
    return points[points[:, 0] > 0]

def load_kitti_calibration(calib_path):
    """Load KITTI calibration matrices"""
    calib = {}
    with open(calib_path, 'r') as f:
        for line in f.readlines():
            line = line.strip()
            if not line:
                continue
            
            if ':' in line:
                key, value = line.split(':', 1)
                calib[key] = np.array([float(x) for x in value.split()])
            else:
                # Handle lines like "R_rect" and "Tr_velo_cam" without colons
                parts = line.split()
                if len(parts) > 1:
                    key = parts[0]
                    values = parts[1:]
                    calib[key] = np.array([float(x) for x in values])
    
    # P2 is the projection matrix for left color camera
    P2 = calib['P2'].reshape(3, 4)
    
    # Tr_velo_to_cam transforms from Velodyne to camera coordinates
    Tr_velo_to_cam = np.eye(4)
    if 'Tr_velo_cam' in calib:
        Tr_velo_to_cam[:3, :4] = calib['Tr_velo_cam'].reshape(3, 4)
    elif 'Tr_velo_to_cam' in calib:
        Tr_velo_to_cam[:3, :4] = calib['Tr_velo_to_cam'].reshape(3, 4)
    
    # R0_rect rectifies the camera
    R0_rect = np.eye(4)
    if 'R_rect' in calib:
        R0_rect[:3, :3] = calib['R_rect'].reshape(3, 3)
    elif 'R0_rect' in calib:
        R0_rect[:3, :3] = calib['R0_rect'].reshape(3, 3)
    
    return P2, Tr_velo_to_cam, R0_rect

def project_lidar_to_camera(points, P2, Tr_velo_to_cam, R0_rect):
    """Project 3D LiDAR points to 2D camera image"""
    # Convert to homogeneous coordinates
    points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
    
    # Transform: Velodyne -> Camera -> Rectified -> Image
    points_cam = (Tr_velo_to_cam @ points_homo.T).T
    points_rect = (R0_rect @ np.hstack([points_cam[:, :3], np.ones((points_cam.shape[0], 1))]).T).T
    points_img = (P2 @ points_rect.T).T
    
    # Normalize by depth
    points_img = points_img / points_img[:, 2:3]
    
    return points_img[:, :2], points_rect[:, 2]  # Return 2D points and depths
