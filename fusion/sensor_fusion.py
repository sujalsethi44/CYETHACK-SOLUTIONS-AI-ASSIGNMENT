import numpy as np

class SensorFusion:
    def __init__(self, image_width=1242, image_height=375):
        self.image_width = image_width
        self.image_height = image_height
    
    def associate_lidar_with_bbox(self, bbox, projected_points, depths):
        """Associate LiDAR points with 2D bounding box and estimate distance"""
        x1, y1, x2, y2 = bbox
        
        # Find points inside the bounding box
        mask = (
            (projected_points[:, 0] >= x1) & 
            (projected_points[:, 0] <= x2) &
            (projected_points[:, 1] >= y1) & 
            (projected_points[:, 1] <= y2) &
            (projected_points[:, 0] >= 0) & 
            (projected_points[:, 0] < self.image_width) &
            (projected_points[:, 1] >= 0) & 
            (projected_points[:, 1] < self.image_height) &
            (depths > 0)
        )
        
        associated_depths = depths[mask]
        
        if len(associated_depths) > 0:
            # Use median depth for robust estimation
            distance = np.median(associated_depths)
            return distance, len(associated_depths)
        
        return None, 0
    
    def fuse_detections_with_lidar(self, tracked_objects, projected_points, depths):
        """Fuse tracked objects with LiDAR data"""
        fused_objects = []
        
        for obj in tracked_objects:
            distance, num_points = self.associate_lidar_with_bbox(
                obj['bbox'], projected_points, depths
            )
            
            fused_obj = obj.copy()
            fused_obj['distance'] = distance
            fused_obj['lidar_points'] = num_points
            fused_objects.append(fused_obj)
        
        return fused_objects
