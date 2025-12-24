import os
import cv2
import time
import glob
import numpy as np
from pathlib import Path

from detector import YOLODetector
from tracker import DeepSORTTracker
from fusion import SensorFusion
from utils import load_velodyne_points, filter_points_behind_camera, load_kitti_calibration, project_lidar_to_camera
from utils.visualization import Visualizer

class PerceptionPipeline:
    def __init__(self, dataset_path):
        self.dataset_path = Path(dataset_path)
        self.detector = YOLODetector()
        self.tracker = DeepSORTTracker()
        self.fusion = SensorFusion()
        self.visualizer = Visualizer()
        
        # Load calibration
        calib_path = self.dataset_path / "data_tracking_calib" / "training" / "calib" / "0000.txt"
        self.P2, self.Tr_velo_to_cam, self.R0_rect = load_kitti_calibration(calib_path)
        
        # Get image paths
        self.image_paths = sorted(glob.glob(str(self.dataset_path / "data_tracking_image_2" / "training" / "image_02" / "0000" / "*.png")))
        
        # Velodyne paths (will be empty until you download velodyne data)
        velodyne_dir = self.dataset_path / "data_tracking_velodyne" / "training" / "velodyne" / "0000"
        if velodyne_dir.exists():
            self.velodyne_paths = sorted(glob.glob(str(velodyne_dir / "*.bin")))
        else:
            self.velodyne_paths = []
            print("Velodyne data not found. Running without LiDAR fusion.")
        
        print(f"Found {len(self.image_paths)} images and {len(self.velodyne_paths)} velodyne files")
    
    def process_frame(self, image_path, velodyne_path=None):
        """Process a single frame"""
        # Load image
        image = cv2.imread(image_path)
        if image is None:
            return None, None
        
        # Object detection
        detections = self.detector.detect(image)
        
        # Multi-object tracking
        tracked_objects = self.tracker.update(detections)
        
        # Process LiDAR data if available
        if velodyne_path and os.path.exists(velodyne_path):
            # Load and process LiDAR data
            lidar_points = load_velodyne_points(velodyne_path)
            lidar_points = filter_points_behind_camera(lidar_points)
            
            # Project LiDAR to camera
            projected_points, depths = project_lidar_to_camera(
                lidar_points, self.P2, self.Tr_velo_to_cam, self.R0_rect
            )
            
            # Sensor fusion
            fused_objects = self.fusion.fuse_detections_with_lidar(
                tracked_objects, projected_points, depths
            )
        else:
            # No LiDAR data - just add None distance to tracked objects
            fused_objects = []
            for obj in tracked_objects:
                fused_obj = obj.copy()
                fused_obj['distance'] = None
                fused_obj['lidar_points'] = 0
                fused_objects.append(fused_obj)
        
        # Visualization
        vis_image = self.visualizer.visualize_frame(image, fused_objects)
        
        return vis_image, fused_objects
    
    def run_pipeline(self, output_video_path="output_video.mp4"):
        """Run the complete perception pipeline"""
        if not self.image_paths:
            print("No image data found. Check dataset path.")
            return
        
        # Initialize video writer
        first_image = cv2.imread(self.image_paths[0])
        height, width = first_image.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_video_path, fourcc, 10.0, (width, height))
        
        # Performance tracking
        frame_times = []
        total_frames = len(self.image_paths)
        has_velodyne = len(self.velodyne_paths) > 0
        
        print(f"Processing {total_frames} frames...")
        if not has_velodyne:
            print("Running without LiDAR data - only detection and tracking")
        
        for i in range(total_frames):
            start_time = time.time()
            
            # Get velodyne path if available
            velodyne_path = self.velodyne_paths[i] if has_velodyne and i < len(self.velodyne_paths) else None
            
            # Process frame
            vis_image, fused_objects = self.process_frame(
                self.image_paths[i], velodyne_path
            )
            
            if vis_image is not None:
                # Write to video
                out.write(vis_image)
                
                # Calculate FPS
                frame_time = time.time() - start_time
                frame_times.append(frame_time)
                fps = 1.0 / frame_time
                
                # Print progress
                if i % 10 == 0:
                    print(f"Frame {i}/{total_frames}, FPS: {fps:.2f}, Objects: {len(fused_objects)}")
        
        # Cleanup
        out.release()
        
        # Performance summary
        avg_fps = 1.0 / np.mean(frame_times)
        print(f"\nPipeline completed!")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Total processing time: {sum(frame_times):.2f}s")
        print(f"Output saved to: {output_video_path}")

def main():
    # Dataset path - modify this to your KITTI dataset location
    dataset_path = "dataset"
    
    if not os.path.exists(dataset_path):
        print(f"Dataset path '{dataset_path}' not found.")
        print("Please ensure your dataset follows this structure:")
        print("dataset/")
        print("  data_tracking_image_2/training/image_02/0000/")
        print("  data_tracking_calib/training/calib/0000.txt")
        print("  data_tracking_velodyne/training/velodyne/0000/ (optional)")
        return
    
    # Initialize and run pipeline
    pipeline = PerceptionPipeline(dataset_path)
    pipeline.run_pipeline("kitti_perception_output.mp4")

if __name__ == "__main__":
    main()
