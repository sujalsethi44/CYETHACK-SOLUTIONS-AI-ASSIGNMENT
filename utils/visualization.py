import cv2
import numpy as np

class Visualizer:
    def __init__(self):
        self.colors = {
            'Car': (0, 255, 0),      # Green
            'Pedestrian': (255, 0, 0) # Blue
        }
    
    def draw_bbox_with_info(self, image, obj):
        """Draw bounding box with class, ID, and distance information"""
        x1, y1, x2, y2 = [int(coord) for coord in obj['bbox']]
        track_id = obj['track_id']
        class_name = obj['class']
        distance = obj.get('distance')
        
        # Get color for class
        color = self.colors.get(class_name, (255, 255, 255))
        
        # Draw bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        
        # Prepare text
        if distance is not None:
            text = f"{class_name} | ID: {track_id} | {distance:.1f}m"
        else:
            text = f"{class_name} | ID: {track_id} | No LiDAR"
        
        # Calculate text size and background
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Draw text background
        cv2.rectangle(image, (x1, y1 - text_height - 10), 
                     (x1 + text_width + 5, y1), color, -1)
        
        # Draw text
        cv2.putText(image, text, (x1 + 2, y1 - 5), font, font_scale, (0, 0, 0), thickness)
        
        return image
    
    def visualize_frame(self, image, fused_objects):
        """Visualize all objects in a frame"""
        vis_image = image.copy()
        
        for obj in fused_objects:
            vis_image = self.draw_bbox_with_info(vis_image, obj)
        
        return vis_image
