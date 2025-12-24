import numpy as np
import cv2

class SimpleTracker:
    def __init__(self):
        self.tracks = {}
        self.next_id = 1
        self.max_disappeared = 10
        self.max_distance = 100
        
    def _calculate_distance(self, box1, box2):
        """Calculate center distance between two bounding boxes"""
        center1 = [(box1[0] + box1[2]) / 2, (box1[1] + box1[3]) / 2]
        center2 = [(box2[0] + box2[2]) / 2, (box2[1] + box2[3]) / 2]
        return np.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)
    
    def update(self, detections):
        if not detections:
            # Update disappeared counter for existing tracks
            to_remove = []
            for track_id in self.tracks:
                self.tracks[track_id]['disappeared'] += 1
                if self.tracks[track_id]['disappeared'] > self.max_disappeared:
                    to_remove.append(track_id)
            
            for track_id in to_remove:
                del self.tracks[track_id]
            
            return []
        
        # If no existing tracks, create new ones
        if not self.tracks:
            for det in detections:
                self.tracks[self.next_id] = {
                    'bbox': det['bbox'],
                    'class': det['class'],
                    'disappeared': 0
                }
                self.next_id += 1
        else:
            # Match detections to existing tracks
            detection_boxes = [det['bbox'] for det in detections]
            track_ids = list(self.tracks.keys())
            track_boxes = [self.tracks[tid]['bbox'] for tid in track_ids]
            
            # Simple distance-based matching
            used_detections = set()
            used_tracks = set()
            
            for i, track_id in enumerate(track_ids):
                min_distance = float('inf')
                best_detection = -1
                
                for j, det_box in enumerate(detection_boxes):
                    if j in used_detections:
                        continue
                    
                    distance = self._calculate_distance(track_boxes[i], det_box)
                    if distance < min_distance and distance < self.max_distance:
                        min_distance = distance
                        best_detection = j
                
                if best_detection != -1:
                    # Update existing track
                    self.tracks[track_id]['bbox'] = detection_boxes[best_detection]
                    self.tracks[track_id]['class'] = detections[best_detection]['class']
                    self.tracks[track_id]['disappeared'] = 0
                    used_detections.add(best_detection)
                    used_tracks.add(track_id)
                else:
                    # Track not matched, increment disappeared counter
                    self.tracks[track_id]['disappeared'] += 1
            
            # Create new tracks for unmatched detections
            for j, det in enumerate(detections):
                if j not in used_detections:
                    self.tracks[self.next_id] = {
                        'bbox': det['bbox'],
                        'class': det['class'],
                        'disappeared': 0
                    }
                    self.next_id += 1
            
            # Remove tracks that have disappeared for too long
            to_remove = []
            for track_id in self.tracks:
                if self.tracks[track_id]['disappeared'] > self.max_disappeared:
                    to_remove.append(track_id)
            
            for track_id in to_remove:
                del self.tracks[track_id]
        
        # Return active tracks
        tracked_objects = []
        for track_id, track_data in self.tracks.items():
            if track_data['disappeared'] == 0:  # Only return currently detected tracks
                tracked_objects.append({
                    'track_id': track_id,
                    'bbox': track_data['bbox'],
                    'class': track_data['class']
                })
        
        return tracked_objects

# Alias for compatibility
DeepSORTTracker = SimpleTracker
