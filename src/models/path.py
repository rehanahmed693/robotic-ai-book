"""
Path Entity
Fields:
- pathId: string (unique identifier for the path)
- poses: Pose[] (array of poses that make up the path)
- frameId: string (coordinate frame of the path)
- sourceMap: string (map from which the path was generated)
"""
from dataclasses import dataclass
from typing import List
from .navigation_plan import Pose


@dataclass
class Path:
    """Represents a navigation path in Isaac Sim/ROS Nav2."""
    path_id: str
    poses: List[Pose]
    frame_id: str  # Coordinate frame for the path
    source_map: str  # Map from which the path was generated
    
    def __post_init__(self):
        if self.poses is None:
            self.poses = []

    def add_pose(self, pose: Pose):
        """Add a pose to the path."""
        if not isinstance(pose, Pose):
            raise ValueError("pose must be a Pose object")
        self.poses.append(pose)

    def insert_pose(self, index: int, pose: Pose):
        """Insert a pose at a specific index in the path."""
        if not isinstance(pose, Pose):
            raise ValueError("pose must be a Pose object")
        if index < 0 or index > len(self.poses):
            raise IndexError("index out of range")
        self.poses.insert(index, pose)

    def remove_pose(self, index: int):
        """Remove a pose at a specific index from the path."""
        if index < 0 or index >= len(self.poses):
            raise IndexError("index out of range")
        del self.poses[index]

    def clear_poses(self):
        """Clear all poses from the path."""
        self.poses = []

    def get_length(self) -> float:
        """Calculate the total length of the path."""
        if len(self.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(self.poses)):
            prev_pos = self.poses[i-1].position
            curr_pos = self.poses[i].position
            dx = curr_pos.x - prev_pos.x
            dy = curr_pos.y - prev_pos.y
            dz = curr_pos.z - prev_pos.z
            segment_length = (dx**2 + dy**2 + dz**2) ** 0.5
            total_length += segment_length
        
        return total_length

    def get_num_poses(self) -> int:
        """Get the number of poses in the path."""
        return len(self.poses)

    def get_start_pose(self) -> Pose:
        """Get the first pose in the path."""
        if len(self.poses) == 0:
            raise ValueError("Path is empty")
        return self.poses[0]

    def get_end_pose(self) -> Pose:
        """Get the last pose in the path."""
        if len(self.poses) == 0:
            raise ValueError("Path is empty")
        return self.poses[-1]

    def validate(self) -> bool:
        """Validate the path configuration."""
        # Check required fields
        if not self.path_id or not isinstance(self.path_id, str):
            raise ValueError("path_id must be a non-empty string")
        
        if not isinstance(self.poses, list):
            raise ValueError("poses must be a list of Pose objects")
        
        if not self.frame_id or not isinstance(self.frame_id, str):
            raise ValueError("frame_id must be a non-empty string")
        
        if not self.source_map or not isinstance(self.source_map, str):
            raise ValueError("source_map must be a non-empty string")
        
        # Validate that all elements in poses are Pose objects
        for pose in self.poses:
            if not isinstance(pose, Pose):
                raise ValueError("All elements in poses must be Pose objects")
        
        return True

    def smooth_path(self, smoothing_factor: float = 0.1) -> 'Path':
        """
        Create a smoothed version of the path using a simple smoothing algorithm.
        The smoothing_factor controls how much smoothing occurs (0.0 = no smoothing, 1.0 = maximum smoothing)
        """
        if len(self.poses) < 3:
            # Can't smooth a path with fewer than 3 poses
            return Path(self.path_id + "_smoothed", self.poses.copy(), self.frame_id, self.source_map)
        
        smoothed_poses = [self.poses[0]]  # First pose remains the same
        
        for i in range(1, len(self.poses) - 1):
            # Calculate smoothed position as a weighted average
            # of the current, previous, and next positions
            prev_pos = self.poses[i-1].position
            curr_pos = self.poses[i].position
            next_pos = self.poses[i+1].position
            
            # New position is: current + alpha * (prev + next - 2*current)
            new_x = curr_pos.x + smoothing_factor * (prev_pos.x + next_pos.x - 2 * curr_pos.x)
            new_y = curr_pos.y + smoothing_factor * (prev_pos.y + next_pos.y - 2 * curr_pos.y)
            new_z = curr_pos.z + smoothing_factor * (prev_pos.z + next_pos.z - 2 * curr_pos.z)
            
            # For orientation, we could implement quaternion slerping,
            # but for simplicity, we'll keep the original orientation
            new_pose = Pose(
                position=type(curr_pos)(new_x, new_y, new_z),
                orientation=self.poses[i].orientation
            )
            smoothed_poses.append(new_pose)
        
        smoothed_poses.append(self.poses[-1])  # Last pose remains the same
        
        return Path(self.path_id + "_smoothed", smoothed_poses, self.frame_id, self.source_map)