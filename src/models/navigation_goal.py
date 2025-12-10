"""
Navigation Goal Entity
Fields:
- goalId: string (unique identifier for the navigation goal)
- targetPose: Pose (target position and orientation)
- tolerance: float (acceptable tolerance for reaching the goal)
- frameId: string (coordinate frame of the goal)
- priority: int (priority level of the goal)
- status: string (current status of the goal)
"""
from dataclasses import dataclass
from typing import Optional
from ..models.simulation_environment import Vector3, Quaternion


@dataclass
class Pose:
    """Represents a position and orientation in 3D space."""
    position: Vector3
    orientation: Quaternion


@dataclass
class NavigationGoal:
    """Represents a navigation goal in Isaac Sim/ROS Nav2."""
    goal_id: str
    target_pose: Pose
    frame_id: str  # Coordinate frame for the target pose
    tolerance: float  # Acceptable tolerance for reaching the goal (in meters)
    priority: int  # Priority level of the goal (0 = highest, higher numbers = lower priority)
    
    # Optional fields
    status: str = "pending"  # Current status of the goal (pending, active, succeeded, failed, canceled)

    def validate(self) -> bool:
        """Validate the navigation goal configuration."""
        # Check required fields
        if not self.goal_id or not isinstance(self.goal_id, str):
            raise ValueError("goal_id must be a non-empty string")
        
        if not isinstance(self.target_pose, Pose):
            raise ValueError("target_pose must be a Pose object")
        
        if not self.frame_id or not isinstance(self.frame_id, str):
            raise ValueError("frame_id must be a non-empty string")
        
        if not isinstance(self.tolerance, (int, float)) or self.tolerance < 0:
            raise ValueError("tolerance must be a non-negative number")
        
        if not isinstance(self.priority, int) or self.priority < 0:
            raise ValueError("priority must be a non-negative integer")
        
        # Validate status if it exists
        valid_statuses = ["pending", "active", "succeeded", "failed", "canceled", "preempted"]
        if self.status and self.status not in valid_statuses:
            raise ValueError(f"status must be one of {valid_statuses}")
        
        return True

    def set_status(self, status: str):
        """Set the status of the navigation goal."""
        valid_statuses = ["pending", "active", "succeeded", "failed", "canceled", "preempted"]
        if status not in valid_statuses:
            raise ValueError(f"status must be one of {valid_statuses}")
        self.status = status

    def is_complete(self) -> bool:
        """Check if the goal is complete (succeeded or failed)."""
        return self.status in ["succeeded", "failed", "canceled"]