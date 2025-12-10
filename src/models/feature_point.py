"""
Feature Point Entity
Fields:
- featureId: string (unique identifier for the feature)
- position: Vector3 (3D position of the feature)
- descriptor: float[] (descriptor vector for the feature)
- trackingStatus: string (status of the feature tracking)
- observations: Observation[] (list of observations of this feature)
"""
from dataclasses import dataclass
from typing import List
from .simulation_environment import Vector3


@dataclass
class CameraIntrinsics:
    """Camera intrinsics for an observation."""
    fx: float  # Focal length in x
    fy: float  # Focal length in y
    cx: float  # Principal point x
    cy: float  # Principal point y
    width: int  # Image width
    height: int  # Image height
    distortion_coeffs: List[float]  # Distortion coefficients [k1, k2, p1, p2, k3, ...]


@dataclass
class Observation:
    """An observation of a feature point in an image."""
    observation_id: str
    image_path: str  # Path to the image file
    camera_pose: 'CameraPose'  # Pose of the camera when the observation was made
    keypoint_2d: 'Keypoint2D'  # 2D keypoint in the image
    timestamp: float  # Timestamp of the observation
    camera_intrinsics: CameraIntrinsics  # Camera intrinsics for this observation
    # Additional properties could include uncertainty, scale, etc.


@dataclass
class CameraPose:
    """Pose of a camera in the world coordinate system."""
    position: Vector3
    orientation: 'Quaternion'  # Using a quaternion for orientation
    # Additional properties could include pose covariance, etc.


@dataclass
class Keypoint2D:
    """A 2D keypoint in an image."""
    x: float  # x coordinate in pixels
    y: float  # y coordinate in pixels
    size: float  # Size of the keypoint
    angle: float  # Angle of the keypoint
    response: float  # Response of the keypoint detector
    octave: int  # Scale level where the keypoint was detected
    class_id: int = -1  # Class ID if applicable (for semantic features)


@dataclass
class FeaturePoint:
    """Represents a feature point in Isaac Sim/ROS perception."""
    feature_id: str
    position: Vector3  # 3D position of the feature in world coordinates
    descriptor: List[float]  # Descriptor vector (e.g., SIFT: 128-dim, ORB: 32-dim, etc.)
    tracking_status: str  # Status of the feature (e.g., "tracked", "lost", "new", "mature")
    
    # Optional fields
    observations: List[Observation] = None

    def __post_init__(self):
        if self.observations is None:
            self.observations = []

    def add_observation(self, observation: Observation):
        """Add an observation to this feature point."""
        self.observations.append(observation)

    def validate(self) -> bool:
        """Validate the feature point configuration."""
        # Check required fields
        if not self.feature_id or not isinstance(self.feature_id, str):
            raise ValueError("feature_id must be a non-empty string")
        
        if not isinstance(self.position, Vector3):
            raise ValueError("position must be a Vector3 object")
        
        if not isinstance(self.descriptor, list) or len(self.descriptor) == 0:
            raise ValueError("descriptor must be a non-empty list of floats")
        
        if not all(isinstance(d, (int, float)) for d in self.descriptor):
            raise ValueError("All elements of descriptor must be numbers")
        
        if not self.tracking_status or not isinstance(self.tracking_status, str):
            raise ValueError("tracking_status must be a non-empty string")
        
        # Validate tracking status
        valid_statuses = ["tracked", "lost", "new", "mature", "outlier"]
        if self.tracking_status.lower() not in [status.lower() for status in valid_statuses]:
            raise ValueError(f"tracking_status must be one of {valid_statuses}")
        
        # Validate optional observations if they exist
        if self.observations is not None:
            if not isinstance(self.observations, list):
                raise ValueError("observations must be a list of Observation objects")
            for observation in self.observations:
                if not isinstance(observation, Observation):
                    raise ValueError("All observations must be Observation objects")
        
        return True

    def get_descriptor_length(self) -> int:
        """Get the length of the descriptor vector."""
        return len(self.descriptor)

    def get_observation_count(self) -> int:
        """Get the number of times this feature has been observed."""
        return len(self.observations)

    def is_observable(self) -> bool:
        """Check if this feature is considered observable based on tracking status."""
        return self.tracking_status.lower() in ["tracked", "mature"]