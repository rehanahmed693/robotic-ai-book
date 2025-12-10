"""
Visual Map Entity
Fields:
- mapId: string (unique identifier for the map)
- name: string (name of the map)
- origin: Vector3 (origin point of the map)
- resolution: float (resolution of the map in meters per pixel)
- width: int (width of the map in pixels)
- height: int (height of the map in pixels)
- data: uint8[] (array of occupancy values)
- features: FeaturePoint[] (list of visual features in the map)
- poseGraph: PoseGraph (graph of robot poses)
"""
from dataclasses import dataclass
from typing import List
import numpy as np
from .simulation_environment import Vector3, Quaternion


@dataclass
class PoseNode:
    """A node in the pose graph representing a robot pose at a specific time."""
    pose_id: str
    position: Vector3
    orientation: Quaternion
    timestamp: float  # seconds since epoch
    # Additional properties could include covariance, etc.


@dataclass
class PoseEdge:
    """An edge in the pose graph representing a spatial relationship between two poses."""
    from_pose_id: str
    to_pose_id: str
    relative_transform: 'PoseTransform'  # Transform from from_pose to to_pose
    covariance: List[float]  # 6x6 covariance matrix as 36-element list
    # Additional properties could include transformation uncertainty, etc.


@dataclass
class PoseTransform:
    """A transform between two poses, used in the pose graph."""
    position: Vector3
    orientation: Quaternion


@dataclass
class PoseGraph:
    """Graph of robot poses forming the map's reference frame."""
    nodes: List[PoseNode]
    edges: List[PoseEdge]
    
    def __post_init__(self):
        if self.nodes is None:
            self.nodes = []
        if self.edges is None:
            self.edges = []


@dataclass
class FeaturePoint:
    """A visual feature point in the map."""
    feature_id: str
    position: Vector3  # 3D position in map coordinates
    descriptor: List[float]  # Descriptor vector (e.g., SIFT, ORB, etc.)
    keypoint: 'Keypoint2D'  # 2D keypoint in the image where this feature was observed
    # Additional properties could include tracking status, observability, etc.


@dataclass
class Keypoint2D:
    """A 2D keypoint in an image."""
    x: float  # x coordinate in pixels
    y: float  # y coordinate in pixels
    size: float  # Size of the keypoint
    angle: float  # Angle of the keypoint
    response: float  # Response of the keypoint detector
    octave: int  # Scale level where the keypoint was detected


@dataclass
class VisualMap:
    """Represents a visual map in Isaac Sim/ROS."""
    map_id: str
    name: str
    origin: Vector3
    resolution: float  # meters per pixel
    width: int  # pixels
    height: int  # pixels
    data: List[int]  # Occupancy grid data (0-100, -1 for unknown)
    
    # Optional fields
    features: List[FeaturePoint] = None
    pose_graph: PoseGraph = None

    def __post_init__(self):
        if self.features is None:
            self.features = []
        if self.pose_graph is None:
            self.pose_graph = PoseGraph([], [])
        
        # Validate that data array has the correct size
        expected_size = self.width * self.height
        if len(self.data) != expected_size:
            raise ValueError(f"Data array size ({len(self.data)}) does not match "
                           f"expected size ({expected_size}) for width={self.width}, "
                           f"height={self.height}")

    def validate(self) -> bool:
        """Validate the visual map configuration."""
        # Check required fields
        if not self.map_id or not isinstance(self.map_id, str):
            raise ValueError("map_id must be a non-empty string")
        
        if not self.name or not isinstance(self.name, str):
            raise ValueError("name must be a non-empty string")
        
        if not isinstance(self.origin, Vector3):
            raise ValueError("origin must be a Vector3 object")
        
        if not isinstance(self.resolution, (int, float)) or self.resolution <= 0:
            raise ValueError("resolution must be a positive number")
        
        if not isinstance(self.width, int) or self.width <= 0:
            raise ValueError("width must be a positive integer")
        
        if not isinstance(self.height, int) or self.height <= 0:
            raise ValueError("height must be a positive integer")
        
        if not isinstance(self.data, list) or len(self.data) != self.width * self.height:
            raise ValueError(f"data must be a list of {self.width * self.height} elements")
        
        # Validate that all values in data are valid occupancy values (-1 to 100)
        for value in self.data:
            if not isinstance(value, (int, float)) or (value != -1 and (value < 0 or value > 100)):
                raise ValueError("All values in data must be -1 (unknown) or between 0-100 (occupied)")
        
        # Validate optional fields if they exist
        if self.features is not None:
            if not isinstance(self.features, list):
                raise ValueError("features must be a list of FeaturePoint objects")
            for feature in self.features:
                if not isinstance(feature, FeaturePoint):
                    raise ValueError("All features must be FeaturePoint objects")
        
        if self.pose_graph is not None and not isinstance(self.pose_graph, PoseGraph):
            raise ValueError("pose_graph must be a PoseGraph object")
        
        return True

    def get_2d_position(self, x: int, y: int) -> Vector3:
        """Convert grid coordinates to world coordinates."""
        world_x = self.origin.x + (x + 0.5) * self.resolution
        world_y = self.origin.y + (y + 0.5) * self.resolution
        world_z = self.origin.z
        return Vector3(world_x, world_y, world_z)

    def get_grid_index(self, x: int, y: int) -> int:
        """Convert 2D grid coordinates to 1D array index."""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            raise IndexError(f"Grid coordinates ({x}, {y}) out of bounds for map of size {self.width}x{self.height}")
        return y * self.width + x

    def get_grid_coordinates(self, index: int) -> tuple:
        """Convert 1D array index to 2D grid coordinates."""
        if index < 0 or index >= len(self.data):
            raise IndexError(f"Index {index} out of bounds for map data of size {len(self.data)}")
        x = index % self.width
        y = index // self.width
        return (x, y)