"""
Visual Mapping Service
Implementation based on the VisualMap and FeaturePoint entities
"""
from typing import Dict, List, Optional
from ..models.visual_map import VisualMap, FeaturePoint, PoseGraph, PoseNode, PoseEdge, PoseTransform
from ..models.simulation_environment import Vector3, Quaternion


class VisualMappingService:
    """
    Service class for creating and managing visual maps in Isaac ROS.
    """
    
    def __init__(self):
        self.maps: Dict[str, VisualMap] = {}
        self.current_map_id: Optional[str] = None

    def create_map(
        self,
        map_id: str,
        name: str,
        origin: Vector3,
        resolution: float,
        width: int,
        height: int
    ) -> VisualMap:
        """
        Create a new visual map with the specified parameters.
        """
        # Initialize the map data with unknown values (-1)
        data = [-1] * (width * height)
        
        # Create the visual map
        visual_map = VisualMap(
            map_id=map_id,
            name=name,
            origin=origin,
            resolution=resolution,
            width=width,
            height=height,
            data=data
        )
        
        # Validate the map
        visual_map.validate()
        
        # Store the map
        self.maps[map_id] = visual_map
        self.current_map_id = map_id
        
        return visual_map

    def get_map(self, map_id: str) -> Optional[VisualMap]:
        """
        Retrieve a visual map by its ID.
        """
        return self.maps.get(map_id)

    def delete_map(self, map_id: str) -> bool:
        """
        Delete a visual map by its ID.
        """
        if map_id in self.maps:
            # If we're deleting the current map, clear the current map ID
            if self.current_map_id == map_id:
                self.current_map_id = None
            del self.maps[map_id]
            return True
        return False

    def update_map_cell(self, map_id: str, x: int, y: int, value: int) -> bool:
        """
        Update a specific cell in the map with an occupancy value.
        Value should be between 0-100 (0 = free, 100 = occupied) or -1 (unknown).
        """
        visual_map = self.get_map(map_id)
        if visual_map is None:
            return False

        try:
            index = visual_map.get_grid_index(x, y)
        except IndexError:
            return False

        if value != -1 and (value < 0 or value > 100):
            return False

        visual_map.data[index] = value
        return True

    def add_feature_point(self, map_id: str, feature: FeaturePoint) -> bool:
        """
        Add a feature point to an existing map.
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            visual_map.features.append(feature)
            return True
        return False

    def update_pose_graph(self, map_id: str, pose_graph: PoseGraph) -> bool:
        """
        Update the pose graph for a map.
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            visual_map.pose_graph = pose_graph
            return True
        return False

    def add_pose_node(self, map_id: str, pose_node: PoseNode) -> bool:
        """
        Add a pose node to the map's pose graph.
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            visual_map.pose_graph.nodes.append(pose_node)
            return True
        return False

    def add_pose_edge(self, map_id: str, pose_edge: PoseEdge) -> bool:
        """
        Add a pose edge to the map's pose graph.
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            visual_map.pose_graph.edges.append(pose_edge)
            return True
        return False

    def get_occupancy(self, map_id: str, x: int, y: int) -> Optional[int]:
        """
        Get the occupancy value of a specific cell in the map.
        Returns the value (0-100 or -1) or None if the coordinates are invalid.
        """
        visual_map = self.get_map(map_id)
        if visual_map is None:
            return None

        try:
            index = visual_map.get_grid_index(x, y)
            return visual_map.data[index]
        except IndexError:
            return None

    def ray_cast(self, map_id: str, start: Vector3, end: Vector3) -> List[Vector3]:
        """
        Perform ray casting between two points in the map.
        Returns a list of grid positions that the ray passes through.
        """
        visual_map = self.get_map(map_id)
        if visual_map is None:
            return []

        # Convert world coordinates to grid coordinates
        start_x = int((start.x - visual_map.origin.x) / visual_map.resolution)
        start_y = int((start.y - visual_map.origin.y) / visual_map.resolution)
        end_x = int((end.x - visual_map.origin.x) / visual_map.resolution)
        end_y = int((end.y - visual_map.origin.y) / visual_map.resolution)

        # Bresenham's line algorithm
        points = []
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        x_step = 1 if start_x < end_x else -1
        y_step = 1 if start_y < end_y else -1
        error = dx - dy

        x, y = start_x, start_y
        while True:
            # Check if the current point is within map bounds
            if 0 <= x < visual_map.width and 0 <= y < visual_map.height:
                world_pos = visual_map.get_2d_position(x, y)
                points.append(world_pos)

            if x == end_x and y == end_y:
                break

            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step

        return points

    def get_map_bounds(self, map_id: str) -> Optional[tuple]:
        """
        Get the world coordinates of the map boundaries (min_x, min_y, max_x, max_y).
        """
        visual_map = self.get_map(map_id)
        if visual_map is None:
            return None

        min_x = visual_map.origin.x
        min_y = visual_map.origin.y
        max_x = min_x + visual_map.width * visual_map.resolution
        max_y = min_y + visual_map.height * visual_map.resolution

        return (min_x, min_y, max_x, max_y)

    def get_feature_points(self, map_id: str) -> List[FeaturePoint]:
        """
        Get all feature points in a map.
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            return visual_map.features
        return []

    def get_pose_graph(self, map_id: str) -> Optional[PoseGraph]:
        """
        Get the pose graph of a map.
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            return visual_map.pose_graph
        return None

    def set_current_map(self, map_id: str) -> bool:
        """
        Set the specified map as the current one.
        """
        if map_id in self.maps:
            self.current_map_id = map_id
            return True
        return False

    def get_current_map(self) -> Optional[VisualMap]:
        """
        Get the currently selected map.
        """
        if self.current_map_id:
            return self.maps.get(self.current_map_id)
        return None

    def list_maps(self) -> List[str]:
        """
        List all available map IDs.
        """
        return list(self.maps.keys())

    def clear_map(self, map_id: str) -> bool:
        """
        Clear all data in a map (set all cells to unknown).
        """
        visual_map = self.get_map(map_id)
        if visual_map:
            visual_map.data = [-1] * len(visual_map.data)
            visual_map.features = []
            visual_map.pose_graph = PoseGraph([], [])
            return True
        return False