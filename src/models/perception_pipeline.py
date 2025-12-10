"""
Perception Pipeline Entity
Fields:
- pipelineId: string (unique identifier for the pipeline)
- name: string (name of the perception pipeline)
- description: string (description of the pipeline)
- vslamAlgorithm: string (the VSLAM algorithm used)
- parameters: VSLAMParameters object (configuration for the algorithm)
- inputSource: string (source of sensor data)
- outputFormat: string (format of perception output)
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class VSLAMParameters:
    """Parameters specific to the VSLAM algorithm."""
    # Visual Odometry parameters
    max_num_features: int = 2000
    min_num_features: int = 100
    feature_tracker_type: str = "fast"  # Options: fast, shi_tomasi, etc.
    matcher_type: str = "brute_force"  # Options: brute_force, flann, etc.
    
    # Bundle adjustment parameters
    bundle_adjustment: bool = True
    max_iterations: int = 100
    
    # Loop closure parameters
    loop_closure_detection: bool = True
    loop_closure_threshold: float = 0.5
    
    # Mapping parameters
    map_resolution: float = 0.05  # meters per pixel
    keyframe_selection_threshold: float = 0.2  # distance threshold for keyframes
    
    # Additional parameters can be added as needed
    extra_params: Optional[Dict[str, Any]] = None


@dataclass
class PerceptionPipeline:
    """Represents a perception pipeline in Isaac ROS."""
    pipeline_id: str
    name: str
    description: str
    vslam_algorithm: str  # Name of the algorithm used (e.g., "ORB-SLAM", "RTAB-Map", "Isaac VSLAM")
    input_source: str  # Source of sensor data (e.g., "/camera/rgb/image_rect_color", "/depth/image_rect")
    output_format: str  # Format of perception output (e.g., "pose_graph", "point_cloud", "feature_map")
    
    # Optional parameters
    parameters: Optional[VSLAMParameters] = None

    def __post_init__(self):
        if self.parameters is None:
            self.parameters = VSLAMParameters()

    def validate(self) -> bool:
        """Validate the perception pipeline configuration."""
        # Check required fields
        if not self.pipeline_id or not isinstance(self.pipeline_id, str):
            raise ValueError("pipeline_id must be a non-empty string")
        
        if not self.name or not isinstance(self.name, str):
            raise ValueError("name must be a non-empty string")
        
        if not self.description or not isinstance(self.description, str):
            raise ValueError("description must be a non-empty string")
        
        if not self.vslam_algorithm or not isinstance(self.vslam_algorithm, str):
            raise ValueError("vslam_algorithm must be a non-empty string")
        
        if not self.input_source or not isinstance(self.input_source, str):
            raise ValueError("input_source must be a non-empty string")
        
        if not self.output_format or not isinstance(self.output_format, str):
            raise ValueError("output_format must be a non-empty string")
        
        # Validate parameters if they exist
        if self.parameters and not isinstance(self.parameters, VSLAMParameters):
            raise ValueError("parameters must be a VSLAMParameters object")
        
        # Validate VSLAM algorithm is supported
        supported_algorithms = [
            "orb_slam", "rtab_map", "vdo_slam", "dvo_slam", 
            "rovio", "okvis", "lsd_slam", "elastic_fusion", "isaac_vslam"
        ]
        if self.vslam_algorithm.lower() not in [alg.lower() for alg in supported_algorithms]:
            raise ValueError(f"vslam_algorithm must be one of {supported_algorithms}")
        
        return True

    def get_output_topic(self) -> str:
        """Generate the ROS topic name for this pipeline's output."""
        return f"/perception/{self.pipeline_id.lower()}/{self.output_format.lower()}"