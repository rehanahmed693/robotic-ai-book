"""
Robot Model Entity
Fields:
- modelId: string (unique identifier for the robot model)
- name: string (name of the robot model)
- urdfPath: string (path to the URDF file defining the robot)
- sdfPath: string (path to the SDF file defining the robot)
- jointConfigurations: JointConfiguration[] (configurations for robot joints)
- kinematicProperties: KinematicProperties object (kinematic details)
- dynamicProperties: DynamicProperties object (dynamic details)
"""
from dataclasses import dataclass
from typing import List, Optional
from .simulation_environment import Vector3, Quaternion


@dataclass
class JointConfiguration:
    """Configuration for a robot joint."""
    joint_name: str
    joint_type: str  # revolute, prismatic, fixed, etc.
    position: float
    velocity: float
    effort: float
    limits_lower: float
    limits_upper: float


@dataclass
class KinematicProperties:
    """Kinematic properties of the robot."""
    base_frame: str
    tip_frame: str
    joint_names: List[str]
    # Additional kinematic properties can be added as needed


@dataclass
class DynamicProperties:
    """Dynamic properties of the robot."""
    mass: float
    center_of_mass: Vector3
    inertia: List[float]  # 3x3 inertia matrix as 9 elements [ixx, ixy, ixz, iyy, iyz, izz]
    # Additional dynamic properties can be added as needed


@dataclass
class RobotModel:
    """Represents a robot model in Isaac Sim."""
    model_id: str
    name: str
    # Either urdf_path or sdf_path must be provided
    urdf_path: Optional[str] = None
    sdf_path: Optional[str] = None
    kinematic_properties: Optional[KinematicProperties] = None
    dynamic_properties: Optional[DynamicProperties] = None
    
    # These will be populated later
    joint_configurations: List[JointConfiguration] = None

    def __post_init__(self):
        if self.joint_configurations is None:
            self.joint_configurations = []
        
        # Validate that at least one of urdf_path or sdf_path is provided
        if not self.urdf_path and not self.sdf_path:
            raise ValueError("Either urdf_path or sdf_path must be provided")

    def add_joint_configuration(self, joint_config: JointConfiguration):
        """Add a joint configuration to the robot."""
        self.joint_configurations.append(joint_config)

    def validate(self) -> bool:
        """Validate the robot model."""
        # Check required fields
        if not self.model_id or not isinstance(self.model_id, str):
            raise ValueError("model_id must be a non-empty string")
        
        if not self.name or not isinstance(self.name, str):
            raise ValueError("name must be a non-empty string")
        
        # Either urdf_path or sdf_path must be provided
        if not self.urdf_path and not self.sdf_path:
            raise ValueError("Either urdf_path or sdf_path must be provided")
        
        if self.urdf_path and not isinstance(self.urdf_path, str):
            raise ValueError("urdf_path must be a string")
        
        if self.sdf_path and not isinstance(self.sdf_path, str):
            raise ValueError("sdf_path must be a string")
        
        # Validate properties if they exist
        if self.kinematic_properties and not isinstance(self.kinematic_properties, KinematicProperties):
            raise ValueError("kinematic_properties must be a KinematicProperties object")
        
        if self.dynamic_properties and not isinstance(self.dynamic_properties, DynamicProperties):
            raise ValueError("dynamic_properties must be a DynamicProperties object")
        
        return True