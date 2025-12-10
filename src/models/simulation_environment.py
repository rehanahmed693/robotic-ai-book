"""
Simulation Environment Entity
Fields:
- environmentId: string (unique identifier for the simulation environment)
- name: string (human-readable name)
- description: string (detailed description of the environment)
- physicsProperties: PhysicsProperties object (gravity, friction, etc.)
- robotModels: RobotModel[] (list of robot models in the environment)
- sensorConfigurations: SensorConfiguration[] (sensor setup for the environment)
- lighting: LightingConfiguration object (lighting setup)
- assets: string[] (file paths to assets used in the environment)
"""
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum


@dataclass
class Vector3:
    """Represents a 3D vector with x, y, z coordinates."""
    x: float
    y: float
    z: float


@dataclass
class Quaternion:
    """Represents a quaternion with x, y, z, w components."""
    x: float
    y: float
    z: float
    w: float


@dataclass
class PhysicsProperties:
    """Physics properties for the simulation environment."""
    gravity: Vector3
    friction: float
    # Additional properties can be added as needed
    restitution: float = 0.0  # bounciness
    linear_damping: float = 0.05
    angular_damping: float = 0.05


@dataclass
class LightingConfiguration:
    """Lighting setup for the environment."""
    ambient_light: Vector3  # RGB values (0-1)
    directional_light_direction: Vector3  # Direction vector
    directional_light_color: Vector3  # RGB values (0-1)
    intensity: float = 1.0


@dataclass
class SimulationEnvironment:
    """Represents a simulation environment in Isaac Sim."""
    environment_id: str
    name: str
    description: str
    physics_properties: PhysicsProperties
    lighting: LightingConfiguration
    
    # These will be populated later as other models are implemented
    robot_models: List[str] = None  # List of robot model IDs
    sensor_configurations: List[str] = None  # List of sensor configuration IDs
    assets: List[str] = None  # File paths to assets
    
    def __post_init__(self):
        if self.robot_models is None:
            self.robot_models = []
        if self.sensor_configurations is None:
            self.sensor_configurations = []
        if self.assets is None:
            self.assets = []

    def add_robot_model(self, robot_model_id: str):
        """Add a robot model to the environment."""
        if robot_model_id not in self.robot_models:
            self.robot_models.append(robot_model_id)

    def add_sensor_configuration(self, sensor_config_id: str):
        """Add a sensor configuration to the environment."""
        if sensor_config_id not in self.sensor_configurations:
            self.sensor_configurations.append(sensor_config_id)

    def add_asset(self, asset_path: str):
        """Add an asset to the environment."""
        if asset_path not in self.assets:
            self.assets.append(asset_path)

    def validate(self) -> bool:
        """Validate the simulation environment."""
        # Check required fields
        if not self.environment_id or not isinstance(self.environment_id, str):
            raise ValueError("environment_id must be a non-empty string")
        
        if not self.name or not isinstance(self.name, str):
            raise ValueError("name must be a non-empty string")
        
        if not self.description or not isinstance(self.description, str):
            raise ValueError("description must be a non-empty string")
        
        # Check physics properties
        if not isinstance(self.physics_properties, PhysicsProperties):
            raise ValueError("physics_properties must be a PhysicsProperties object")
        
        # Check lighting configuration
        if not isinstance(self.lighting, LightingConfiguration):
            raise ValueError("lighting must be a LightingConfiguration object")
        
        return True