"""
Sensor Configuration Entity
Fields:
- sensorId: string (unique identifier for the sensor)
- sensorType: string (type of sensor - camera, lidar, imu, etc.)
- mountPoint: Vector3 (position of sensor on the robot)
- orientation: Quaternion (orientation of the sensor)
- parameters: SensorParameters object (specific parameters for the sensor)
- outputFormat: string (format of sensor output data)
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional
from .simulation_environment import Vector3, Quaternion


@dataclass
class SensorParameters:
    """Parameters specific to the sensor type."""
    # Common parameters
    update_rate: float = 30.0  # Hz
    
    # Camera-specific parameters
    resolution: Optional[tuple] = None  # (width, height)
    fov: Optional[float] = None  # Field of view in degrees
    
    # LiDAR-specific parameters
    range_min: Optional[float] = None
    range_max: Optional[float] = None
    rotation_frequency: Optional[float] = None
    channels: Optional[int] = None
    
    # IMU-specific parameters
    noise_density: Optional[float] = None
    random_walk: Optional[float] = None
    
    # Additional parameters can be added as needed
    extra_params: Optional[Dict[str, Any]] = None


@dataclass
class SensorConfiguration:
    """Represents a sensor configuration in Isaac Sim."""
    sensor_id: str
    sensor_type: str  # camera, lidar, imu, etc.
    mount_point: Vector3
    orientation: Quaternion
    output_format: str  # format of sensor output data
    
    # These are optional as they depend on sensor type
    parameters: Optional[SensorParameters] = None

    def validate(self) -> bool:
        """Validate the sensor configuration."""
        # Check required fields
        if not self.sensor_id or not isinstance(self.sensor_id, str):
            raise ValueError("sensor_id must be a non-empty string")
        
        if not self.sensor_type or not isinstance(self.sensor_type, str):
            raise ValueError("sensor_type must be a non-empty string")
        
        if not isinstance(self.mount_point, Vector3):
            raise ValueError("mount_point must be a Vector3 object")
        
        if not isinstance(self.orientation, Quaternion):
            raise ValueError("orientation must be a Quaternion object")
        
        if not self.output_format or not isinstance(self.output_format, str):
            raise ValueError("output_format must be a non-empty string")
        
        # Validate parameters if they exist
        if self.parameters and not isinstance(self.parameters, SensorParameters):
            raise ValueError("parameters must be a SensorParameters object")
        
        # Validate sensor type is supported
        supported_types = ['camera', 'lidar', 'imu', 'gps', 'force_torque', 'contact_sensor']
        if self.sensor_type.lower() not in [t.lower() for t in supported_types]:
            raise ValueError(f"sensor_type must be one of {supported_types}")
        
        return True

    def get_sensor_topic(self) -> str:
        """Generate the ROS topic name for this sensor."""
        return f"/sensor/{self.sensor_id.lower()}/{self.sensor_type.lower()}"