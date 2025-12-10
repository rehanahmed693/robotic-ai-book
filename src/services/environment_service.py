"""
Environment Creation Service
Implementation based on the SimulationEnvironment entity
"""
from typing import Dict, List, Optional
from ..models.simulation_environment import SimulationEnvironment, PhysicsProperties, LightingConfiguration, Vector3
from ..models.robot_model import RobotModel


class EnvironmentCreationService:
    """
    Service class for creating and managing simulation environments in Isaac Sim.
    """
    
    def __init__(self):
        self.environments: Dict[str, SimulationEnvironment] = {}
        self.active_environment: Optional[SimulationEnvironment] = None

    def create_environment(
        self,
        environment_id: str,
        name: str,
        description: str,
        gravity: Vector3,
        friction: float,
        ambient_light: Vector3,
        directional_light_direction: Vector3,
        directional_light_color: Vector3,
        intensity: float = 1.0
    ) -> SimulationEnvironment:
        """
        Create a new simulation environment with the specified parameters.
        """
        # Create physics properties
        physics_props = PhysicsProperties(
            gravity=gravity,
            friction=friction
        )
        
        # Create lighting configuration
        lighting_config = LightingConfiguration(
            ambient_light=ambient_light,
            directional_light_direction=directional_light_direction,
            directional_light_color=directional_light_color,
            intensity=intensity
        )
        
        # Create the simulation environment
        environment = SimulationEnvironment(
            environment_id=environment_id,
            name=name,
            description=description,
            physics_properties=physics_props,
            lighting=lighting_config
        )
        
        # Validate the environment
        environment.validate()
        
        # Store the environment
        self.environments[environment_id] = environment
        
        return environment
    
    def get_environment(self, environment_id: str) -> Optional[SimulationEnvironment]:
        """
        Retrieve an environment by its ID.
        """
        return self.environments.get(environment_id)
    
    def delete_environment(self, environment_id: str) -> bool:
        """
        Delete an environment by its ID.
        """
        if environment_id in self.environments:
            del self.environments[environment_id]
            if self.active_environment and self.active_environment.environment_id == environment_id:
                self.active_environment = None
            return True
        return False
    
    def add_robot_to_environment(self, environment_id: str, robot_model: RobotModel) -> bool:
        """
        Add a robot model to an existing environment.
        """
        environment = self.get_environment(environment_id)
        if environment:
            environment.add_robot_model(robot_model.model_id)
            return True
        return False
    
    def update_physics_properties(self, environment_id: str, gravity: Vector3, friction: float) -> bool:
        """
        Update physics properties of an existing environment.
        """
        environment = self.get_environment(environment_id)
        if environment:
            environment.physics_properties.gravity = gravity
            environment.physics_properties.friction = friction
            return True
        return False
    
    def list_environments(self) -> List[str]:
        """
        List all available environment IDs.
        """
        return list(self.environments.keys())
    
    def set_active_environment(self, environment_id: str) -> bool:
        """
        Set the specified environment as the active one.
        """
        environment = self.get_environment(environment_id)
        if environment:
            self.active_environment = environment
            return True
        return False
    
    def get_active_environment(self) -> Optional[SimulationEnvironment]:
        """
        Get the currently active environment.
        """
        return self.active_environment