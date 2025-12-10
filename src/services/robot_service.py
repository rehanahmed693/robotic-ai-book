"""
Robot Configuration Service
Implementation based on the RobotModel entity
"""
from typing import Dict, List, Optional
from ..models.robot_model import RobotModel, JointConfiguration, KinematicProperties, DynamicProperties
from ..models.simulation_environment import Vector3


class RobotConfigurationService:
    """
    Service class for creating and managing robot models in Isaac Sim.
    """
    
    def __init__(self):
        self.robots: Dict[str, RobotModel] = {}

    def create_robot_model(
        self,
        model_id: str,
        name: str,
        urdf_path: Optional[str] = None,
        sdf_path: Optional[str] = None,
        base_frame: str = "base_link",
        tip_frame: str = "tool0",
        mass: float = 1.0,
        center_of_mass: Optional[Vector3] = None
    ) -> RobotModel:
        """
        Create a new robot model with the specified parameters.
        """
        if center_of_mass is None:
            center_of_mass = Vector3(0.0, 0.0, 0.0)
        
        # Create kinematic properties
        kinematic_props = KinematicProperties(
            base_frame=base_frame,
            tip_frame=tip_frame,
            joint_names=[]
        )
        
        # Create dynamic properties
        dynamic_props = DynamicProperties(
            mass=mass,
            center_of_mass=center_of_mass,
            inertia=[1.0, 0.0, 0.0, 1.0, 0.0, 1.0]  # Default inertia matrix
        )
        
        # Create the robot model
        robot = RobotModel(
            model_id=model_id,
            name=name,
            urdf_path=urdf_path,
            sdf_path=sdf_path,
            kinematic_properties=kinematic_props,
            dynamic_properties=dynamic_props
        )
        
        # Validate the robot model
        robot.validate()
        
        # Store the robot
        self.robots[model_id] = robot
        
        return robot

    def get_robot_model(self, model_id: str) -> Optional[RobotModel]:
        """
        Retrieve a robot model by its ID.
        """
        return self.robots.get(model_id)

    def delete_robot_model(self, model_id: str) -> bool:
        """
        Delete a robot model by its ID.
        """
        if model_id in self.robots:
            del self.robots[model_id]
            return True
        return False

    def add_joint_configuration(self, model_id: str, joint_config: JointConfiguration) -> bool:
        """
        Add a joint configuration to an existing robot model.
        """
        robot = self.get_robot_model(model_id)
        if robot:
            robot.add_joint_configuration(joint_config)
            # Update the kinematic properties to include the new joint
            if joint_config.joint_name not in robot.kinematic_properties.joint_names:
                robot.kinematic_properties.joint_names.append(joint_config.joint_name)
            return True
        return False

    def update_robot_urdf_path(self, model_id: str, urdf_path: str) -> bool:
        """
        Update the URDF path for an existing robot model.
        """
        robot = self.get_robot_model(model_id)
        if robot:
            robot.urdf_path = urdf_path
            return True
        return False

    def update_robot_sdf_path(self, model_id: str, sdf_path: str) -> bool:
        """
        Update the SDF path for an existing robot model.
        """
        robot = self.get_robot_model(model_id)
        if robot:
            robot.sdf_path = sdf_path
            return True
        return False

    def update_kinematic_properties(self, model_id: str, base_frame: str, tip_frame: str, joint_names: List[str]) -> bool:
        """
        Update kinematic properties of an existing robot model.
        """
        robot = self.get_robot_model(model_id)
        if robot and robot.kinematic_properties:
            robot.kinematic_properties.base_frame = base_frame
            robot.kinematic_properties.tip_frame = tip_frame
            robot.kinematic_properties.joint_names = joint_names
            return True
        return False

    def update_dynamic_properties(self, model_id: str, mass: float, center_of_mass: Vector3) -> bool:
        """
        Update dynamic properties of an existing robot model.
        """
        robot = self.get_robot_model(model_id)
        if robot and robot.dynamic_properties:
            robot.dynamic_properties.mass = mass
            robot.dynamic_properties.center_of_mass = center_of_mass
            return True
        return False

    def list_robots(self) -> List[str]:
        """
        List all available robot model IDs.
        """
        return list(self.robots.keys())