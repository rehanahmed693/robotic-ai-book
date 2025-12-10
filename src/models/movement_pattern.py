"""
Movement Pattern Entity
Fields:
- patternId: string (unique identifier for the movement pattern)
- name: string (name of the movement pattern)
- description: string (description of the movement)
- jointTrajectories: JointTrajectory[] (joint trajectories for the movement)
- balanceConstraints: BalanceConstraints object (balance constraints for the movement)
- gaitType: string (type of gait if applicable)
"""
from dataclasses import dataclass
from typing import List, Optional
from ..models.simulation_environment import Vector3, Quaternion


@dataclass
class JointPoint:
    """A point in a joint trajectory with position, velocity, and acceleration."""
    time_from_start: float  # Time from the beginning of the trajectory (seconds)
    positions: List[float]  # Joint positions (radians for revolute joints, meters for prismatic)
    velocities: List[float]  # Joint velocities (rad/s or m/s)
    accelerations: List[float]  # Joint accelerations (rad/s^2 or m/s^2)
    effort: Optional[List[float]] = None  # Joint efforts (torques in Nm or forces in N)


@dataclass
class JointTrajectory:
    """A trajectory for one or more joints."""
    joint_names: List[str]  # Names of the joints in the trajectory
    trajectory_points: List[JointPoint]  # Points defining the trajectory over time
    start_time: float = 0.0  # Time to start execution (seconds)

    def __post_init__(self):
        if self.trajectory_points is None:
            self.trajectory_points = []


@dataclass
class BalanceConstraint:
    """A constraint for maintaining balance during movement."""
    constraint_id: str
    constraint_type: str  # e.g., "zero_moment_point", "center_of_mass", "foot_placement"
    parameters: dict  # Constraint-specific parameters
    activation_threshold: float  # Threshold for activating the constraint (e.g., angle, force)
    
    def validate(self) -> bool:
        """Validate the balance constraint."""
        if not self.constraint_id or not isinstance(self.constraint_id, str):
            raise ValueError("constraint_id must be a non-empty string")
        
        if not self.constraint_type or not isinstance(self.constraint_type, str):
            raise ValueError("constraint_type must be a non-empty string")
        
        if not isinstance(self.parameters, dict):
            raise ValueError("parameters must be a dictionary")
        
        if not isinstance(self.activation_threshold, (int, float)):
            raise ValueError("activation_threshold must be a number")
        
        # Validate constraint type is supported
        supported_types = ["zero_moment_point", "center_of_mass", "foot_placement", "torso_stability", "angular_momentum"]
        if self.constraint_type.lower() not in [t.lower() for t in supported_types]:
            raise ValueError(f"constraint_type must be one of {supported_types}")
        
        return True


@dataclass
class BalanceConstraints:
    """Collection of balance constraints for a movement pattern."""
    constraints: List[BalanceConstraint]
    
    def __post_init__(self):
        if self.constraints is None:
            self.constraints = []
    
    def add_constraint(self, constraint: BalanceConstraint):
        """Add a balance constraint to the collection."""
        if not isinstance(constraint, BalanceConstraint):
            raise ValueError("constraint must be a BalanceConstraint object")
        self.constraints.append(constraint)
        
    def validate(self) -> bool:
        """Validate all balance constraints."""
        for constraint in self.constraints:
            if not isinstance(constraint, BalanceConstraint):
                raise ValueError("All constraints must be BalanceConstraint objects")
            constraint.validate()
        return True


@dataclass
class MovementPattern:
    """Represents a movement pattern for humanoid robots in Isaac Sim."""
    pattern_id: str
    name: str
    description: str
    
    # Optional fields
    joint_trajectories: List[JointTrajectory] = None
    balance_constraints: Optional[BalanceConstraints] = None
    gait_type: Optional[str] = None  # e.g., "walk", "trot", "pace", "amble", "bound", "gallop", "jump", "turn"

    def __post_init__(self):
        if self.joint_trajectories is None:
            self.joint_trajectories = []
        if self.balance_constraints is None:
            self.balance_constraints = BalanceConstraints([])

    def add_joint_trajectory(self, trajectory: JointTrajectory):
        """Add a joint trajectory to the movement pattern."""
        if not isinstance(trajectory, JointTrajectory):
            raise ValueError("trajectory must be a JointTrajectory object")
        self.joint_trajectories.append(trajectory)

    def validate(self) -> bool:
        """Validate the movement pattern configuration."""
        # Check required fields
        if not self.pattern_id or not isinstance(self.pattern_id, str):
            raise ValueError("pattern_id must be a non-empty string")
        
        if not self.name or not isinstance(self.name, str):
            raise ValueError("name must be a non-empty string")
        
        if not self.description or not isinstance(self.description, str):
            raise ValueError("description must be a non-empty string")
        
        # Validate optional fields if they exist
        if self.joint_trajectories is not None:
            if not isinstance(self.joint_trajectories, list):
                raise ValueError("joint_trajectories must be a list of JointTrajectory objects")
            for trajectory in self.joint_trajectories:
                if not isinstance(trajectory, JointTrajectory):
                    raise ValueError("All trajectories must be JointTrajectory objects")
        
        if self.balance_constraints is not None and not isinstance(self.balance_constraints, BalanceConstraints):
            raise ValueError("balance_constraints must be a BalanceConstraints object")
        
        # Validate gait type if specified
        if self.gait_type is not None:
            supported_gaits = ["walk", "trot", "pace", "amble", "bound", "gallop", "jump", "turn", "stand", "sit", "crouch"]
            if self.gait_type.lower() not in [g.lower() for g in supported_gaits]:
                raise ValueError(f"gait_type must be one of {supported_gaits}")
        
        return True

    def get_duration(self) -> float:
        """Calculate the total duration of the movement pattern."""
        if not self.joint_trajectories:
            return 0.0
        
        max_duration = 0.0
        for trajectory in self.joint_trajectories:
            if trajectory.trajectory_points:
                last_point = trajectory.trajectory_points[-1]
                duration = last_point.time_from_start
                max_duration = max(max_duration, duration)
        
        return max_duration

    def get_num_joints(self) -> int:
        """Get the number of joints involved in the movement pattern."""
        if not self.joint_trajectories:
            return 0
        
        all_joint_names = set()
        for trajectory in self.joint_trajectories:
            all_joint_names.update(trajectory.joint_names)
        
        return len(all_joint_names)