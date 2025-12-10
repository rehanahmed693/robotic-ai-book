"""
Balance Constraint Entity
Fields:
- constraintId: string (unique identifier for the constraint)
- constraintType: string (type of balance constraint)
- parameters: object (parameters for the constraint)
- activationThreshold: float (threshold for activating the constraint)
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional
from ..models.simulation_environment import Vector3


@dataclass
class BalanceConstraint:
    """Represents a balance constraint for humanoid movement in Isaac Sim."""
    constraint_id: str
    constraint_type: str  # Type of balance constraint
    parameters: Dict[str, Any]  # Constraint-specific parameters
    activation_threshold: float  # Threshold for activating the constraint
    
    def validate(self) -> bool:
        """Validate the balance constraint configuration."""
        # Check required fields
        if not self.constraint_id or not isinstance(self.constraint_id, str):
            raise ValueError("constraint_id must be a non-empty string")
        
        if not self.constraint_type or not isinstance(self.constraint_type, str):
            raise ValueError("constraint_type must be a non-empty string")
        
        if not isinstance(self.parameters, dict):
            raise ValueError("parameters must be a dictionary")
        
        if not isinstance(self.activation_threshold, (int, float)):
            raise ValueError("activation_threshold must be a number")
        
        # Validate constraint type is supported
        supported_types = [
            "zero_moment_point", "center_of_mass", "foot_placement", 
            "torso_stability", "angular_momentum", "capture_point",
            "static_stability", "dynamic_stability"
        ]
        if self.constraint_type.lower() not in [t.lower() for t in supported_types]:
            raise ValueError(f"constraint_type must be one of {supported_types}")
        
        # Validate parameters based on constraint type
        self._validate_parameters()
        
        return True
    
    def _validate_parameters(self):
        """Validate parameters based on the constraint type."""
        constraint_type = self.constraint_type.lower()
        
        if constraint_type == "zero_moment_point":
            required_params = ["reference_point", "margin"]
            optional_params = ["control_gains"]
            
        elif constraint_type == "center_of_mass":
            required_params = ["reference_point", "bounds"]
            optional_params = ["control_gains", "weight"]
            
        elif constraint_type == "foot_placement":
            required_params = ["foot_positions", "support_polygon"]
            optional_params = ["swing_trajectory", "stance_time"]
            
        elif constraint_type == "torso_stability":
            required_params = ["reference_orientation", "max_angle"]
            optional_params = ["control_gains"]
            
        elif constraint_type == "angular_momentum":
            required_params = ["reference_value", "max_deviation"]
            optional_params = ["control_gains"]
            
        elif constraint_type == "capture_point":
            required_params = ["reference_point", "margin"]
            optional_params = ["control_gains"]
            
        elif constraint_type == "static_stability":
            required_params = ["support_polygon", "margin"]
            optional_params = ["weight_distribution"]
            
        elif constraint_type == "dynamic_stability":
            required_params = ["stability_margin", "frequency"]
            optional_params = ["control_gains"]
        
        else:
            # For unknown constraint types, just check that required params exist
            required_params = []
            optional_params = []
        
        # Check for required parameters
        for param in required_params:
            if param not in self.parameters:
                raise ValueError(f"Missing required parameter '{param}' for constraint type '{constraint_type}'")
        
        # Check for valid parameter types where applicable
        if "reference_point" in self.parameters:
            ref_point = self.parameters["reference_point"]
            if not isinstance(ref_point, (list, tuple)) or len(ref_point) != 3:
                raise ValueError("reference_point must be a list/tuple with 3 elements [x, y, z]")
            if not all(isinstance(coord, (int, float)) for coord in ref_point):
                raise ValueError("reference_point coordinates must be numbers")
        
        if "bounds" in self.parameters:
            bounds = self.parameters["bounds"]
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 6:
                raise ValueError("bounds must be a list/tuple with 6 elements [min_x, max_x, min_y, max_y, min_z, max_z]")
            if not all(isinstance(bound, (int, float)) for bound in bounds):
                raise ValueError("bounds values must be numbers")
        
        if "max_angle" in self.parameters:
            max_angle = self.parameters["max_angle"]
            if not isinstance(max_angle, (int, float)) or max_angle < 0:
                raise ValueError("max_angle must be a non-negative number")
    
    def get_parameter(self, param_name: str, default: Any = None) -> Any:
        """Get a parameter value with an optional default."""
        return self.parameters.get(param_name, default)
    
    def set_parameter(self, param_name: str, value: Any):
        """Set a parameter value."""
        self.parameters[param_name] = value
    
    def is_active(self, current_value: float) -> bool:
        """Check if the constraint is active based on the current value."""
        return abs(current_value) > abs(self.activation_threshold)
    
    def get_constraint_type(self) -> str:
        """Get the constraint type."""
        return self.constraint_type
    
    def get_urgency_level(self) -> str:
        """Get an urgency level based on how close the current value is to the threshold."""
        # This would be called with a current value in a real implementation
        # For now, we'll return a placeholder based on the threshold value
        if abs(self.activation_threshold) < 0.1:
            return "high"
        elif abs(self.activation_threshold) < 0.5:
            return "medium"
        else:
            return "low"