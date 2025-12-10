"""
Navigation Goal Service
Implementation based on the NavigationGoal entity
"""
from typing import Dict, List, Optional
from ..models.navigation_goal import NavigationGoal, Pose
from ..models.simulation_environment import Vector3, Quaternion


class NavigationGoalService:
    """
    Service class for managing navigation goals in Isaac ROS Nav2.
    """
    
    def __init__(self):
        self.goals: Dict[str, NavigationGoal] = {}
        self.active_goal_id: Optional[str] = None

    def create_goal(
        self,
        goal_id: str,
        target_pose: Pose,
        frame_id: str,
        tolerance: float = 0.5,
        priority: int = 0
    ) -> NavigationGoal:
        """
        Create a new navigation goal with the specified parameters.
        """
        # Create the navigation goal
        goal = NavigationGoal(
            goal_id=goal_id,
            target_pose=target_pose,
            frame_id=frame_id,
            tolerance=tolerance,
            priority=priority,
            status="pending"
        )
        
        # Validate the goal
        goal.validate()
        
        # Store the goal
        self.goals[goal_id] = goal
        
        return goal

    def get_goal(self, goal_id: str) -> Optional[NavigationGoal]:
        """
        Retrieve a navigation goal by its ID.
        """
        return self.goals.get(goal_id)

    def delete_goal(self, goal_id: str) -> bool:
        """
        Delete a navigation goal by its ID.
        """
        if goal_id in self.goals:
            # If we're deleting the active goal, clear the active goal ID
            if self.active_goal_id == goal_id:
                self.active_goal_id = None
            del self.goals[goal_id]
            return True
        return False

    def start_goal(self, goal_id: str) -> bool:
        """
        Start a navigation goal by its ID.
        """
        goal = self.get_goal(goal_id)
        if goal:
            goal.set_status("active")
            self.active_goal_id = goal_id
            # In a real implementation, this would trigger the actual navigation
            print(f"Starting navigation goal: {goal.goal_id}")
            return True
        return False

    def cancel_goal(self, goal_id: str) -> bool:
        """
        Cancel a navigation goal by its ID.
        """
        goal = self.get_goal(goal_id)
        if goal:
            goal.set_status("canceled")
            if self.active_goal_id == goal_id:
                self.active_goal_id = None
            print(f"Canceled navigation goal: {goal.goal_id}")
            return True
        return False

    def set_active_goal(self, goal_id: str) -> bool:
        """
        Set a specific goal as the active one.
        """
        if goal_id in self.goals:
            self.active_goal_id = goal_id
            return True
        return False

    def get_active_goal(self) -> Optional[NavigationGoal]:
        """
        Get the currently active navigation goal.
        """
        if self.active_goal_id:
            return self.goals.get(self.active_goal_id)
        return None

    def update_goal_pose(self, goal_id: str, new_pose: Pose) -> bool:
        """
        Update the target pose of a navigation goal.
        """
        goal = self.get_goal(goal_id)
        if goal:
            goal.target_pose = new_pose
            return True
        return False

    def update_goal_tolerance(self, goal_id: str, new_tolerance: float) -> bool:
        """
        Update the tolerance of a navigation goal.
        """
        goal = self.get_goal(goal_id)
        if goal and new_tolerance >= 0:
            goal.tolerance = new_tolerance
            return True
        return False

    def list_goals(self) -> List[str]:
        """
        List all available goal IDs.
        """
        return list(self.goals.keys())

    def get_goals_by_status(self, status: str) -> List[NavigationGoal]:
        """
        Get all goals with a specific status.
        """
        return [goal for goal in self.goals.values() if goal.status == status]

    def get_goals_by_priority_range(self, min_priority: int, max_priority: int) -> List[NavigationGoal]:
        """
        Get all goals within a specific priority range.
        """
        return [
            goal for goal in self.goals.values()
            if min_priority <= goal.priority <= max_priority
        ]

    def prioritize_goals(self) -> List[NavigationGoal]:
        """
        Return all goals sorted by priority (lowest number = highest priority).
        """
        return sorted(self.goals.values(), key=lambda g: g.priority)

    def get_goal_status(self, goal_id: str) -> str:
        """
        Get the status of a specific goal.
        """
        goal = self.get_goal(goal_id)
        if goal:
            return goal.status
        return "not_found"

    def complete_goal(self, goal_id: str, success: bool = True) -> bool:
        """
        Mark a goal as completed (either succeeded or failed).
        """
        goal = self.get_goal(goal_id)
        if goal:
            if success:
                goal.set_status("succeeded")
            else:
                goal.set_status("failed")
            
            if self.active_goal_id == goal_id:
                self.active_goal_id = None
            
            print(f"Goal {goal_id} marked as {'succeeded' if success else 'failed'}")
            return True
        return False

    def reset_service(self):
        """
        Reset the service, clearing all goals.
        """
        self.goals = {}
        self.active_goal_id = None