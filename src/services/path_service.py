"""
Path Planning Service
Implementation based on the NavigationPlan and Path entities
"""
from typing import Dict, List, Optional
from ..models.navigation_plan import NavigationPlan, Path, Costmap
from ..models.navigation_goal import NavigationGoal, Pose
from ..models.simulation_environment import Vector3


class PathPlanningService:
    """
    Service class for managing path planning in Isaac ROS Nav2.
    """
    
    def __init__(self):
        self.plans: Dict[str, NavigationPlan] = {}
        self.active_plan_id: Optional[str] = None
        self.current_costmap: Optional[Costmap] = None

    def create_plan(
        self,
        plan_id: str,
        frame_id: str,
        global_plan: Path
    ) -> NavigationPlan:
        """
        Create a new navigation plan with the specified parameters.
        """
        # Create the navigation plan
        plan = NavigationPlan(
            plan_id=plan_id,
            frame_id=frame_id,
            global_plan=global_plan,
            planner_status="initialized"
        )
        
        # Validate the plan
        plan.validate()
        
        # Store the plan
        self.plans[plan_id] = plan
        
        return plan

    def get_plan(self, plan_id: str) -> Optional[NavigationPlan]:
        """
        Retrieve a navigation plan by its ID.
        """
        return self.plans.get(plan_id)

    def delete_plan(self, plan_id: str) -> bool:
        """
        Delete a navigation plan by its ID.
        """
        if plan_id in self.plans:
            # If we're deleting the active plan, clear the active plan ID
            if self.active_plan_id == plan_id:
                self.active_plan_id = None
            del self.plans[plan_id]
            return True
        return False

    def set_active_plan(self, plan_id: str) -> bool:
        """
        Set a specific plan as the active one.
        """
        if plan_id in self.plans:
            self.active_plan_id = plan_id
            return True
        return False

    def get_active_plan(self) -> Optional[NavigationPlan]:
        """
        Get the currently active navigation plan.
        """
        if self.active_plan_id:
            return self.plans.get(self.active_plan_id)
        return None

    def add_goal_to_plan(self, plan_id: str, goal: NavigationGoal) -> bool:
        """
        Add a navigation goal to an existing plan.
        """
        plan = self.get_plan(plan_id)
        if plan:
            plan.add_goal(goal)
            return True
        return False

    def set_local_plan(self, plan_id: str, local_plan: Path) -> bool:
        """
        Set or update the local plan for a navigation plan.
        """
        plan = self.get_plan(plan_id)
        if plan:
            plan.local_plan = local_plan
            return True
        return False

    def set_costmap(self, plan_id: str, costmap: Costmap) -> bool:
        """
        Set or update the costmap for a navigation plan.
        """
        plan = self.get_plan(plan_id)
        if plan:
            plan.costmap = costmap
            # Also update the current costmap for the service
            self.current_costmap = costmap
            return True
        return False

    def update_plan_status(self, plan_id: str, status: str) -> bool:
        """
        Update the status of a navigation plan.
        """
        plan = self.get_plan(plan_id)
        if plan:
            valid_statuses = ["initialized", "computing", "ready", "executing", "completed", "failed", "aborted"]
            if status not in valid_statuses:
                raise ValueError(f"status must be one of {valid_statuses}")
            plan.planner_status = status
            return True
        return False

    def start_plan_execution(self, plan_id: str) -> bool:
        """
        Start executing a navigation plan.
        """
        plan = self.get_plan(plan_id)
        if plan:
            plan.planner_status = "executing"
            self.active_plan_id = plan_id
            print(f"Starting execution of plan: {plan.plan_id}")
            return True
        return False

    def stop_plan_execution(self) -> bool:
        """
        Stop the currently active plan execution.
        """
        if self.active_plan_id:
            plan = self.get_plan(self.active_plan_id)
            if plan:
                plan.planner_status = "ready"  # Or "paused" depending on implementation
                print(f"Stopped execution of plan: {plan.plan_id}")
            self.active_plan_id = None
            return True
        return False

    def compute_global_path(self, plan_id: str, start_pose: Pose, goal_pose: Pose) -> Optional[Path]:
        """
        Compute a global path for the plan using a simple algorithm (in practice, this would use A*, Dijkstra, etc.).
        This is a simplified implementation for demonstration purposes.
        """
        plan = self.get_plan(plan_id)
        if not plan:
            return None

        # Simple straight-line path computation for demonstration
        # In a real implementation, this would use a proper path planning algorithm like A*, Dijkstra, etc.
        path_poses = [start_pose]
        
        # Calculate the straight line between start and goal
        start_pos = start_pose.position
        goal_pos = goal_pose.position
        
        # Define the number of intermediate points
        num_steps = 10
        for i in range(1, num_steps):
            t = i / num_steps
            x = start_pos.x + t * (goal_pos.x - start_pos.x)
            y = start_pos.y + t * (goal_pos.y - start_pos.y)
            z = start_pos.z + t * (goal_pos.z - start_pos.z)
            
            # For simplicity, we'll keep the orientation the same as start
            intermediate_pose = Pose(
                position=Vector3(x, y, z),
                orientation=start_pose.orientation
            )
            path_poses.append(intermediate_pose)
        
        # Add the goal pose
        path_poses.append(goal_pose)
        
        # Create the path
        path = Path(
            path_id=f"{plan_id}_global_path",
            poses=path_poses,
            frame_id=plan.frame_id,
            source_map="default_map"  # This would be the actual map
        )
        
        # Update the plan with the new global path
        plan.global_plan = path
        
        return path

    def compute_local_path(self, plan_id: str, current_pose: Pose, global_path: Path, look_ahead: int = 5) -> Optional[Path]:
        """
        Compute a local path based on the global path and current position.
        This is a simplified implementation for demonstration purposes.
        """
        plan = self.get_plan(plan_id)
        if not plan:
            return None

        # In a real implementation, this would use a local planner like TEB, DWA, etc.
        # For this example, we'll just return a path that's a subset of the global path
        # from the current position onwards
        
        # Find the closest pose to the current position in the global path
        closest_idx = 0
        min_dist = float('inf')
        for i, pose in enumerate(global_path.poses):
            dx = current_pose.position.x - pose.position.x
            dy = current_pose.position.y - pose.position.y
            dist = (dx**2 + dy**2)**0.5
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Create a new path from the closest point onwards (up to look_ahead points)
        local_poses = global_path.poses[closest_idx:closest_idx + look_ahead]
        
        if not local_poses:
            return None
        
        # Create the local path
        local_path = Path(
            path_id=f"{plan_id}_local_path",
            poses=local_poses,
            frame_id=plan.frame_id,
            source_map="default_map"  # This would be the actual map
        )
        
        # Update the plan with the new local path
        plan.local_plan = local_path
        
        return local_path

    def list_plans(self) -> List[str]:
        """
        List all available plan IDs.
        """
        return list(self.plans.keys())

    def get_plan_status(self, plan_id: str) -> str:
        """
        Get the status of a specific plan.
        """
        plan = self.get_plan(plan_id)
        if plan:
            return plan.planner_status
        return "not_found"

    def get_current_costmap(self) -> Optional[Costmap]:
        """
        Get the current costmap being used for planning.
        """
        return self.current_costmap

    def reset_service(self):
        """
        Reset the service, clearing all plans.
        """
        self.plans = {}
        self.active_plan_id = None
        self.current_costmap = None