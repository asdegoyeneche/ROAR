from ROAR.planning_module.local_planner.lane_following_local_planner import LaneFollowingLocalPlanner
from ROAR.perception_module.lane_detector import LaneDetector
from ROAR.perception_module.object_detector import ObjectDetector
from ROAR.agent_module.agent import Agent
from pathlib import Path
#from ROAR.control_module.pid_controller import PIDController
from ROAR.control_module.lqr_controller import LQRController
#from ROAR.planning_module.local_planner.simple_waypoint_following_local_planner import \
#    SimpleWaypointFollowingLocalPlanner
from ROAR.planning_module.local_planner.smooth_waypoint_following_local_planner import \
    SmoothWaypointFollowingLocalPlanner
from ROAR.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
import logging


class LQRAgent(Agent):
    def __init__(self, target_speed=40, **kwargs):
        super().__init__(**kwargs)
        self.target_speed = target_speed
        self.logger = logging.getLogger("LQR Agent")
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.lqr_controller = LQRController(agent=self, steering_boundary=(-1, 1), throttle_boundary=(-1, 1))
        self.mission_planner = WaypointFollowingMissionPlanner(agent=self)
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(agent=self)
        self.local_planner = LaneFollowingLocalPlanner(
        #self.local_planner = SimpleWaypointFollowingLocalPlanner(
            agent=self,
            controller=self.lqr_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.lane_detector = LaneDetector(agent=self)
        self.front_rgb_camera = self.agent_settings.front_rgb_cam
        self.front_depth_camera = self.agent_settings.front_depth_cam
        self.obj_detector = ObjectDetector(agent=self, camera=self.front_rgb_camera, name="front")
        #self.left_depth_camera = self.agent_settings.left_depth_cam
        #self.right_depth_camera = self.agent_settings.right_depth_cam
        self.left_depth_camera = None
        self.right_depth_camera = None
        #self.left_obj_detector = ObjectDetector(agent=self, camera=self.left_depth_camera, name="left")
        #self.right_obj_detector = ObjectDetector(agent=self, camera=self.right_depth_camera, name="right")

        #self.init_depth_cams()

        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {self.route_file_path.as_posix()}")

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(LQRAgent, self).run_step(vehicle=vehicle,
                                       sensors_data=sensors_data)
        self.transform_history.append(self.vehicle.transform)
        self.lane_detector.run_in_series()
        self.obj_detector.run_in_series()
        #self.left_obj_detector.run_in_series()
        #self.right_obj_detector.run_in_series()
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_in_series()
        return control

    def init_depth_cams(self) -> None:
        """
        Initialize the cameras by calculating the camera intrinsics and
        ensuring that the output folder path exists

        Returns:
            None
        """
        if self.left_depth_camera is not None:
            self.left_depth_camera.intrinsics_matrix = (
                self.left_depth_camera.calculate_default_intrinsics_matrix()
            )
        if self.right_depth_camera is not None:
            self.right_depth_camera.intrinsics_matrix = (
                self.right_depth_camera.calculate_default_intrinsics_matrix()
            )

    def sync_data(self, sensors_data: SensorsData, vehicle: Vehicle) -> None:
        """
        Sync agent's state by updating Sensor Data and vehicle information

        Args:
            sensors_data: the new frame's sensor data
            vehicle: the new frame's vehicle state

        Returns:
            None
        """
        super().sync_data(sensors_data, vehicle)

        if self.left_depth_camera is not None:
            self.left_depth_camera.data = (
                sensors_data.left_depth.data
                if sensors_data.left_depth is not None
                else None
            )

        if self.right_depth_camera is not None:
            self.right_depth_camera.data = (
                sensors_data.right_depth.data
                if sensors_data.right_depth is not None
                else None
            )

