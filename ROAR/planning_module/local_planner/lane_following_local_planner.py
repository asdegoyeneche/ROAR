import math
from ROAR.planning_module.local_planner.smooth_waypoint_following_local_planner import SmoothWaypointFollowingLocalPlanner
from functools import reduce

from ROAR.utilities_module.utilities import two_points_to_pitch
from ROAR.perception_module.lane_detector import LaneDetector
from ROAR.planning_module.local_planner.local_planner import LocalPlanner
from ROAR.utilities_module.data_structures_models import Location, Rotation, Transform
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.control_module.controller import Controller
from ROAR.planning_module.mission_planner.mission_planner import MissionPlanner
from ROAR.planning_module.behavior_planner.behavior_planner import BehaviorPlanner

import logging
from typing import Union
from ROAR.utilities_module.errors import (
    AgentException,
)
from ROAR.agent_module.agent import Agent
import json
from pathlib import Path


class LaneFollowingLocalPlanner(SmoothWaypointFollowingLocalPlanner):

    def run_in_series(self) -> VehicleControl:
        """
        Run step for the local planner
        Procedure:
            1. Sync data
            2. get the correct look ahead for current speed
            3. get the correct next waypoint
            4. feed waypoint into controller
            5. return result from controller

        Returns:
            next control that the local think the agent should execute.
        """
        if (
                len(self.mission_planner.mission_plan) == 0
                and len(self.way_points_queue) == 0
        ):
            return VehicleControl()

        # get vehicle's location
        vehicle_transform: Union[Transform, None] = self.agent.vehicle.transform

        if vehicle_transform is None:
            raise AgentException("I do not know where I am, I cannot proceed forward")

        # redefine closeness level based on speed
        self.set_closeness_threhold(self.closeness_threshold_config)

        # get current lane center
        lane_detector: LaneDetector = self.agent.lane_detector
        lane_center = lane_detector.lane_center
        next_location = Location.from_array(lane_center[0]*0+lane_center[1]*1) + vehicle_transform.location*0
        # next_pitch = two_points_to_pitch(lane_center[0], lane_center[1])
        # next_rotation = Rotation(yaw=vehicle_transform.rotation.yaw, 
        #                          pitch=math.degrees(next_pitch), 
        #                          roll=vehicle_transform.rotation.roll)
        target_waypoint = Transform(location=next_location, rotation=vehicle_transform.rotation)
        
        curr_closest_dist = float("inf")
        while True:
            if len(self.way_points_queue) == 0:
                self.logger.info("Destination reached")
                return VehicleControl()
            waypoint: Transform = self.way_points_queue[0]
            curr_dist = vehicle_transform.location.distance(waypoint.location)
            if curr_dist < curr_closest_dist:
                # if i find a waypoint that is closer to me than before
                # note that i will always enter here to start the calculation for curr_closest_dist
                curr_closest_dist = curr_dist
            elif curr_dist < self.closeness_threshold:
                # i have moved onto a waypoint, remove that waypoint from the queue
                self.way_points_queue.popleft()
            else:
                break
        target_waypoint = target_waypoint * .5 + self.way_points_queue[0] * .5

        control: VehicleControl = self.controller.run_in_series(next_waypoint=target_waypoint)
        # self.logger.debug(f"\n"
        #                   f"Curr Transform: {self.agent.vehicle.transform}\n"
        #                   f"Target Transform: {target_waypoint}\n"
        #                   f"Control: {control} | Speed: {Vehicle.get_speed(self.agent.vehicle)}\n")
        return control
