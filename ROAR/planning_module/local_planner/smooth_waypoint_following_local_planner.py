from functools import reduce
from typing import Union

from ROAR.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.utilities_module.data_structures_models import Transform
from ROAR.utilities_module.errors import (
    AgentException,
)
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
import numpy as np


class SmoothWaypointFollowingLocalPlanner(SimpleWaypointFollowingLocalPlanner):

    def next_waypoint_smooth_and_speed(self, smooth_factor=300, speed_lookahead=300) -> (Transform, float):
        smooth_factor = min(smooth_factor, len(self.way_points_queue))
        speed_lookahead = min(speed_lookahead, len(self.way_points_queue))
        sample_points = range(0, smooth_factor, smooth_factor//10)

        location_sum = reduce(lambda x, y: x + y,
                              (self.way_points_queue[i].location for i in sample_points))
        rotation_sum = reduce(lambda x, y: x + y,
                              (self.way_points_queue[i].rotation for i in sample_points))

        mean_yaw_0 = (self.way_points_queue[0].rotation.yaw + self.way_points_queue[1].rotation.yaw) / 2
        mean_yaw_ahead = (self.way_points_queue[speed_lookahead - 1].rotation.yaw +
                          self.way_points_queue[speed_lookahead].rotation.yaw) / 2
        angle_difference0 = abs((mean_yaw_0 - mean_yaw_ahead) % (2 * np.pi))
        angle_difference = min((2 * np.pi) - angle_difference0, angle_difference0)
        # print(mean_yaw_0, mean_yaw_ahead, angle_difference0, angle_difference)
        speed_multiplier = (1 - angle_difference / (2 * np.pi))  # ** 2.0

        # speed_multiplier = 1.0
        # angles = []
        # for index in (speed_lookahead, speed_lookahead // 5):
        #     delta_x = self.way_points_queue[index].location.x - self.way_points_queue[0].location.x
        #     delta_z = self.way_points_queue[index].location.z - self.way_points_queue[0].location.z
        #     angles.append(np.arctan2(delta_z, delta_x))

        # delta_angle = angles[0] - angles[1]

        print(speed_multiplier)
        num_points = len(sample_points)
        return Transform(location=location_sum / num_points, rotation=rotation_sum / num_points), speed_multiplier

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

        # get current waypoint
        curr_closest_dist = float("inf")
        while True:
            if len(self.way_points_queue) == 0:
                self.logger.info("Destination reached")
                return VehicleControl()
            # waypoint: Transform = self.way_points_queue[0]
            waypoint, speed_factor = self.next_waypoint_smooth_and_speed()
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

        target_waypoint, speed_factor = self.next_waypoint_smooth_and_speed()
        control: VehicleControl = self.controller.run_in_series(next_waypoint=target_waypoint,
                                                                speed_multiplier=speed_factor)
        self.logger.debug(f"\n"
                          f"Curr Transform: {self.agent.vehicle.transform}\n"
                          f"Target Location: {target_waypoint.location}\n"
                          f"Control: {control} | Speed: {Vehicle.get_speed(self.agent.vehicle)}\n")
        return control
