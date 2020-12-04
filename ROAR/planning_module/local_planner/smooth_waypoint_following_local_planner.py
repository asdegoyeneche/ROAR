from functools import reduce
from typing import Union

from ROAR.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.utilities_module.data_structures_models import Transform
from ROAR.utilities_module.errors import (
    AgentException,
)
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl


class SmoothWaypointFollowingLocalPlanner(SimpleWaypointFollowingLocalPlanner):

    def next_waypoint_smooth(self, smooth_factor=500) -> Transform:
        smooth_factor = min(smooth_factor, len(self.way_points_queue))
        location_sum = reduce(lambda x, y: x + y, (self.way_points_queue[i].location for i in range(smooth_factor)))
        rotation_sum = reduce(lambda x, y: x + y, (self.way_points_queue[i].rotation for i in range(smooth_factor)))

        return Transform(location=location_sum / smooth_factor, rotation=rotation_sum / smooth_factor)

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
            waypoint: Transform = self.next_waypoint_smooth()
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

        target_waypoint = self.next_waypoint_smooth()
        control: VehicleControl = self.controller.run_in_series(next_waypoint=target_waypoint)
        self.logger.debug(f"\n"
                          f"Curr Transform: {self.agent.vehicle.transform}\n"
                          f"Target Location: {target_waypoint.location}\n"
                          f"Control: {control} | Speed: {Vehicle.get_speed(self.agent.vehicle)}\n")
        return control
