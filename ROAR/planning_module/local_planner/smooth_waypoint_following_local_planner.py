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

    def next_waypoint_smooth_and_speed(self, smooth_factor=300, speed_lookahead=500) -> (Transform, float):
        # car_speed = np.linalg.norm(self.agent.vehicle.velocity.to_array()) * 3.6  # m/s to km/hr
        # lookahead_multiplier = car_speed / 170  # self.agent.agent_settings.target_speed
        #
        # waypoint_lookahead = max(waypoint_lookahead // 2, int(waypoint_lookahead * lookahead_multiplier))
        # speed_lookahead = max(speed_lookahead // 2, int(speed_lookahead * lookahead_multiplier))
        #
        # waypoint_lookahead = min(waypoint_lookahead, len(self.way_points_queue) - 1)
        # speed_lookahead = min(speed_lookahead, len(self.way_points_queue) - 1)
        #
        # print("WAYPOINT", waypoint_lookahead)
        #
        # if waypoint_lookahead > 1:  # can get rid of if statement here
        #     target_waypoint = self.way_points_queue[waypoint_lookahead]
        # else:
        #     target_waypoint = self.way_points_queue[-1]

        smooth_factor = min(smooth_factor, len(self.way_points_queue) - 1)
        speed_lookahead = min(speed_lookahead, len(self.way_points_queue) - 1)

        if smooth_factor > 10:
            sample_points = range(0, smooth_factor, smooth_factor//10)
            location_sum = reduce(lambda x, y: x + y,
                                  (self.way_points_queue[i].location for i in sample_points))
            rotation_sum = reduce(lambda x, y: x + y,
                                  (self.way_points_queue[i].rotation for i in sample_points))

            num_points = len(sample_points)
            target_waypoint = Transform(location=location_sum / num_points, rotation=rotation_sum / num_points)
        else:
            target_waypoint = self.way_points_queue[-1]

        # angle_std = np.std([self.way_points_queue[i].rotation.yaw for i in sample_points])
        # # speed_multiplier = 1 - angle_std
        # speed_multiplier = 1.0
        # print()
        # print([round(self.way_points_queue[i].rotation.yaw, 2) for i in sample_points])
        # print([round(self.way_points_queue[i].rotation.pitch, 2) for i in sample_points])

        # mean_pitch_0 = (self.way_points_queue[0].rotation.pitch + self.way_points_queue[1].rotation.pitch) / 2
        # mean_pitch_ahead = (self.way_points_queue[speed_lookahead - 1].rotation.pitch +
        #                     self.way_points_queue[speed_lookahead].rotation.pitch) / 2
        # angle_difference = abs(np.deg2rad(mean_pitch_0 - mean_pitch_ahead) % (2 * np.pi))
        # angle_difference = min((2 * np.pi) - angle_difference, angle_difference)
        #
        # speed_multiplier = (1.0 - angle_difference / np.pi)

        if speed_lookahead > 0:
            # print(speed_lookahead, len(self.way_points_queue))
            angle_difference = self._calculate_angle_error(self.way_points_queue[speed_lookahead])
            # Angle difference is between 0 and 180, but unlikely to be more than 90
            speed_multiplier = max(0.6, (1.0 - 1.2 * angle_difference / np.pi))
            # speed_multiplier = np.exp(- 2.0 * angle_difference) * 0.5 + 0.5
            # speed_multiplier = max(0.5, (1.0 - angle_difference / np.pi) ** 1.5)
            # speed_multiplier = max(0.5, (2.0 - (1.0 + angle_difference) ** 2))

            # print("Angle difference:", np.degrees(angle_difference))
            # print("Speed Multiplier", speed_multiplier)
        else:
            speed_multiplier = 0.0

        # angles = []
        # for index in (speed_lookahead, speed_lookahead // 5):
        #     delta_x = self.way_points_queue[index].location.x - self.way_points_queue[0].location.x
        #     delta_z = self.way_points_queue[index].location.z - self.way_points_queue[0].location.z
        #     angles.append(np.arctan2(delta_z, delta_x))
        # delta_angle = angles[0] - anglessum

        return target_waypoint, speed_multiplier

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

    def _calculate_angle_error(self, next_waypoint: Transform, epsilon=10):
        # calculate a vector that represent where you are going

        v_vec = np.array([np.cos(np.radians(self.agent.vehicle.transform.rotation.pitch)),
                          0,
                          np.sin(np.radians(self.agent.vehicle.transform.rotation.pitch))])
        v_vec_norm = np.linalg.norm(v_vec)  # Already norm 1

        w_vec = next_waypoint.location.to_array() - self.controller.agent.vehicle.transform.location.to_array()
        w_vec[1] = 0.
        w_vec_norm = np.linalg.norm(w_vec)

        # print("NORM", round(w_vec_norm, 3), round(v_vec_norm, 3))
        # if w_vec_norm > epsilon:
        return np.math.acos(np.dot(v_vec, w_vec) / (v_vec_norm * w_vec_norm))  # 0 to np.pi
        # return 0.0
