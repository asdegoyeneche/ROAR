from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_Jetson.jetson_vehicle import Vehicle as JetsonVehicle
from typing import Optional, Tuple

from ROAR_Jetson.arduino_cmd_sender import ArduinoCommandSender

from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import Transform
from Bridges.jetson_bridge import JetsonBridge
from ROAR_Jetson.camera import RS_D435i
import logging
import pygame
from ROAR_Jetson.jetson_keyboard_control import JetsonKeyboardControl
import numpy as np
from ROAR_Jetson.configurations.configuration import Configuration as JetsonConfig
from ROAR_Jetson.arduino_receiver import ArduinoReceiver
from ROAR_Jetson.vive.vive_tracker_client import ViveTrackerClient
from ROAR_Jetson.vive.models import ViveTrackerMessage
import serial
import sys
from pathlib import Path
import cv2


class JetsonRunner:
    """
    In charge of maintaining the state of the game.
    Drive command for agent to move next step
    Drive command for jetson to issue next command
    Update PyGame visualizations and controls parsing
    """

    def __init__(self, agent: Agent, jetson_config: JetsonConfig):
        self.jetson_vehicle: JetsonVehicle = JetsonVehicle()
        self.jetson_config = jetson_config
        self.agent = agent
        self.jetson_bridge = JetsonBridge()
        self.logger = logging.getLogger("Jetson Runner")
        self.display: Optional[pygame.display] = None
        self.serial: Optional[serial.Serial] = None
        self.transform = Transform()
        self.controller = JetsonKeyboardControl()
        self.rs_d435i: Optional[RS_D435i] = None
        self.vive_tracker: Optional[ViveTrackerClient] = None

        if jetson_config.initiate_pygame:
            self.setup_pygame()
        self.setup_serial()
        self.setup_jetson_vehicle()

        self.auto_pilot = True
        self.pygame_initiated = False
        self.logger.info("Jetson Vehicle Connected and Intialized. All Hardware is online and running")

    def setup_pygame(self):
        """
        Initiate pygame
        Returns:

        """
        pygame.init()
        pygame.font.init()
        import platform
        if platform.architecture()[1] == "ELF":
            self.display = pygame.display.set_mode((self.jetson_config.pygame_display_width,
                                                    self.jetson_config.pygame_display_height),
                                                   pygame.OPENGLBLIT)
        else:
            self.display = pygame.display.set_mode((self.jetson_config.pygame_display_width,
                                                    self.jetson_config.pygame_display_height),
                                                   pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.pygame_initiated = True
        self.logger.debug("PyGame initiated")

    def setup_serial(self):
        try:
            if 'win' in sys.platform:
                self.serial = serial.Serial(port=self.jetson_config.win_serial_port,
                                            baudrate=self.jetson_config.baud_rate,
                                            timeout=self.jetson_config.arduino_timeout,
                                            writeTimeout=self.jetson_config.write_timeout)
            else:
                self.serial = serial.Serial(port=self.jetson_config.unix_serial_port,
                                            baudrate=self.jetson_config.baud_rate,
                                            timeout=self.jetson_config.arduino_timeout,
                                            writeTimeout=self.jetson_config.write_timeout)
        except Exception as e:
            self.logger.error(f"Unable to establish serial connection: {e}")

    def setup_jetson_vehicle(self):
        """
        Add component to JetsonVehicle instance
        Returns:
            None
        """

        try:
            self.jetson_vehicle.add(ArduinoCommandSender(serial=self.serial,
                                                         servo_throttle_range=[self.jetson_config.motor_min,
                                                                               self.jetson_config.motor_max],
                                                         servo_steering_range=[self.jetson_config.theta_min,
                                                                               self.jetson_config.theta_max]),
                                    inputs=['throttle', 'steering'], threaded=True)
        except Exception as e:
            self.logger.error(f"Ignoring Error during ArduinoCommandSender set up: {e}")
        try:
            self.jetson_vehicle.add(ArduinoReceiver(serial=self.serial, client_ip=self.jetson_config.client_ip))
        except Exception as e:
            self.logger.error(f"Ignoring Error during ArduinoReceiver setup: {e}")
        try:
            self.rs_d435i = RS_D435i(image_w=self.agent.front_rgb_camera.image_size_x,
                                     image_h=self.agent.front_rgb_camera.image_size_y,
                                     image_output=True)
            self.jetson_vehicle.add(self.rs_d435i, threaded=True)
        except Exception as e:
            self.logger.error(f"Unable to connect to Intel Realsense: {e}")

        try:
            self.vive_tracker = ViveTrackerClient(host=self.jetson_config.vive_tracker_host,
                                                      port=self.jetson_config.vive_tracker_port,
                                                      tracker_name=self.jetson_config.vive_tracker_name)
            self.jetson_vehicle.add(self.vive_tracker, threaded=True)
        except Exception as e:
            self.logger.error(f"Failed to initalize Vive Tracker: {e}")

    def start_game_loop(self, use_manual_control=False):
        self.logger.info("Starting Game Loop")
        try:
            self.jetson_vehicle.start_part_threads()
            self.agent.start_module_threads()
            clock = pygame.time.Clock()
            should_continue = True
            while should_continue:
                clock.tick_busy_loop(60)

                # assign intrinsics matrix as soon as it arrives
                self._assign_camera_intrinsics()

                # pass throttle and steering into the bridge
                sensors_data, vehicle = self.convert_data()

                # run a step of agent
                vehicle_control = VehicleControl()
                if self.auto_pilot:
                    vehicle_control: VehicleControl = self.agent.run_step(sensors_data=sensors_data, vehicle=vehicle)

                # manual control always take precedence
                if use_manual_control:
                    should_continue, vehicle_control = self.update_pygame(clock=clock)
                else:
                    should_continue, _ = self.update_pygame(clock=clock)
                    # uncomment to not display on pygame on autodrive mode
                    # should_continue, _ = self.update_pygame_keyboard(clock=clock)
                # self.logger.debug(f"Vehicle Control = [{vehicle_control}]")
                # pass the output into sender to send it

                self.jetson_vehicle.update_parts(new_throttle=vehicle_control.throttle,
                                                 new_steering=vehicle_control.steering)
                # print(vehicle_control)
        except KeyboardInterrupt:
            self.logger.info("Keyboard Interrupt detected. Safely quitting")
            self.jetson_vehicle.stop()
        except Exception as e:
            self.logger.error(f"Something bad happened: [{e}]")
        finally:
            self.on_finish()

    def on_finish(self):
        self.jetson_vehicle.stop()
        self.agent.shutdown_module_threads()

    def convert_data(self) -> Tuple[SensorsData, Vehicle]:
        """
        Convert sensor and vehicle state data from source to agent
        Returns:
            SensorsData and Vehicle state.
        """
        sensors_data: SensorsData = self.jetson_bridge.convert_sensor_data_from_source_to_agent(
            source={
                "front_rgb": self.jetson_vehicle.front_rgb_img,
                "rear_rgb": None,
                "front_depth": self.jetson_vehicle.front_depth_img,
                "imu": None,
                "vive_tracking": self.vive_tracker.latest_tracker_message
                if self.vive_tracker is not None and self.vive_tracker.latest_tracker_message is not None else None
            }
        )
        if sensors_data.vive_tracker_data is not None:
            self.jetson_vehicle.location = sensors_data.vive_tracker_data.location.to_array()
            self.jetson_vehicle.rotation = sensors_data.vive_tracker_data.rotation.to_array()
            self.jetson_vehicle.velocity = sensors_data.vive_tracker_data.velocity.to_array()
        new_vehicle = self.jetson_bridge.convert_vehicle_from_source_to_agent(self.jetson_vehicle)
        # print(new_vehicle.transform)
        return sensors_data, new_vehicle

    def update_pygame(self, clock) -> Tuple[bool, VehicleControl]:
        """
        Update the pygame window, including parsing keypress

        Args:
            clock: pygame clock

        Returns:
            bool - whether to continue the game
            VehicleControl - the new VehicleControl cmd by the keyboard
        """
        if self.display is not None and self.agent.front_rgb_camera.data is not None:
            array: np.ndarray = self.agent.front_rgb_camera.data.copy()[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.display.blit(surface, (0, 0))
        pygame.display.flip()
        return self.update_pygame_keyboard(clock=clock)

    def update_pygame_keyboard(self, clock):
        return self.controller.parse_events(clock=clock)

    def _assign_camera_intrinsics(self):
        if self.rs_d435i.depth_camera_intrinsics is not None:
            self.agent.front_depth_camera.intrinsics_matrix = self.rs_d435i.depth_camera_intrinsics
            self.agent.front_depth_camera.distortion_coefficient = self.rs_d435i.depth_camera_distortion_coefficients
        if self.rs_d435i.rgb_camera_intrinsics is not None:
            self.agent.front_rgb_camera.intrinsics_matrix = self.rs_d435i.rgb_camera_intrinsics
            self.agent.front_rgb_camera.distortion_coefficient = self.rs_d435i.rgb_camera_distortion_coefficients
