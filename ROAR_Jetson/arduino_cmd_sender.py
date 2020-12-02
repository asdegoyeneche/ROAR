from serial import Serial
import logging
import time
import numpy as np
from typing import List, Tuple, Optional

MOTOR_MAX = 1750
MOTOR_MIN = 800
MOTOR_NEUTRAL = 1500
THETA_MAX = 3000
THETA_MIN = 0


class ArduinoCommandSender:
    """
    Responsible for translating Agent Throttle and Steering to Servo (motor on the race car) RPM and issue the command
    """

    def __init__(self,
                 serial: Serial,
                 min_command_time_gap: float = 0.1,
                 agent_throttle_range: Optional[List] = None,
                 agent_steering_range: Optional[List] = None,
                 servo_throttle_range: Optional[List] = None,
                 servo_steering_range: Optional[List] = None):
        """
        Initialize parameters.

        Args:
            min_command_time_gap: minimum command duration between two commands
        """
        if agent_steering_range is None:
            agent_steering_range = [-1, 1]
        if agent_throttle_range is None:
            agent_throttle_range = [-1, 1]
        if servo_throttle_range is None:
            servo_throttle_range = [MOTOR_MIN, MOTOR_MAX]
        if servo_steering_range is None:
            servo_steering_range = [THETA_MIN, THETA_MAX]

        self.serial = serial

        self.prev_throttle = 1500  # record previous throttle, set to neutral initially 
        self.prev_steering = 1500  # record previous steering, set to neutral initially
        self.last_cmd_time = None
        # time in seconds between two commands to avoid killing the arduino
        self.min_command_time_gap = min_command_time_gap
        self.agent_throttle_range = agent_throttle_range
        self.agent_steering_range = agent_steering_range
        self.servo_throttle_range = servo_throttle_range
        self.servo_steering_range = servo_steering_range
        self.logger = logging.getLogger("Jetson CMD Sender")
        self.logger.debug("Jetson CMD Sender Initialized")

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        """
        Run a step of command

        Args:
            throttle: new throttle, in the range of agent_throttle_range
            steering: new steering, in the range of agent_steering_range
            **args:

        Returns:
            None
        """

        if self.last_cmd_time is None:
            self.last_cmd_time = time.time()
        elif time.time() - self.last_cmd_time > self.min_command_time_gap:
            self.send_cmd(throttle=throttle, steering=steering)
            self.last_cmd_time = time.time()

    def send_cmd(self, throttle, steering):
        """
        Step 1: maps the cmd from agent_steering_range and agent_throttle_range to servo ranges
        Args:
            throttle: new throttle, in the range of agent_throttle_range
            steering: new steering, in the range of agent_steering_range

        Returns:
            None
        """

        throttle_send, steering_send = self.map_control(throttle, steering)
        try:
            self.send_cmd_helper(new_throttle=throttle_send, new_steering=steering_send)
        except KeyboardInterrupt as e:
            self.logger.debug("Interrupted Using Keyboard")
            exit(0)
        except Exception as e:
            self.logger.error(f"Something bad happened {e}")

    def send_cmd_helper(self, new_throttle, new_steering):
        """
        Actually send the command
        Args:
            new_throttle: new throttle, in the range of servo_throttle_range
            new_steering: new steering, in the range of servo_steering_range

        Returns:

        """
        if self.prev_throttle != new_throttle or self.prev_steering != new_steering:
            serial_msg = '& {} {}\r'.format(new_throttle, new_steering)
            self.logger.debug(f"Sending [{serial_msg.rstrip()}]")
            self.serial.write(serial_msg.encode('ascii'))
            self.prev_throttle = new_throttle
            self.prev_steering = new_steering

    def shutdown(self):
        """
        Ensure the device is shut down properly by sending neutral cmd 5 times
        Returns:

        """
        self.logger.debug('Shutting down')
        for i in range(5):
            self.logger.debug("Sending Neutral Command for safe shutdown")
            self.send_cmd_helper(new_throttle=1500, new_steering=1500)

    def map_control(self, throttle, steering) -> Tuple[int, int]:
        """
        Maps control from agent ranges to servo ranges
        Args:
            throttle: new throttle, in the range of agent_throttle_range
            steering: new steering, in the range of agent_steering_range

        Returns:
            Tuple of throttle and steering in servo ranges
        """
        return (int(np.interp(x=throttle,
                              xp=self.agent_throttle_range,
                              fp=self.servo_throttle_range)),
                int(np.interp(x=steering,
                              xp=self.agent_steering_range,
                              fp=self.servo_steering_range)))
