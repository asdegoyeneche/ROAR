import socket
import json
from models import ViveTrackerMessage
from triad_openvr import TriadOpenVR
import time
import logging
from typing import Optional, Dict, List, Any
from pathlib import Path
from datetime import datetime


class ViveTrackerRecorder:
    def __init__(self, output_dir_path: Path, tracker_name: str, interval: float = 1 / 250, max_retry: int = 10000):
        self.output_dir_path = output_dir_path

        if self.output_dir_path.exists() is False:
            self.output_dir_path.mkdir(parents=True, exist_ok=True)

        self.output_file = open(f"{(self.output_dir_path / 'RFS_Track.txt').as_posix()}",
                                'w')  # (self.output_dir_path / f"RFS_Track_{datetime.now()}.txt").open('w')
        self.tracker_name = tracker_name
        self.interval = interval
        self.max_retry = max_retry
        self.triad_openvr: Optional[TriadOpenVR] = None
        self.logger = logging.getLogger("Vive Tracker Publisher")
        self.initialize_openvr()

    def initialize_openvr(self):
        try:
            self.triad_openvr = TriadOpenVR()
            self.triad_openvr.print_discovered_objects()
        except Exception as e:
            self.logger.error(f"Failed to Initialize Socket. Make sure subscriber is running. Error: {e}")

    def poll(self) -> List[ViveTrackerMessage]:
        trackers = self.get_trackers()
        messages: List[ViveTrackerMessage] = []
        for tracker_name, tracker in trackers.items():
            euler = tracker.get_pose_euler()
            x, y, z, yaw, pitch, roll = euler
            message = ViveTrackerMessage(valid=True, x=x, y=y, z=z,
                                         yaw=yaw, pitch=pitch, roll=roll, device_name=tracker_name)
            messages.append(message)
        return messages

    def get_trackers(self) -> Dict[str, Any]:
        trackers = {tracker_name: tracker for tracker_name, tracker in self.triad_openvr.devices.items()
                    if "tracker" in tracker_name}
        return trackers

    def start_one(self):
        start = time.time()
        try:
            messages = self.poll()
            for message in messages:
                self.record(data=message)
            error_count = 0
        except TypeError as e:
            self.logger.error(f"Error: {e}\nUnable to Connect to Vive Tracker, trying again {error_count}. "
                              f"Try Moving the Tracker to reactivate it")
            error_count += 1
        except ConnectionAbortedError:
            self.logger.error("Failed to send")
        except ConnectionResetError as e:
            self.logger.error("Client Disconnected")
            self.s = None

        sleep_time = self.interval - (time.time() - start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    def start(self):
        error_count = 0
        self.logger.info("Tracking Started")
        while error_count < self.max_retry:
            start = time.time()
            try:
                messages = self.poll()
                for message in messages:
                    self.record(data=message)
                error_count = 0
            except TypeError as e:
                self.logger.error(f"Error: {e}\nUnable to Connect to Vive Tracker, trying again {error_count}. "
                                  f"Try Moving the Tracker to reactivate it")
                error_count += 1
            except ConnectionAbortedError:
                self.logger.error("Failed to send")
            except ConnectionResetError as e:
                self.logger.error("Client Disconnected")
                self.s = None

            sleep_time = self.interval - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def record(self, data: ViveTrackerMessage):
        if data.device_name == self.tracker_name:
            x, y, z, roll, pitch, yaw = self.to_right_handed(data.x, data.y, data.z, data.roll, data.pitch, data.yaw)
            recording_data = f"{x}, {y},{z},{roll},{pitch},{yaw}"
            m = f"Recording: {recording_data}"
            self.logger.info(m)
            self.output_file.write(f"{x}, {y},{z},{roll},{pitch},{yaw}\n")
        # data_to_send = self.construct_json_message(data=data)

    @staticmethod
    def to_right_handed(x, y, z, roll, pitch, yaw):
        return x, y, z, roll, pitch, yaw

    @staticmethod
    def construct_json_message(data: ViveTrackerMessage) -> str:
        json_data = json.dumps(data.json(), sort_keys=False, indent=2)
        json_data += ";"
        return json_data


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    vive_tracker_publisher = ViveTrackerRecorder(output_dir_path=Path("./data"), tracker_name="tracker_1")
    vive_tracker_publisher.start_one()
