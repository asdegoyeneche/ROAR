import socket
import sys
import logging
from typing import Optional
try:
    from ROAR_Jetson.vive.models import ViveTrackerMessage
except:
    from models import ViveTrackerMessage
import json
import time


class ViveTrackerClient:
    def __init__(self, host, port, tracker_name, interval=0.1, buffer_length=1024):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self.interval = interval
        self.buffer_length = buffer_length
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(5)
        self.latest_tracker_message = None
        self.logger = logging.getLogger(f"Vive Tracker Client [{self.tracker_name}]")
        self.logger.info("Tracker Initialized")

    def update(self):
        self.logger.info(f"Start Subscribing to [{self.host}:{self.port}] "
                         f"for [{self.tracker_name}] Vive Tracker Updates")
        buffer: str = ""
        while True:
            start = time.time()
            try:
                self.socket.sendto(bytes(self.tracker_name + "\n", "utf-8"), (self.host, self.port))
                received_message = str(self.socket.recv(1024), "utf-8")
                if received_message != '':

                    found_ending_char = ';' in received_message
                    if found_ending_char:
                        buffer = buffer + received_message[:-1]
                        self.update_latest_tracker_message(buffer)
                        buffer = ""
                    else:
                        buffer += received_message
                else:
                    pass
                end = time.time()
                interval = end - start
                if interval < self.interval:
                    time.sleep(self.interval - interval)
            except socket.timeout:
                self.logger.error("Timed out")
            except ConnectionResetError as e:
                self.logger.debug("Connection reset. Retrying")
            except OSError as e:
                pass
            except KeyboardInterrupt:
                exit(1)

    def run_threaded(self):
        pass

    def shutdown(self):
        self.socket.close()

    def update_latest_tracker_message(self, buffer):
        try:
            d = json.loads(json.loads(buffer))
            vive_tracker_message = ViveTrackerMessage.parse_obj(d)
            if vive_tracker_message.device_name == self.tracker_name:
                self.latest_tracker_message = vive_tracker_message
            # self.logger.info(self.latest_tracker_message)
        except Exception as e:
            self.logger.error(f"Error: {e} \nMaybe it is related to unable to parse buffer [{buffer}]. ")


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    HOST, PORT = "192.168.1.5", 8000
    client = ViveTrackerClient(host=HOST, port=PORT, tracker_name="tracker_1")
    client.update()
