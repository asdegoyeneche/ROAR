import socketserver
from typing import Any, Optional, List, Dict
from socketserver import BaseServer
from triad_openvr import TriadOpenVR
import logging
from models import ViveTrackerMessage
import json
from pprint import pprint

class ViveTrackerServer(socketserver.BaseRequestHandler):
    def poll(self, tracker_name) -> Optional[ViveTrackerMessage]:
        tracker = self.get_tracker(tracker_name=tracker_name)

        if tracker is not None:
            message: Optional[ViveTrackerMessage] = self.create_tracker_message(tracker=tracker, tracker_name=tracker_name)
            return message
        else:
            triad_openvr = self.reconnect_triad_vr()

        return None

    @staticmethod
    def get_tracker(tracker_name):
        # print(triad_openvr.devices)
        return triad_openvr.devices.get(tracker_name, None)

    @staticmethod
    def create_tracker_message(tracker, tracker_name):

        try:
            euler = tracker.get_pose_euler()
            vel_x, vel_y, vel_z = tracker.get_velocity()
            x, y, z,  yaw, pitch, roll = euler
            message = ViveTrackerMessage(valid=True, x=x, y=y, z=z,
                                         yaw=yaw, pitch=pitch, roll=roll,
                                         vel_x=vel_x, vel_y=vel_y, vel_z=vel_z,
                                         device_name=tracker_name)
            return message
        except:
            print(f"Cannot find Tracker {tracker} is either offline or malfunctioned")
            triad_openvr = TriadOpenVR.reconnect_triad_vr()
            return None


    @staticmethod
    def construct_json_message(data: ViveTrackerMessage) -> str:
        json_data = json.dumps(data.json(), sort_keys=False, indent=2)
        json_data += ";"
        return json_data

    def get_trackers(self) -> Dict[str, Any]:
        trackers = {tracker_name: tracker for tracker_name, tracker in triad_openvr.devices.items()
                    if "tracker" in tracker_name}
        return trackers

    def handle(self):
        tracker_name = self.request[0].strip().decode()
        socket = self.request[1]
        message: Optional[ViveTrackerMessage] = self.poll(tracker_name=tracker_name)
        print("message = ", message)
        if message is not None:
            message = (self.construct_json_message(data=message))
            socket.sendto(message.encode(), self.client_address)
    @staticmethod
    def reconnect_triad_vr():
        print(
            f"Trying to reconnect to OpenVR to refresh devices. "
            f"Devices online:")
        triad_openvr = TriadOpenVR()
        pprint(triad_openvr.devices)
        print("")
        return triad_openvr

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    HOST, PORT = "192.168.1.8", 8000
    server = socketserver.UDPServer((HOST, PORT), ViveTrackerServer)
    triad_openvr = ViveTrackerServer.reconnect_triad_vr()
    print("Server Started. Listening for client connection")
    server.serve_forever()
