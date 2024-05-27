#!/usr/bin/env python3
                      

import threading
import time

import rospy
import pymavlink.mavutil


class OccupierUdpPort(threading.Thread):
    '''Class is meant to occupy a Udp port. For some SITLs it is important
    to have a datalink on e.g. 14550 port to just take off (btw QGC
    utilizes 14550).

    To run le script as node paste this within your launch-file:
    <node name="occupier_udp_port" pkg="chvk_tools" type="occupier_udp_port.py" output="log"/>
    '''


    def __init__(self, port: int = 14550, interval: float = 1.) -> None:
        super(OccupierUdpPort, self).__init__()
        self.url = '0.0.0.0:' + str(port)
        self.interval = interval
        self.running = False


    def run(self) -> None:

        self.running = True
        self.mavlink_connection = pymavlink.mavutil.mavlink_connection(
            self.url, autoreconnect=True, baud=57600)
        self.mavlink_connection.wait_heartbeat()

        while self.running:
            self.mavlink_connection.mav.heartbeat_send(
                pymavlink.mavutil.mavlink.MAV_TYPE_GCS,
                pymavlink.mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0,
                0,
                0)
            time.sleep(self.interval)


    def stop(self) -> None:
        self.running = False


def main() -> None:

    rospy.init_node('occupier_udp_port', disable_signals=True)

    occupierUdpPort = OccupierUdpPort()
    rospy.loginfo('Port 14550 has been occupied.')
    occupierUdpPort.run()


if __name__ == '__main__':
    main()
