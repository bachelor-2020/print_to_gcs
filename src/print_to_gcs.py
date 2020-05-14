#! /usr/bin/env python
# coding=utf-8
from __future__ import print_function

import rospy
import message_filters
import requests
import json

from sensor_msgs.msg import NavSatFix
from vision_msgs.msg import Detection2DArray


class Print_to_gcs:
    def __init__(self):
        rospy.init_node("findings")

        sub_object_detection = message_filters.Subscriber(
            "objects", Detection2DArray, queue_size=1)

        sub_gps = message_filters.Subscriber("mavros/global_position/global", NavSatFix, queue_size=10) # Fra Ardupilot
        #sub_gps = message_filters.Subscriber(
        #    "gps/fix", NavSatFix, queue_size=1
        #)  # FakeGPS

        msg_sync = message_filters.ApproximateTimeSynchronizer(
            [sub_object_detection, sub_gps], 10, 0.1)
        msg_sync.registerCallback(self.print_to_gcs)

        self.rate = rospy.Rate(2)

    def print_to_gcs(self, msg_obj, msg_gps):
        if msg_gps.status.status < 0:
            return

        if len(msg_obj.detections) > 0:
            for i in range(len(msg_obj.detections)):
                if msg_obj.detections[i].results[0].id == 1:
                    requests.post("http://app:5000/api/drones/0/findings",
                                  json={
                                      "position": {
                                          "latitude": msg_gps.latitude,
                                          "longitude": msg_gps.longitude
                                      },
                                      "image_id": 1
                                  })


if __name__ == "__main__":
    try:
        p = Print_to_gcs()
        while not rospy.is_shutdown():
            p.rate.sleep()

    except rospy.ROSInterruptException:
        pass
