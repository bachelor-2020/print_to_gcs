#! /usr/bin/env python
# coding=utf-8
from __future__ import print_function

import rospy

import cv2
import base64

from cv_bridge import CvBridge
import message_filters
import requests
import json

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

# IP adresse for bakkestasjon
GCS_IP = "192.168.4.2"

class Print_to_gcs:
    def __init__(self):
        rospy.init_node("findings")

        sub_detection_image = message_filters.Subscriber(
            "darknet_ros/detection_image", Image, queue_size=1
        )

        sub_detection_box = message_filters.Subscriber(
            "darknet_ros/bounding_boxes", BoundingBoxes, queue_size=1
        )

        # sub_gps = message_filters.Subscriber(
        #     "mavros/global_position/global", NavSatFix, queue_size=1
        # )  # Fra Ardupilot

        sub_gps = message_filters.Subscriber(
            "gps/fix", NavSatFix, queue_size=1
        )  # FakeGPS

        msg_sync = message_filters.ApproximateTimeSynchronizer(
            [sub_detection_image, sub_detection_box, sub_gps], 10, 0.1
        )
        msg_sync.registerCallback(self.print_to_gcs)

        self.rate = rospy.Rate(2)

    def print_to_gcs(self, msg_image, msg_box, msg_gps):
        if msg_gps.status.status < 0:
            print("Ikke fått gps fix".decode("utf-8"))
            return

        if len(msg_box.bounding_boxes) > 0:
            print("\nAntall objekter funnet: ", len(msg_box.bounding_boxes))
            for i in range(len(msg_box.bounding_boxes)):

                if msg_box.bounding_boxes[i].id == 0:

                    bridge = CvBridge()

                    img = bridge.imgmsg_to_cv2(msg_image, "bgr8")
                    retval, jpg = cv2.imencode('.jpg', img)

                    requests.post("http://" + GCS_IP + ":5000/api/findings",
                                  json={
                                      "position": {
                                          "latitude": msg_gps.latitude,
                                          "longitude": msg_gps.longitude
                                      },
                                      "image": base64.b64encode(jpg)
                                  })


if __name__ == "__main__":
    try:
        p = Print_to_gcs()
        while not rospy.is_shutdown():
            p.rate.sleep()

    except rospy.ROSInterruptException:
        pass
