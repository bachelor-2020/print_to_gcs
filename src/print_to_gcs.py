#! /usr/bin/env python
# coding=utf-8
from __future__ import print_function

import PIL.Image
import numpy as np

import rospy
import message_filters

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes


class Print_to_gcs:
    def __init__(self):
        rospy.init_node("print_to_gcs")

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
        print("\nGPS data:")
        print("\tTidsstempel:")
        print("\t\tSekunder:", msg_gps.header.stamp.secs)
        print("\t\tNanosekunder:", msg_gps.header.stamp.nsecs)
        print(
            "\tBreddegrad:", msg_gps.latitude
        )  # Positivt tall er nord for ekvator, negativt er sør
        print(
            "\tLengdegrad:", msg_gps.longitude
        )  # Positivt tall er øst for Greenwich, negativt vest
        print(
            "\tHøyde:".decode("utf-8"), msg_gps.altitude
        )  #  NaN hvis ingen høyde er tilgjengelig
        print("\tStatus gps:", msg_gps.status.status, end="")
        if msg_gps.status.status >= 0:
            print(" -- OK")
        else:
            print(" -- Ikke fått gps fix".decode("utf-8"))

        if len(msg_box.bounding_boxes) > 0:
            print("\nAntall objekter funnet: ", len(msg_box.bounding_boxes))
            for i in range(len(msg_box.bounding_boxes)):
                print("\tObjekt: ", i, ": ")
                print("\tTid: ", msg_box.bounding_boxes[i].id)

                if msg_box.bounding_boxes[i].id == 0:
                    print("\t\t** Person funnet! **")

                    img = PIL.Image.frombytes(
                        "RGB",
                        (msg_image.height, msg_image.width),
                        msg_image.data,
                        "raw",
                    )
                    print(type(img))
                    print(img.mode, img.size)
                    img.save("/tmp/test{}.png".format(msg_gps.header.stamp.secs))

        else:
            print("\nIngen objekter funnet")


if __name__ == "__main__":
    try:
        p = Print_to_gcs()
        while not rospy.is_shutdown():
            p.rate.sleep()

    except rospy.ROSInterruptException:
        pass
