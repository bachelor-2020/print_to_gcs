#! /usr/bin/env python
from __future__ import print_function
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import NavSatFix
import rospy
from rospy_message_converter import json_message_converter
import json
import message_filters


class Print_to_gcs:
    def __init__(self):
        rospy.init_node("print_to_gcs")

        sub_object_detection = rospy.Subscriber(
            "/objects", Detection2DArray, self.print_object_to_gcs
        )
        sub_gps = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.print_gps_to_gcs
        )

        self.rate = rospy.Rate(2)

    def print_object_to_gcs(self, msg):
        # msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        # id = msg_dict["detections"][1]["results"][0]["id"]
        # msg_json = json_message_converter.convert_ros_message_to_json(msg)
        # print(json.dumps(msg_json, sort_keys=True, indent=4, separators=(",", ": ")))

        if len(msg.detections) > 0:
            print("Antall objekter funnet: ", len(msg.detections))
            for i in range(len(msg.detections)):
                print("Objekt: ", i, ": ")
                # for j in range(len(msg.detections[i].results)): # I test er len alltid 1
                print("id: ", msg.detections[i].results[0].id)

                if msg.detections[i].results[0].id == 1:
                    print("Person funnet!")

        else:
            print("Ingen objekter funnet")

    def print_gps_to_gcs(self, msg):
        print(msg.latitude)
        print(msg.longitude)


if __name__ == "__main__":
    try:
        p = Print_to_gcs()
        while not rospy.is_shutdown():
            p.rate.sleep()

    except rospy.ROSInterruptException:
        pass
