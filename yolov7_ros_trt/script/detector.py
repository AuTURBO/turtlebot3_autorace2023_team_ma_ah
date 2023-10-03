#!/usr/bin/env python3

import rospy
import sys
import os
from yolov7_ros import YOLOV7_TRT_ROS


if __name__ == "__main__":
    rospy.init_node("yolov7_trt_node", log_level=rospy.INFO)

    ns = rospy.get_name() + "/"

    # input_img_topic = rospy.get_param(ns + "input_img_topic", "/usb_cam/image_raw")
    # output_img_topic = rospy.get_param(ns + "output_img_topic", "/yolov7/image_raw")
    # output_topic = rospy.get_param(ns + "output_topic", "yolo/detections")
    # conf_thresh = rospy.get_param(ns + "conf_thresh", 0.25)
    # img_size = rospy.get_param(ns + "img_size", 640)
    # visualize = rospy.get_param(ns + "visualize", True)

    input_img_topic = "/camera/image/compressed"
    output_img_topic = "/yolov7/image_raw"
    output_topic = "yolo/detections"
    conf_thresh = 0.3
    img_size = 320
    visualize = True

    categories = ["intersection", "left", "obstacle", "parking", "right", "tune", "tunnel"]

    detector = YOLOV7_TRT_ROS(
        conf_thresh=conf_thresh,
        img_size=img_size,
        visualize=visualize,
        input_img_topic=input_img_topic,
        pub_topic=output_topic,
        output_img_topic=output_img_topic,
        categories= categories,)
    
    while True:
        detector.inference() # The CUDA inference function must be run on the main thread.

    rospy.spin()