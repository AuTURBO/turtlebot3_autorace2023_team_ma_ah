#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import sys 
from vision_msgs.msg import Detection2DArray
from collections import deque
from std_msgs.msg import String

class Yolo_filter:
    def __init__(self, image_topic, yolo_detection_topic, filted_detection_topic):
        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)
        
        self.bbox_size_threshold = 3500
        self.detection_buffer = deque()
        self.detection_buffer_len = 10
        self.count_threshold = 5
        self.filtered_detection_list = []
        self.labels = ("boom_barrier", "intersection", "left", "no_entry", "obstacle", \
                       "parking", "right", "stop", "traffic_light", "tunnel")


        # self.img_subscriber = rospy.Subscriber(image_topic, CompressedImage, self.img_cb)
        self.yolo_detection_subscriber =  rospy.Subscriber(yolo_detection_topic, Detection2DArray, self.detection_cb)
        self.filted_detection_publisher = rospy.Publisher(filted_detection_topic, String, queue_size=10)

    def img_cb(self, img_msg):
        try:
            # cv_image = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
            cv_image = CvBridge().compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.process(cv_image)

    def detection_cb(self, detection_msg):
        bbox_size_filtered_detection_msg = [] # 1 frame
        for i in range(len(detection_msg.detections)):
            obj_bbox_size =  detection_msg.detections[i].bbox.size_x * detection_msg.detections[i].bbox.size_y
            #print(f"bbox size : {obj_bbox_size}")
            if obj_bbox_size > self.bbox_size_threshold:
                bbox_size_filtered_detection_msg.append(detection_msg.detections[i])

        if len(self.detection_buffer) < self.detection_buffer_len:     
            self.detection_buffer.append(bbox_size_filtered_detection_msg)
        else:
            self.detection_buffer.popleft()
            self.detection_buffer.append(bbox_size_filtered_detection_msg)
        # print(len(self.detection_buffer))

        count_list = [0] * len(self.labels)
        # 최근 10번 인퍼런스 결과에서 일정값 이상 검출되었을때 최종 결과로 확정
        for i in range(len(self.detection_buffer)): # object num = i

            print(f"Object number: {len(self.detection_buffer[i])}")
            for j in range(len(self.detection_buffer[i])): # each object
                obj_id = self.detection_buffer[i][j].results[0].id
                obj_score = self.detection_buffer[i][j].results[0].score

                if self.labels[obj_id] in self.labels:
                    print(f"Detection : {self.labels[obj_id]}")
                    # print("Check!!")
                    count_list[obj_id] +=1
                # print(self.labels[obj_id])
                # strFormat = ''.join('%-15s' for i in range(len(self.labels)))
                # strFormat.join('\n')
                # print(strFormat)
                # strOut = strFormat % self.labels
                # print(strOut)
                #'%-10s%-10s%-10s\n'

                #print(self.labels)
                #print(count_list)
        idx = [i for i, x in enumerate(count_list) if x > self.count_threshold]

        result = str([self.labels[i] for i in idx])

        print(result)

        msg = String()
        msg.data = result
        
        self.filted_detection_publisher.publish(msg)



if __name__ == '__main__':
    rospy.init_node("yolo_filter")
    
    image_topic = "/camera/image/compressed"
    yolo_detection_topic = "/yolov7/detection"
    filted_detection_topic = "/filtered/detection"
    yolo_filter = Yolo_filter(image_topic, yolo_detection_topic, filted_detection_topic)

    rospy.spin()