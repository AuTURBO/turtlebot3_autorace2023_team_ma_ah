#!/usr/bin/env python3
import sys
import cv2 
import imutils
import time

from ma_ah_perception.yoloDet  import YoloTRT

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D


class YOLOV7_TRT_ROS:
    def __init__(self, conf_thresh, img_size, visualize, input_img_topic, pub_topic, output_img_topic, categories):
        self.conf_thresh = conf_thresh
        self.img_size = img_size
        self.visualize = visualize
        self.input_img_topic = input_img_topic
        self.pub_topic = pub_topic
        self.output_img_topic = output_img_topic
        self.categories = categories
        self.cv_image = None
        
        self.prevTime = 0 

        self.model = YoloTRT(library="../../JetsonYoloV7-TensorRT/yolov7/build/libmyplugins.so", engine="../../JetsonYoloV7-TensorRT/yolov7/build/yolov7-tiny.engine", conf=self.conf_thresh, categories=self.categories, yolo_ver="v7")

        self.img_subscriber = rospy.Subscriber(self.input_img_topic, CompressedImage, self.img_cb)
        self.detection_result_publisher = rospy.Publisher(self.pub_topic, Detection2DArray, queue_size=10) 
    def inference(self):
        if self.cv_image is None:
            print("image None")
        else:
            frame = imutils.resize(self.cv_image, width=640)
            detections, t = self.model.Inference(frame)

            detection_msg_array = Detection2DArray()

            for obj in detections:
                detection2d_msg = Detection2D()
                obj_hypothesis = ObjectHypothesisWithPose()
                obj_num = [i for i in range(len(self.categories)) if obj['class'] in self.categories[i]]
                obj_hypothesis.id = obj_num[0]
                obj_hypothesis.score = obj['conf']

                detection2d_msg.bbox.size_x = obj['box'][2] - obj['box'][0]
                detection2d_msg.bbox.size_y = obj['box'][3] - obj['box'][1]

                pose_x = obj['box'][0] + detection2d_msg.bbox.size_x // 2
                pose_y = obj['box'][1] + detection2d_msg.bbox.size_y // 2
                
                detection2d_msg.bbox.center.x = pose_x
                detection2d_msg.bbox.center.y = pose_y

                detection2d_msg.results.append(obj_hypothesis)

                detection_msg_array.detections.append(detection2d_msg) 

                # print(obj['class'], obj['conf'], obj['box'])
            # print("FPS: {} sec".format(1/t))

            detection_msg_array.header.stamp = rospy.Time.now()

            # print(detection_msg_array)

            self.detection_result_publisher.publish(detection_msg_array)

            cv2.imshow("frame", frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                sys.exit(0)

    def img_cb(self, img_msg):
        """ callback function for publisher """
        try:
            # cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8") <- Image
            self.cv_image = CvBridge().compressed_imgmsg_to_cv2(img_msg, "bgr8") # <- Compressed Image
        except CvBridgeError as e:
            print(e)
        else:            
            curTime = time.time()

            sec = curTime - self.prevTime
            self.prevTime = curTime

            # 1 / time per frame
            fps = 1/(sec)

            print(f"Time {sec} ")
            print(f"Estimated fps {fps} ") 

            str = "FPS : %0.1f" % fps

            
            cv2.putText(self.cv_image, str, (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

            #cv2.imshow("Output", self.cv_image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                sys.exit(0)
            #cv2.imshow("Image window", cv_image)
            # cv2.waitKey(1)


