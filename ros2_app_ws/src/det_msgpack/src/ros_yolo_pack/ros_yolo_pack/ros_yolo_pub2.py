#!/usr/bin/env python3

# Copyright 2024 JetsonAI CO., LTD.
#
# Author: Kate Kim

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge, CvBridgeError

import ultralytics
from PIL import Image
import numpy as np

from queue import Queue
import math
from rclpy.qos import QoSProfile
from det_msgpack.msg import DetInfo

gst_str = ("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

ultralytics.checks()

from ultralytics import YOLO

class DetTopicPublisher(Node):

    def __init__(self):
        super().__init__('ros_yolo_test_pub')

        self.trt_model = YOLO("/data/CHECK/yolov11n.engine")

        qos_profile = QoSProfile(depth=10)
        self.det_publisher = self.create_publisher(DetInfo, 'detinfo', qos_profile)
                
        self.get_logger().info('[[[DetTopicPublisher]]]')

        self.cap = cv2.VideoCapture(gst_str)
        if not (self.cap.isOpened()):
            print("Could not open video device")
        # To set the resolution

        frame_width = 640
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.det_width = int(frame_width * 0.4)
        self.det_cx_min = int(frame_width * 0.2)
        self.det_cx_max = int(frame_width * 0.8)

        cv2.namedWindow("YOLO11 ROS", cv2.WINDOW_GUI_EXPANDED)

        self.detMsg = DetInfo()
        self.detMsg.label = 'none'

        self.timer = self.create_timer(0.10, self.publish_det_msg)      

    def publish_det_msg(self):

        ret, frame = self.cap.read()
        if ret:
            results = self.trt_model.predict(frame, imgsz=480)

            self.getPersonDetInfo(results) 

            #self.get_logger().info(
            #        "label:{} xmin:{} ymin:{} xmax:{} ymax:{}".format(
            #         self.detMsg.label,self.detMsg.xmin, self.detMsg.ymin, 
            #self.detMsg.xmax, self.detMsg.ymax)) 
            if(self.detMsg.label == "person"):
                self.det_publisher.publish(self.detMsg)      
                self.get_logger().info('<Published> label: {0}'.format(self.detMsg.label))

            annotated_frame = results[0].plot()
            cv2.imshow("YOLO11 ROS", annotated_frame)
            cv2.waitKey(1)


    def getPersonDetInfo(self, results):

        for result in results:
            for box in result.boxes:
                clsID = int(box.cls)
                clsName = self.trt_model.names[clsID]
                print(clsName)
                self.detMsg.label = clsName

                if(clsName == "person"):
                    xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                    bbox_width = xmax - xmin
                    bbox_cx = xmin + int(bbox_width/2.0)
                    if (bbox_width > self.det_width and bbox_cx>self.det_cx_min and bbox_cx<self.det_cx_max):
                        
                        self.detMsg.xmin = xmin
                        self.detMsg.ymin = ymin
                        self.detMsg.xmax = xmax
                        self.detMsg.ymax = ymax                   
                        break
        return 

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()        


def main(args=None):
  
    rclpy.init()
    node = rclpy.create_node("cam_viewer")

    global bridge
    bridge = CvBridge()


    dettopic_publisher = DetTopicPublisher() 
    rclpy.spin(dettopic_publisher)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

# ros2 run cv_basics cam_node
