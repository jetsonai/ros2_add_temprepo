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

class DetTopicPublisher(Node):

    def __init__(self):
        super().__init__('ros_yolo_test_pub')
        qos_profile = QoSProfile(depth=10)
        self.det_publisher = self.create_publisher(DetInfo, 'detinfo', qos_profile)
                
        self.get_logger().info('[[[DetTopicPublisher]]]')
        self.timer = self.create_timer(0.33, self.publish_det_msg)      

    def publish_det_msg(self, detMsg):
        self.get_logger().info(
                "label:{} xmin:{} ymin:{} xmax:{} ymax:{}".format(
                 detMsg.label,detMsg.xmin, detMsg.ymin, detMsg.xmax, detMsg.ymax)) 
        if detMsg.label == 'person':
            self.det_publisher.publish(detMsg)      
            self.get_logger().info('<Published> label: {0}'.format(detMsg.label))


ultralytics.checks()

from ultralytics import YOLO

trt_model = YOLO("/data/CHECK/yolov11n.engine")

def main(args=None):
  
    rclpy.init()
    node = rclpy.create_node("cam_viewer")

    global bridge
    bridge = CvBridge()

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(gst_str)
    if not (cap.isOpened()):
        print("Could not open video device")
    # To set the resolution
    frame_width = 640
    det_width = int(frame_width * 0.4)
    det_cx_min = int(frame_width * 0.2)
    det_cx_max = int(frame_width * 0.8)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    dettopic_publisher = DetTopicPublisher() 

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # Display the resulting frame
        #cv2.imshow('preview',frame)
        if ret:
            
            results = trt_model.predict(frame, imgsz=480)
            detMsg = DetInfo()
            detMsg.label = 'none'
            
            for result in results:
                for box in result.boxes:
                    clsID = int(box.cls)
                    clsName = trt_model.names[clsID]
                    print(clsName)
                    xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                    bbox_width = xmax - xmin
                    bbox_cx = xmin + int(bbox_width/2.0)
                    if(clsName == "person"):
                        if (bbox_width > det_width and bbox_cx>det_cx_min and bbox_cx<det_cx_max):
                            detMsg.label = clsName
                            detMsg.xmin = xmin
                            detMsg.ymin = ymin
                            detMsg.xmax = xmax
                            detMsg.ymax = ymax                   
                            break
            
            dettopic_publisher.publish_det_msg(detMsg)
            
            annotated_frame = results[0].plot()

            cv2.imshow("YOLOv11", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()

# ros2 run cv_basics cam_node
