#!/usr/bin/env python3

# Copyright 2024 JetsonAI CO., LTD.
#
# Author: Kate Kim

import rclpy 
from rclpy.node import Node 
import cv2, time
import numpy as np
from ros_yolo_pack.darknet import *
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from queue import Queue
import numpy as np
import math
from rclpy.qos import QoSProfile
from det_msgpack.msg import DetInfo

gst_str = ("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

packet_path = "/home/nvidia/ros_app_ws/src/ros_yolo_pack/"

data_path = packet_path + "yolo/challenge.mp4"
coco_path = packet_path +  "/yolo/coco.names"
cfg_path = packet_path +  "/yolo/yolov4-tiny.cfg"
weight_path = packet_path +  "/yolo/yolov4-tiny.weights"

class DetTopicPublisher(Node):

    def __init__(self):
        super().__init__('ros_yolo_test_pub')
        qos_profile = QoSProfile(depth=10)
        self.det_publisher = self.create_publisher(DetInfo, 'detinfo', qos_profile)
                
        self.get_logger().info('[[[DetTopicPublisher]]]')
        self.timer = self.create_timer(0.33, self.publish_det_msg)      

    def publish_det_msg(self, detMsg):
        if detMsg.label == 'person':
            self.det_publisher.publish(detMsg)      
            self.get_logger().info('Published label: {0}'.format(detMsg.label))
    

def convert2relative(bbox, darknet_width, darknet_height):
    """
    YOLO format use relative coordinates for annotation
    """
    x, y, w, h  = bbox

    _height     = darknet_height
    _width      = darknet_width
    return x/_width, y/_height, w/_width, h/_height

def convert2original(image, bbox, darknet_width, darknet_height):
    x, y, w, h = convert2relative(bbox, darknet_width, darknet_height)

    image_h, image_w, __ = image.shape

    orig_x       = int(x * image_w)
    orig_y       = int(y * image_h)
    orig_width   = int(w * image_w)
    orig_height  = int(h * image_h)

    bbox_converted = (orig_x, orig_y, orig_width, orig_height)

    return bbox_converted

def image_detection2(frame, network, class_names, class_colors, thresh):
    # Darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    darknet_width = network_width(network)
    darknet_height = network_height(network)
    darknet_image = make_image(darknet_width, darknet_height, 3)
    
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (darknet_width, darknet_height),
                               interpolation=cv2.INTER_LINEAR)

    copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = detect_image(network, class_names, darknet_image, thresh=thresh)

    detections_adjusted = []
    detMsg = DetInfo()
    detMsg.label = 'none'

    for label, confidence, bbox in detections:
        bbox_adjusted = convert2original(frame, bbox, darknet_width, darknet_height)
        detections_adjusted.append((str(label), confidence, bbox_adjusted))
        if(label == 'person'):
            cx, cy, bw, bh = bbox_adjusted
            detMsg.label = 'person'
            detMsg.center_x = cx
            detMsg.center_y = cy
            detMsg.b_width = bw
            detMsg.b_height = bh   
            
    free_image(darknet_image)
    image = draw_boxes(detections_adjusted, frame, class_colors)
    return image, detMsg
    
def main(args=None):
  
    rclpy.init()    
    print(coco_path)

    global bridge
    bridge = CvBridge()

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(gst_str)
    if not (cap.isOpened()):
        print("Could not open video device")
    # To set the resolution
    
    width = 640
    height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    print('width:{} height:{}'.format(width,height))

    network, class_names, class_colors = load_network3(
            cfg_path, coco_path,
            weight_path, batch_size=1)

    thresh=.25    
    dettopic_publisher = DetTopicPublisher() 

    while(True):
        # Capture frame-by-frame
        ret, cv_image = cap.read()
        
        output, detMsg = image_detection2(
                cv_image, network, class_names, class_colors, thresh)
                
        dettopic_publisher.publish_det_msg(detMsg)

        # Display the resulting frame
        cv2.imshow('frame',output)
        
        #print('view video frame')
        # Waits for a user input to quit the application
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

    dettopic_publisher.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
