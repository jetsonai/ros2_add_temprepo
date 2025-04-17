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
from rclpy.qos import QoSProfile
from det_msgpack.msg import DetInfo

gst_str = ("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

packet_path = "/home/nvidia/ros_app_ws/src/ros_yolo_pack/"

data_path = packet_path + "yolo/challenge.mp4"
coco_path = packet_path +  "/yolo/coco.names"
cfg_path = packet_path +  "/yolo/yolov4-tiny.cfg"
weight_path = packet_path +  "/yolo/yolov4-tiny.weights"



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
    #darknet_width = darknet.network_width(network)
    #darknet_height = darknet.network_height(network)
    #darknet_image = darknet.make_image(darknet_width, darknet_height, 3)
    darknet_width = network_width(network)
    darknet_height = network_height(network)
    darknet_image = make_image(darknet_width, darknet_height, 3)
    
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (darknet_width, darknet_height),
                               interpolation=cv2.INTER_LINEAR)

    #darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    #detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = detect_image(network, class_names, darknet_image, thresh=thresh)

    detections_adjusted = []
    detMsg = DetInfo()
    for label, confidence, bbox in detections:
        bbox_adjusted = convert2original(frame, bbox, darknet_width, darknet_height)
        detections_adjusted.append((str(label), confidence, bbox_adjusted))
        detMsg.label = 'none'
        if(label == 'person'):
            bbox1 = bbox_adjusted
            #msg = DetInfo()
            detMsg.label = label
            detMsg.b_width = bbox1[2]
            detMsg.b_height = bbox1[3]          
            detMsg.center_x = int(bbox1[0] + bbox1[2]/2.0)
            detMsg.center_y = int(bbox1[1] + bbox1[3]/2.0)
            
    #darknet.free_image(darknet_image)
    #image = darknet.draw_boxes(detections_adjusted, frame, class_colors)
    free_image(darknet_image)
    image = draw_boxes(detections_adjusted, frame, class_colors)
    return image, detMsg




def model_detect(img):
    classIds, scores, boxes = model.detect(img, confThreshold=0.6, nmsThreshold=0.4)

    print(len(classIds))
    print(classIds)
    print(scores)

    for (classId, score, box) in zip(classIds, scores, boxes):
        cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                      color=(0, 255, 0), thickness=2)
        #text = '%s: %.2f' % (classes[classId[0]], score)
        text = '%s: %.2f' % (classes[classId], score)
        cv2.putText(img, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    color=(0, 255, 0), thickness=2)

    return img
    

   

class DetTopicPublisher(Node):

    def __init__(self):
        super().__init__('ros_yolo_node_pub')
        qos_profile = QoSProfile(depth=10)
        self.det_publisher = self.create_publisher(DetInfo, 'detinfo', qos_profile)
        #cap = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture(gst_str)
        if not (self.cap.isOpened()):
            print("Could not open video device")
        # To set the resolution
        
        width = 640
        height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        print('width:{} height:{}'.format(width,height))

        self.network, self.class_names, self.class_colors = load_network3(
                cfg_path, coco_path,
                weight_path, batch_size=1)

        self.thresh=.25 
                
        self.get_logger().info('[[[DetTopicPublisher]]]')
        self.timer = self.create_timer(0.33, self.publish_det_msg)      

    def publish_det_msg(self):
        # Capture frame-by-frame
        ret, cv_image = self.cap.read()
        
        output, detMsg = image_detection2(
                cv_image, self.network, self.class_names, self.class_colors, self.thresh)    

        self.det_publisher.publish(detMsg)
        #cv2.imshow('frame',output)
        #cv2.waitKey(1)
        
        #self.get_logger().info('Published label: {0}'.format(detMsg.label))
    
    def __del__(self):
        self.get_logger().info('[[[DetTopicPublisher __del__]]]')
        self.cap.release()
        cv2.destroyAllWindows()    
     
    '''    
    def yolo_cam_func(self):
   
        while(True):
            # Capture frame-by-frame
            ret, cv_image = cap.read()
            
            output= image_detection2(
                    cv_image, network, class_names, class_colors, thresh)

            # Display the resulting frame
            cv2.imshow('frame',output)
            
            #print('view video frame')
            # Waits for a user input to quit the application
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()
    '''           
    
def main(args=None):
  
    rclpy.init()
    #node = rclpy.create_node("ros_yolo_node_pub")
    
    print(coco_path)

    global bridge
    bridge = CvBridge()

    node = DetTopicPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
