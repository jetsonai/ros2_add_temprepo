#!/usr/bin/env python3

# Copyright 2024 JetsonAI CO., LTD.
#
# Author: Kate Kim

import rclpy 
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan 
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy

import threading
import rclpy

ranges_list = []
 
class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_sub_node')
        qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(
          LaserScan, 
          '/scan', 
          self.listener_callback, 
          qos)

        self.subscription # prevent unused variable warning
      

    def listener_callback(self, data):
        self.get_logger().info('Receiving lidar frame')

        global ranges_list
        ranges_list = data.ranges
        bsafe = self.safeDistance(ranges_list)
     
    def safeDistance(self, ranges):
        bSafe = 0
        if( len(ranges) > 90) :
            bSafe = 1
            for f in range(340,360):
                #print("ranges[{}] : {}".format(f, ranges[f]))
                if(ranges[f] <= 0.2 and ranges[f] != 0.0):
                    bSafe = 0
                    self.get_logger().info("<WARNING> ranges[{}] : {}".format(f, ranges[f]))
                    break
            if(bSafe == 1):
                for f in range(0,20):
                    #print("ranges[{}] : {}".format(f, ranges[f]))
                    if(ranges[f] <= 0.2 and ranges[f] != 0.0):
                        bSafe = 0
                        self.get_logger().info("<WARNING> ranges[{}] : {}".format(f, ranges[f]))
                        break
        print("bSafe:{}".format(bSafe))
        return bSafe

class BasicTopicPublisher(Node):

    def __init__(self):
        super().__init__('rostopic_pub')
        qos_profile = QoSProfile(depth=10)
        self.counter_publisher = self.create_publisher(String, 'counter', qos_profile)
        self.timer = self.create_timer(1, self.publish_counter_msg)
        self.count = 0

    def publish_counter_msg(self):
        msg = String()
        msg.data = 'Hello ROS2: {0}'.format(self.count)
        self.counter_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1
    
def main(args=None):

    rclpy.init(args=args)
    image_subscriber = LidarSubscriber() 
    rclpy.spin(image_subscriber)

    ###
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node2)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = node.create_rate(2)
    try:
        while rclpy.ok():
            print('Help me body, you are my only hope')
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()
    ###

    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
