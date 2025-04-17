#!/usr/bin/env python
#
# Copyright (c) 2024 JetsonAI CO., LTD.
# Author: Kate Kim

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from det_msgpack.msg import DetInfo

class DetTopicSubscriber(Node):

    def __init__(self):
        super().__init__('ros_yolo_test_sub')
        qos_profile = QoSProfile(depth=10)
        self.basicsrv_subscriber = self.create_subscription(
            DetInfo, 'detinfo',
            self.subscribe_detinfo,
            qos_profile)

    def subscribe_detinfo(self, msg):
        self.get_logger().info(" label:{} xmin:{} ymin:{} xmax:{} ymax:{}".format(msg.label,
                msg.xmin, msg.ymin, msg.xmax, msg.ymax)) 

def main(args=None):
    rclpy.init(args=args)
    node = DetTopicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
