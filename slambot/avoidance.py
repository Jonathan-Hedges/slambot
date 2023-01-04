#! /usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

boundary = [
    0.2451727901,
    0.2428336146,
    0.250057401,
    0.2683529899,
    0.3020615468,
    0.3613358746,
    0.4,
    0.4,
    0.4,
    0.4,
    0.3613358746,
    0.3020615468,
    0.2683529899,
    0.250057401,
    0.2428336146,
    0.2451727901,
]

class AvoidanceNode(Node):

    def __init__(self):
        super().__init__('avoidance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, '/slambot/cmd_demo', 10)

    def listener_callback(self, msg):
        print("Successful sub!")
        segments = [
            min(min(msg.ranges[0:44]), 1),
            min(min(msg.ranges[45:89]), 1),
            min(min(msg.ranges[90:134]), 1),
            min(min(msg.ranges[135:179]), 1),
            min(min(msg.ranges[180:224]), 1),
            min(min(msg.ranges[225:269]), 1),
            min(min(msg.ranges[270:314]), 1),
            min(min(msg.ranges[315:359]), 1),
            min(min(msg.ranges[360:404]), 1),
            min(min(msg.ranges[405:449]), 1),
            min(min(msg.ranges[450:494]), 1),
            min(min(msg.ranges[495:539]), 1),
            min(min(msg.ranges[540:584]), 1),
            min(min(msg.ranges[585:629]), 1),
            min(min(msg.ranges[630:674]), 1),
            min(min(msg.ranges[675:719]), 1),
        ]
        flag = True
        left_flag = True
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0
        linear_speed = 0.2
        angular_speed = 0.3

        closest = 1
	
        state_description = ""

        for x in range(16):
    	    if segments[x] < boundary[x]:
            	flag = False

        if flag:
            state_description = 'objects: none'
            linear_x = linear_speed
            angular_z = 0.0		

        elif not flag:
            left_obj = 0
            right_obj = 0

            for x in range(0, 7):
                if segments[x] < closest:
                    closest = segments[x]
                    left_flag = False
    
            for x in range(9, 16):
                if segments[x] < closest:
                    closest = segments[x]
                    left_flag = True
    
            if not left_flag:
                state_description = 'objects: right'
                linear_x = 0.0
                angular_z = angular_speed
    
            elif left_flag:
                state_description = 'objects: left'
                linear_x = 0.0
                angular_z = -angular_speed
    
            else:
                state_description = 'unknown case'
                rospy.loginfo(segments)
    
        print(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

def main(args=None):
    print("this is a new script")
    rclpy.init(args=args)
    
    avoidance_node = AvoidanceNode()

    rclpy.spin(avoidance_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

