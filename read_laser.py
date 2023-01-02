#! /usr/bin/env python

import rclpy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
	#900 / 5 = 180
	regions = [
		min(min(msg.ranges[0:143]), 20),
		min(min(msg.ranges[144:287]), 20),
		min(min(msg.ranges[288:431]), 20),
		min(min(msg.ranges[432:575]), 20),
		min(min(msg.ranges[576:719]), 20),
	]
	rospy.loginfo(regions)

def main():
	print("Started read_laser node!")
	rclpy.init(args=args)

	node = rclpy.node.Node("LaserNode")
    	
	sub = rospy.Subscriber(LaserScan, '/Roomba/laser/scan', clbk_laser)

	rclpy.spin(node)
    
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
