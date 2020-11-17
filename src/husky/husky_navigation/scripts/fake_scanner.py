#!/usr/bin/env python
from sensor_msgs.msg import LaserScan 
import rospy
import std_msgs.msg

def scan_callback(data):
   return

def main():
   rospy.init_node('fake_scan', anonymous=True)
   scan_subscriber = rospy.Subscriber('/remapped_scan', LaserScan, scan_callback)
   scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)
   rate = rospy.Rate(3)
   m = 720
   empty_scan_msg = LaserScan()
   empty_scan_msg.angle_min = -2.3561899662
   empty_scan_msg.angle_max = 2.3561899662
   empty_scan_msg.angle_increment = 0.0065540750511
   empty_scan_msg.time_increment = 0.0
   empty_scan_msg.scan_time = 0.0 
   empty_scan_msg.range_min = 0.10000000149
   empty_scan_msg.range_max = 30.0
   empty_scan_msg.ranges = [0.0 for i in range(m)]
   empty_scan_msg.intensities = [float('inf') for i in range(m)]
   h = std_msgs.msg.Header()
   h.frame_id = "base_laser"
   for i in range(10):
      h.stamp = rospy.Time.now()
      h.seq += 1
      empty_scan_msg.header = h
      scan_publisher.publish(empty_scan_msg)
      rate.sleep()
   #infinite range, zero intensity --> can't detect anyting
   empty_scan_msg.ranges = [float('inf') for i in range(m)]
   empty_scan_msg.intensities = [0.0 for i in range(m)]
   while not rospy.is_shutdown():
      h.stamp = rospy.Time.now()
      h.seq += 1
      empty_scan_msg.header = h
      scan_publisher.publish(empty_scan_msg)
      rate.sleep()

if __name__ == '__main__':
   main()