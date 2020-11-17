#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import LaserScan
import tf
#from tf_msgs.msg import tfMessage
from geometry_msgs.msg import PoseStamped
import std_msgs.msg


spawn_pose = [0.0, 0.0, 0.0]
goal = [0.0, 0.0, 0.0]
posefile = None
epsilon = 0.05
current_pose = None

def pose_callback(data):
   global current_pose
   current_pose = data
   
def goal_callback(data):
   global goal, posefile
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
   #print("THIS IS THE GOAL: "+str(goal))
   goal = data
   if posefile is not None:
      f = open(posefile, 'a')
      f.write("GOAL: {}\n".format(str(goal)))
   #print("THIS IS THE INITIALIZED GOAL: "+str(goal))

def parse_message(trans, rot):
   trans.extend(rot)
   return str(trans)

def main():
   global posefile, current_pose
   rospy.loginfo("STARTED LOG FOR DD")
   # anonymous adds number
   rospy.init_node('pose_logger', anonymous=False)
   #scan_subscriber = rospy.Subscriber('/tf', tfMessage, pose_callback)
   #listener = tf.TransformListener()
   rospy.Subscriber("/move_base/goal", PoseStamped, goal_callback)
   # open pose file
   dirname = os.path.dirname(__file__)
   #dirname="/home/meriel/husky_ws/world_parser/"
   filename = os.path.join(dirname, 'poses.log')
   posefile = open(filename, 'w')
   # write a pose every second
   rate = rospy.Rate(1)
   trans = None
   rot = None
   while not rospy.is_shutdown():
      #rospy.loginfo("INSIDE ROSPY LOOP")
      try:
         (trans,rot) = listener.lookupTransform('odom', '/base_link', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         continue
      msg_string = parse_message(trans, rot)
      posefile.write(msg_string+"\n")
      rate.sleep()
   posefile.close()
   

if __name__ == '__main__':
   main()