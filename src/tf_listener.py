#!/usr/bin/env python

import sys
import rospy
import roslib
import tf
import geometry_msgs.msg

""" Tf listener from panda base link
to object 33 (mug). To find position/ orientation.
"""

class tf_listenner_class():
  def __init__(self):
    # Initialize node
    rospy.init_node('panda_tf_listener')
    # Create listener
    listener = tf.TransformListener()
    # Get time
    now = rospy.Time.now()
    while not rospy.is_shutdown():
      # Wait for transform
      listener.waitForTransform('/panda_link0', '/tag_0', rospy.Time(), rospy.Duration(5.0)) 
      # Get transform
      (translation, rotation) = listener.lookupTransform('/panda_link0', '/tag_0', rospy.Time(0)) 
      # Test
      print "translation: ", translation
      print "rotation: ", rotation 


if __name__ == '__main__':
  #try:
  tf_listenner_class()
  #except (tf.LookupException):
   # continue
  # Test
  print "Transform was found"
  # For orientation
  quaternion = (rotation[0], rotation[1], rotation[2], rotation[3])
  euler = tf.transformations.euler_from_quaternion(quaternion)
  # Get values
  roll = euler[0]
  pitch = euler[1]
  yaw = euler[2]
  # Test
  #print("roll: " roll, " pitch: ", pitch, " yaw: ", yaw)

  rate.sleep()
