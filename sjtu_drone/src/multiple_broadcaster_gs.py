#!/usr/bin/env python  
import roslib
roslib.load_manifest('sjtu_drone')
import rospy
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    
    rospy.init_node('gs_tf_broadcaster')
    br = tf.TransformBroadcaster()
    br.sendTransform((10,10, 0),
                     quaternion_from_euler(0,0,0),
                     rospy.Time.now(),
                     "gstation",
                     "world")
    rospy.spin()
    
