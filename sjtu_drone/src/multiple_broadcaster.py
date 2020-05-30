#!/usr/bin/env python  
import roslib

import rospy
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist

def handle_turtle_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    roll=pitch=yaw=0.0
    queue_orient=msg.pose.pose.orientation
    list_orient=[queue_orient.x,queue_orient.y,queue_orient.z,queue_orient.w]
    (r,p,y)=euler_from_quaternion (list_orient)
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     quaternion_from_euler(r,p,y),
                     rospy.Time.now(),
                     robotname,
                     "world")
    #print(robotname)

if __name__ == '__main__':
    
    rospy.init_node('robot_tf_broadcaster')
    robotname = rospy.get_param('~robot')
    rospy.Subscriber('/%s/odom'%robotname,
                     Odometry,
                     handle_turtle_pose,
                     robotname)
    rospy.spin()