#!/usr/bin/env python  
import roslib
roslib.load_manifest('sjtu_drone')
import rospy
from geometry_msgs.msg import Pose
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import subprocess 

def handle_turtle_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    roll=pitch=yaw=0.0
    queue_orient=msg.orientation
    list_orient=[queue_orient.x,queue_orient.y,queue_orient.z,queue_orient.w]
    (r,p,y)=euler_from_quaternion (list_orient)
    br.sendTransform((msg.position.x, msg.position.y, 0),
                     quaternion_from_euler(r,p,y),
                     rospy.Time.now(),
                     robotname,
                     "world")

if __name__ == '__main__':
    rospy.init_node('drone_broadcaster')
    rospy.set_param('/drone_status',0)
    subprocess.call('rosparam load /home/rahul/catkin_ws/src/sjtu_drone/p.yaml',shell=True)
    robotname = rospy.get_param('~robot')
    rospy.Subscriber('/drone/gt_pose',
                    Pose,
                    handle_turtle_pose,
                    robotname)
    rospy.spin()
    
