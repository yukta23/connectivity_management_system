#!/usr/bin/env python  
import roslib
roslib.load_manifest('sjtu_drone')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from sensor_msgs.msg import LaserScan
import time

lasr1 = LaserScan()
lasr2 = LaserScan()
def divide_lsr_data(msg):
    
    regions = [
        min(100,min(msg.ranges[0:71])),
        min(100,min(msg.ranges[72:143])),
        min(100,min(msg.ranges[144:215])),
        min(100,min(msg.ranges[216:287])),
        min(100,min(msg.ranges[288:359])),
        min(100,min(msg.ranges[360:431])),
        min(100,min(msg.ranges[432:503])),
        min(100,min(msg.ranges[504:575])),
        min(100,min(msg.ranges[576:647])),
        min(100,min(msg.ranges[648:719])),
    ] 
    print regions

def clbk_laser1(msg):
    # 720 / 5 = 144
    global lasr1
    lasr1 = msg

def clbk_laser2(msg):
    # 720 / 5 = 144
    global lasr2
    lasr2 = msg 

if __name__ == '__main__':
    i=0
    rospy.init_node('robot_tf_node_listen')
    node_listen = tf.TransformListener()
    drone_controller = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    laser_pub = rospy.Publisher('/drone/laser_bot',LaserScan ,queue_size=5)
    laser_sub1 = rospy.Subscriber('/robot2/m2wr/laser/scan', LaserScan, clbk_laser1)
    laser_sub2 = rospy.Subscriber('/robot3/m2wr/laser/scan', LaserScan, clbk_laser2)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (data_t,data_r) = node_listen.lookupTransform('/gs', '/robot2', rospy.Time(0))
            (data_t2,data_r2) = node_listen.lookupTransform('/drone', '/robot2', rospy.Time(0))
            (data_t3,data_r3) = node_listen.lookupTransform('/gs', '/robot3', rospy.Time(0))
            (data_t4,data_r4) = node_listen.lookupTransform('/drone', '/robot3', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        velocity_msg = geometry_msgs.msg.Twist()
        angular_d_r2=4 * math.atan2(data_t2[1], data_t2[0])
        linear_d_r2=0.5 * math.sqrt(data_t2[0] ** 2 + data_t2[1] ** 2)

        angular_d_r3=4 * math.atan2(data_t4[1], data_t4[0])
        linear_d_r3=0.5 * math.sqrt(data_t4[0] ** 2 + data_t4[1] ** 2)

        linear_g_r2= 0.5 * math.sqrt(data_t[0] ** 2 + data_t[1] ** 2)

        linear_g_r3= 0.5 * math.sqrt(data_t3[0] ** 2 + data_t3[1] ** 2)
        
        if linear_g_r2< 10 and linear_g_r3 < 10:
            print "[None] Both Robots in Range of Ground Station"
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = 0
            drone_controller.publish(velocity_msg)

        elif linear_g_r2< 10 and linear_g_r3 > 10:
            while linear_d_r3 >5:
                print "[Single Mode] Drone following Robot3"
                (t,r) = node_listen.lookupTransform('/drone', '/robot3', rospy.Time(0))
                angular_d_r3=4 * math.atan2(t[1], t[0])
                linear_d_r3=0.5 * math.sqrt(t[0] ** 2 + t[1] ** 2)
                velocity_msg.linear.x = 10*linear_d_r3
                velocity_msg.angular.z = angular_d_r3
                drone_controller.publish(velocity_msg)
            velocity_msg.linear.x=0
            velocity_msg.angular.z=0
            drone_controller.publish(velocity_msg)
            divide_lsr_data(lasr2)
            laser_pub.publish(lasr2)
            

        elif linear_g_r2> 10 and linear_g_r3 < 10:
            while linear_d_r2 >5:
                print "[Single Mode] Drone following Robot2"
                (t,r) = node_listen.lookupTransform('/drone', '/robot2', rospy.Time(0))
                angular_d_r2=4 * math.atan2(t[1], t[0])
                linear_d_r2=0.5 * math.sqrt(t[0] ** 2 + t[1] ** 2)
                velocity_msg.linear.x = 10*linear_d_r2
                velocity_msg.angular.z = angular_d_r2
                drone_controller.publish(velocity_msg)
            velocity_msg.linear.x=0
            velocity_msg.angular.z=0
            drone_controller.publish(velocity_msg)
            divide_lsr_data(lasr1)
            laser_pub.publish(lasr1)
            

        else :
            if i==0:
                while linear_d_r2 >5 and linear_g_r2 > 10:
                    print "[Alternate Mode] Drone following Robot2"
                    (t,r) = node_listen.lookupTransform('/drone', '/robot2', rospy.Time(0))
                    (t2,r2) = node_listen.lookupTransform('/gs', '/robot2', rospy.Time(0))
                    angular_d_r2=4 * math.atan2(t[1], t[0])
                    linear_d_r2=0.5 * math.sqrt(t[0] ** 2 + t[1] ** 2)
                    linear_g_r2=0.5 * math.sqrt(t2[0] ** 2 + t2[1] ** 2)
                    velocity_msg.linear.x = 10*linear_d_r2
                    velocity_msg.angular.z = angular_d_r2
                    drone_controller.publish(velocity_msg)
                velocity_msg.linear.x=0
                velocity_msg.angular.z=0
                drone_controller.publish(velocity_msg)
                i=1
                t = rospy.Time.from_sec(time.time())
                seconds = t.to_sec()
                diff=0
                while True:
                    t = rospy.Time.from_sec(time.time())
                    seconds2 = t.to_sec()
                    diff= seconds2-seconds
                    laser_pub.publish(lasr1)
                    divide_lsr_data(lasr1)
                    if diff > 10 :
                        break

            elif i==1:
                while linear_d_r3 >5 and linear_g_r3 >10:
                    print "[Alternate Mode] Drone following Robot3"
                    (t,r) = node_listen.lookupTransform('/drone', '/robot3', rospy.Time(0))
                    (t2,r2) = node_listen.lookupTransform('/gs', '/robot3', rospy.Time(0))
                    angular_d_r3=4 * math.atan2(t[1], t[0])
                    linear_d_r3=0.5 * math.sqrt(t[0] ** 2 + t[1] ** 2)
                    linear_g_r3=0.5 * math.sqrt(t2[0] ** 2 + t2[1] ** 2)
                    velocity_msg.linear.x = 10*linear_d_r3
                    velocity_msg.angular.z = angular_d_r3
                    drone_controller.publish(velocity_msg)
                velocity_msg.linear.x=0
                velocity_msg.angular.z=0
                drone_controller.publish(velocity_msg)
                i=0
                t = rospy.Time.from_sec(time.time())
                seconds = t.to_sec()
                diff=0
                while True:
                    t = rospy.Time.from_sec(time.time())
                    seconds2 = t.to_sec()
                    diff= seconds2-seconds
                    laser_pub.publish(lasr2)
                    divide_lsr_data(lasr2)
                    if diff > 10 :
                        break

        # rospy.loginfo(velocity_msg)
        rate.sleep()