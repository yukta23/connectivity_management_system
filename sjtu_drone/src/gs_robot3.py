#!/usr/bin/env python  
import roslib
roslib.load_manifest('sjtu_drone')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import os 
from sensor_msgs.msg import LaserScan

regions = [] 
regions_drone=[]
def clbk_laser(msg):
    # 720 / 5 = 144
    global regions
    regions = [
        min(min(msg.ranges[0:143]), 100),
        min(min(msg.ranges[144:287]), 100),
        min(min(msg.ranges[288:431]), 100),
        min(min(msg.ranges[432:575]), 100),
        min(min(msg.ranges[576:713]), 100),
    ] 
    
def read_laser_from_drone(msg):
    global regions_drone
    regions_drone = [
        min(min(msg.ranges[0:143]), 100),
        min(min(msg.ranges[144:287]), 100),
        min(min(msg.ranges[288:431]), 100),
        min(min(msg.ranges[432:575]), 100),
        min(min(msg.ranges[576:713]), 100),
    ] 

if __name__ == '__main__':
    i=0
    rospy.init_node('gs_robot3')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    sub = rospy.Subscriber('/robot3/m2wr/laser/scan', LaserScan, clbk_laser)
    lsr_data = rospy.Subscriber('/drone/laser_bot', LaserScan, read_laser_from_drone)
    rate = rospy.Rate(10.0)
    f=open("robot3.csv","w")
    try:
        f=open("robot3.csv","w")
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/gs', '/robot3', rospy.Time(0))
                (trans2,rot2) = listener.lookupTransform('/drone','/robot3',rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            angular= 4 * math.atan2(trans[1], trans[0])
            linear_d = 0.5 * math.sqrt(trans2[0] ** 2 + trans2[1] ** 2)
            t=0
            if linear > 10:
                v = rospy.get_param('/drone_status')
                
                if v == 0:
                    print "[R3] Connectivity about to loose. Deploying Drone !"
                    rospy.set_param('/drone_status',1)
                    cmd = os.popen('rostopic pub /drone/takeoff std_msgs/Empty "{}"')
                elif  t==0 and linear_d >=5:
                    print "[R3] Drone already Deployed. Waiting for Connection to re-establish... "
                    t=1
                elif linear_d < 5 and t==1:
                    print "[R3] Connection Re-established !"
                    t=2
                else :
                    s=""
                    print "[R3] Through Drone"
                    print regions_drone
                    f.write("Drone,")
                    f.write(" ".join(map(str, regions_drone)))
                    f.write("\n")
            else:
                s=""
                print "[R3] In proximity"
                print regions
                f.write("Proximity,")
                f.write(" ".join(map(str, regions)))
                f.write("\n")
    except KeyboardInterrupt:
            f.close()
