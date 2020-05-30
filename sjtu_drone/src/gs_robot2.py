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
def make_laser_blocks(msg):
    # 720 / 5 = 144
    global regions
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
    
def read_laser_from_drone(msg):
    global regions_drone
    regions_drone = [
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

if __name__ == '__main__':
    i=0
    rospy.init_node('gs_robot2')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    sub = rospy.Subscriber('/robot2/m2wr/laser/scan', LaserScan, make_laser_blocks)
    lsr_data = rospy.Subscriber('/drone/laser_bot', LaserScan, read_laser_from_drone)
    f=open("robot2.csv","w")
    rate = rospy.Rate(10.0)
    try:
        f=open("robot2.csv","w")
        while not rospy.is_shutdown():
            try:
                (pos_t,rot) = listener.lookupTransform('/gs', '/robot2', rospy.Time(0))
                (pos_t2,rot2) = listener.lookupTransform('/drone','/robot2',rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            linear = 0.5 * math.sqrt(pos_t[0] ** 2 + pos_t[1] ** 2)
            angular= 4 * math.atan2(pos_t[1], pos_t[0])
            linear_d = 0.5 * math.sqrt(pos_t2[0] ** 2 + pos_t2[1] ** 2)
            t =  0
            if linear > 10:
                v = rospy.get_param('/drone_status')
                
                if v == 0:
                    print "[R2] Connectivity about to loose. Deploying Drone !"
                    rospy.set_param('/drone_status',1)
                    cmd = os.popen('rostopic pub /drone/takeoff std_msgs/Empty "{}"')
                elif t==0 and linear_d>=5:
                    print "[R2] Drone Deployed. Waiting for Connection to re-establish... "
                    t=1
                elif linear_d < 5 and t==1:
                    print "[R2] Connection Re-established !"
                    t=2
                else :
                    
                    print "[R2] Through Drone"
                    print regions_drone
                    f.write("Drone,")
                    f.write(" ".join(map(str, regions_drone)))
                    f.write("\n")
            else:
                s=""
                print "[R2] In proximity"
                print regions
                f.write("Proximity,")
                f.write(" ".join(map(str, regions)))
                f.write("\n")
    except KeyboardInterrupt:
            f.close()
