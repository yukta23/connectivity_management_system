#!/usr/bin/env python  
import roslib
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String,Empty
from m2wr_description.msg import finalData,Ack
from os import system,getenv,popen
from subprocess import Popen,call
from math import atan2,sqrt
from utils import *
from geometry_msgs.msg import Pose,Twist
from tf.transformations import euler_from_quaternion
from random import random
from threading import Thread
from time import time
import rosgraph
x=None
y=None
z_pos=None
yaw=None
num_robot=3
failed_grp=[0,0]

def comm_channel(msg):
    global gs_pub
    if(msg.ping=="GOT IT"):
        rospy.set_param('/sender_acknowledgement'+grp,1)
        print_term("---------------------------Reply from ground robot------------------------")
        print_term(msg.data)
    print_term(msg.data.group)
    gs_pub.publish(msg.data)
        
def print_term(msg):
    global term_pub   
    msg = str(msg)
    prefix = "DRONE : "
    if(term_pub!=None):
        term_pub.publish(prefix+msg)

def handle_Pose(msg): 
    global x
    global y
    global z_pos
    global yaw
    x=msg.position.x
    y=msg.position.y
    z=msg.position.z
    orientation_q=msg.orientation
    orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (r,p,yaw)=euler_from_quaternion (orientation_list)
   
def path_plan(goal,dist):
    global x
    global y
    global yaw
    global pub
    global bs_lat
    global bs_lon
    print_term(goal)
    inc_x=goal[0]-x
    inc_y=goal[1]-y
    dis = sqrt(inc_x**2+inc_y**2)
    print_term("IN path plan DIS : {}".format(dis))
    speed=Twist()
    while(dis>dist):
        inc_x=goal[0]-x
        inc_y=goal[1]-y
        dis = sqrt(inc_x**2+inc_y**2)
        angle=atan2(inc_y,inc_x)
        if abs(angle - yaw) > 0.3 :
            speed.linear.x = 0.0
            speed.angular.z = 0.5
        else:
            speed.linear.x = 1.0
            speed.angular.z = 0.0
        pub.publish(speed)
    speed=Twist()
    pub.publish(speed)
    return

def check_leader(matrix):
    global bs_lat
    global bs_lon
    global num_robot
    global grp
    global failed_grp
    grp = matrix[0]
    locations=matrix[1]
    leader_idx = rospy.get_param("/"+grp+"_leader")
    path_plan(locations["r"+str(leader_idx)],5)
    #############drone reached over leader ,ping the leader wait for response
    # msg=Ack()
    msg=finalData()
    msg.ping="ACK"
    msg.drone=1
    print_term("Pinging -------------")
    leader_pub = rospy.Publisher('/'+grp+'_r'+str(leader_idx)+'/comm_channel',finalData,queue_size=10)
    leader_pub.publish(msg)
    for i in range(0,3):
        init=time()
        end=time()+1
        while(end-init<5 and not rospy.has_param('/sender_acknowledgement'+grp)):
            if(rospy.has_param('/sender_acknowledgement'+grp)):
                rospy.delete_param('/sender_acknowledgement'+grp)
                print_term("Leader is ok")
                return
            end=time()
    
    master = rosgraph.Master('/mynode')
    try:
        master.lookupNode('/'+grp+'_r'+str(rospy.get_param('/'+grp+'_leader'))+'/'+grp+'_r'+str(rospy.get_param('/'+grp+'_leader')))
    except (rosgraph.masterapi.MasterError):
        print_term("I think leader is damaged")
        bs_distance=[0]*num_robot
        mini = 9999999
        flag=False
        for i in range(num_robot):
            if i!=leader_idx and rospy.get_param("/"+grp+"/fail"+str(i))==0:
                try:
                    master.lookupNode('/'+grp+'_r'+str(i)+'/'+grp+'_r'+str(i))
                    r = "r"+str(i)
                    bs_distance[i] = distance_lat_lon(locations[r][0],locations[r][1],bs_lat,bs_lon)
                    flag=True
                    if mini>bs_distance[i]:
                        l_idx=i
                        mini=bs_distance[i]
                except (rosgraph.masterapi.MasterError):
                    continue
        if flag:            
            print_term("New Leader is set at {}".format(l_idx))
            rospy.set_param("/"+grp+"_leader",l_idx)
        else:
            failed_grp[int(grp[3])-1]=1
            rospy.set_param('/'+grp+'_fail',1)
            print_term("No robot left in the cluster")
            return

def sdf(e):
    grp=e[0]
    matrix=e[1]
    head=rospy.get_param('/'+grp+'_leader')
    bs_lat = rospy.get_param("/bs/lat")
    bs_lon = rospy.get_param("/bs/lon")
    dis =sqrt((matrix['r'+str(head)][0]-bs_lat)**2+(matrix['r'+str(head)][1]-bs_lon)**2)
    return dis 
    

def handle_queue():
    global z_pos
    global failed_grp
    while True:
        q = rospy.get_param("/queue1")
        if(len(q)!=0):
            # q.sort(key=sdf)
            to_serve = q[0]
            if failed_grp[int(to_serve[0][3])-1]:
                temp=rospy.get_param("/queue1",q)
                rospy.set_param("/queue1",temp[1:])
                continue
            print_term(to_serve)
            if(rospy.get_param("/takeoff")==0):
                take_off_pub.publish(Empty())
                rospy.set_param("/takeoff",1)
                print_term("Taking off")
            check_leader(to_serve)
            print_term("queue: "+str(q))
            temp=rospy.get_param("/queue1",q)
            rospy.set_param("/queue1",temp[1:])

if __name__ == '__main__':
    rospy.init_node('drone')
    rospy.set_param("/takeoff",0)
    bs_lat = rospy.get_param("/bs/lat")
    bs_lon = rospy.get_param("/bs/lon")
    term_pub = rospy.Publisher('/drone/terminal',String,queue_size=10)
    gs_pub = rospy.Publisher('/send_gs',finalData,queue_size=10)
    rospy.Subscriber('/drone/gt_pose',Pose,handle_Pose)
    pub=rospy.Publisher('/cmd_vel', Twist,queue_size=5)
    take_off_pub = rospy.Publisher('/drone/takeoff', Empty,queue_size=1)
    land_pub = rospy.Publisher('/drone/land', Empty,queue_size=1)
    rospy.Subscriber('/drone/comm_channel',finalData,comm_channel)
    # drone_pub=rospy.Publisher('/drone/comm_channel',Ack,queue_size=10)
    print_term("running")
    handle_q = Thread(target=handle_queue)
    handle_q.start()
    rospy.spin()
