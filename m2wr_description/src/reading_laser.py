#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan,NavSatFix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from m2wr_description.msg import finalData,Ack
from math import atan2,fabs,radians,sqrt,pi,exp,pow,acos,sin,cos
from utils import *
from tf.transformations import euler_from_quaternion
from time import sleep,time
from threading import Thread
from operator import itemgetter
from std_msgs.msg import String,Header
from numpy import array,sign,clip,concatenate,full
from sys import maxsize
from os import getenv,system,path
from  subprocess import Popen
from random import choice,randint,random,randrange
from string import ascii_letters
term_pub=None
leader_pub=None
pub=None
pub1=None
pub2=None
own_comm=None
drone_pub=None
grp=None
sub2=None
robot_idx=-1
boundaries=[0,0,0,0]

x_pos=0.0
y_pos=0.0
regions=[-1,-1,-1,-1,-1,-1,-1,-1,-1]
theta=None
s=0 #boundary condition
locations={'r0':[-1,-1],'r1':[-1,-1],'r2':[-1,-1]}
locations_ll={'r0':[-1,-1],'r1':[-1,-1],'r2':[-1,-1]}
num_robot=3
distance=[[0] * num_robot for i in range(num_robot)]
leader=-1
current_robot=None
visited_and_distance=[]
location_other=None
data=list()
loc_pub=[]
status_map=[0]*(num_robot)
send_to_drone=0
last_loc_update=[0]*num_robot

def handle_ack(msg):
    global drone_pub
    global send_to_drone
    if(msg.ping=="ACK") and send_to_drone!=1:
        ack=Ack()
        ack.ping="GOT IT"
        drone_pub.publish(ack)
        send_to_drone=1
        return
        
def comm_channel(msg):
    global locations
    global grp
    global robot_idx
    global stop_r
    global current_robot
    global distance
    global num_robot
    global pub
    global pub2
    global leader
    global drone_pub
    global send_to_drone

    if rospy.has_param("/"+grp+"_leader") and rospy.has_param('/'+grp+'/fail'+str(robot_idx)):
        leader = rospy.get_param("/"+grp+"_leader")
        status_map[robot_idx]=int(rospy.get_param('/'+grp+'/fail'+str(robot_idx))==0)
        
        if(status_map[robot_idx]):
            print_term("My leader is {}".format(leader))
            if msg.loc_data==1:
                print_term("GOT LOC DATA ")
                sender_robot = msg.sender
                locations[sender_robot][0]=msg.x
                locations[sender_robot][1]=msg.y
                last_loc_update[int(msg.sender[1])]=time()
                locations_ll[sender_robot][0]=msg.latitude
                locations_ll[sender_robot][1]=msg.longitude
                update_dis_matrix()
            elif msg.sensor_data==1:
                if(msg.sender==msg.origin):
                    print_term("GOT SENSOR MSG FROM : {}".format(msg.sender))
                    
                else:
                    print_term("GOT SENSOR MSG Forwarded FROM : {} ,originated from : {}".format(msg.sender,msg.origin))
                
                msg.sender=current_robot
                if len(msg.path)==1: #leader itself
                    bs_lat = rospy.get_param("/bs/lat")
                    bs_lon = rospy.get_param("/bs/lon")
                    dis =sqrt((locations[current_robot][0]-bs_lat)**2+(locations[current_robot][1]-bs_lon)**2)
                    if dis<20: 
                        pub2.publish(msg)
                        send_to_drone=0
                    if send_to_drone==1:
                        drone_pub.publish(msg)
                else:
                    msg.path=msg.path[1:]
                    temp_pub = rospy.Publisher('/'+grp+'_r'+str(msg.path[0])+'/comm_channel',finalData,queue_size=1)
                    temp_pub.publish(msg)
                    temp_pub.unregister()

            elif msg.drone==1:
                # if send_to_drone==1:
                #     drone_pub.publish(msg)
                    # ack=Ack()
                    # ack.data=msg
                    # drone_pub.publish(ack)
                if(msg.ping=="ACK") and send_to_drone!=1:
                    msg.ping="GOT IT"
                    drone_pub.publish(msg)
                    send_to_drone=1
                
        # else:
        #     if len(msg.path)==1: #leader itself
        #         rospy.Subscriber('/drone/comm_channel',Ack,handle_ack)
        #         if send_to_drone==1:
        #             ack=Ack()
        #             ack.data=msg
        #             drone_pub.publish(ack)

def update_dis_matrix():
    global locations
    global num_robot
    global distance
    for i in range(num_robot):
        temp=int(rospy.get_param('/'+grp+'/fail'+str(i))==0)
        if not temp:
            distance[robot_idx][i]=maxsize
            distance[i][robot_idx]=maxsize
            status_map[i]=0
            continue
        status_map[i]=1
        for j in range(num_robot):
            if i==j:
                distance[i][j]=0
            else:
                slat = locations['r'+str(i)][0]
                slon = locations['r'+str(i)][1]
                elat = locations['r'+str(j)][0]
                elon = locations['r'+str(j)][1]
                dis=distance_lat_lon(slat,slon,elat,elon)
                if dis>15:
                    dis=maxsize
                distance[i][j]=dis
                distance[j][i]=dis
  
def get_path(node):
    global visited_and_distance
    global leader
    leader = rospy.get_param("/"+grp+"_leader")
    path=[]
    while(node!=leader):
        if(visited_and_distance[node][2]==None):
            return path
        path.append(node)
        node=visited_and_distance[node][2]
    path.append(leader)
    return path

def to_be_visited(num_robot):
    global visited_and_distance
    v = -10
    # Choosing the vertex with the minimum distance
    for index in range(num_robot):
        if visited_and_distance[index][0] == 0 and (v < 0 or visited_and_distance[index][1] <= visited_and_distance[v][1]):
            v = index
    return v

def multihop(num_robot):
    global visited_and_distance
    global current_robot
    global distance
    global robot_idx
    global grp
    global own_comm
    global leader
    global locations
    global loc_pub
    global pub2

    leader = rospy.get_param("/"+grp+"_leader")
    for vertex in range(num_robot):
        to_visit = to_be_visited(num_robot)
        if to_visit<0:
            return
        for neighbor_index in range(num_robot):
            if neighbor_index!=to_visit and visited_and_distance[neighbor_index][0] == 0:
                new_distance = visited_and_distance[to_visit][1] + distance[to_visit][neighbor_index]
                if visited_and_distance[neighbor_index][1] > new_distance:
                    visited_and_distance[neighbor_index][1] = new_distance
                    visited_and_distance[neighbor_index][2]=to_visit
        visited_and_distance[to_visit][0] = 1
    msg=finalData()
    msg.sender=current_robot
    msg.origin=current_robot
    msg.group=grp
    if len(data)!=0:
        msg.data=data
    msg.sensor_data=1
    msg.leader_id=leader
    msg.x=locations[current_robot][0]
    msg.y=locations[current_robot][1]
    msg.latitude=locations_ll[current_robot][0]
    msg.longitude=locations_ll[current_robot][1]
    msg.path=get_path(robot_idx)
    if len(msg.path)==0:
        return
    print_term("Path : {}".format(msg.path))
    h = Header()
    h.stamp = rospy.Time.now() 
    msg.header= h
    own_comm.publish(msg)
    
def initialize(num_robot):
    global visited_and_distance
    global leader
    leader = rospy.get_param("/"+grp+"_leader")
    visited_and_distance=[]

    for i in range(num_robot):
        visited_and_distance.append([0,maxsize,-1])
    
    visited_and_distance[leader][1]=0
    visited_and_distance[leader][2]=leader

def send_sensor_data():
    global num_robot
    global robot_idx,grp
    global visited_and_distance
    global leader
    while rospy.has_param("/"+grp+"_leader"):
        # print_term("sending data")
        initialize(num_robot)
        # print_term("initialized")
        multihop(num_robot)    
        sleep(randint(1,5))

def print_term(msg):
    global term_pub
    global current_robot
    msg = str(msg)
    prefix = ""+grp+current_robot+" : "
    if(term_pub!=None):
        term_pub.publish(prefix+msg)

def newOdom(msg):
    global x_pos
    global y_pos
    global theta
    global robot_idx
    global status_map
    x_pos=msg.pose.pose.position.x
    y_pos=msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    global stop_r
    (r, p, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    if(rospy.has_param('/'+grp+'/fail'+str(robot_idx))):
        status_map[robot_idx]= int(rospy.get_param('/'+grp+'/fail'+str(robot_idx))==0)
        if status_map[robot_idx]!=1:
            status_map[robot_idx]=0
            rospy.set_param('/'+grp+'_failures',rospy.get_param('/'+grp+'_failures')+1)
            pub.publish(Twist())
            stop_r=1
            print_term("Failed")  

def start(msg):
    angle=list(range(91,0,-1))+list(range(91,181))
    maps={}
    for i in angle:
        if round(float(msg.ranges[i]),2)==0:
            value=float(5)
        else:
            value=round(float(msg.ranges[i]),2)
        maps[i]=value
    sorted_map=sorted(maps.items(),key=itemgetter(1),reverse=True)
    max_map={sorted_map[i]:sorted_map[i+1] for i in range(0,len(sorted_map),2)}
    return maps,max_map

def start2(msg,x,y):
    angle=list(range(x,360))+list(range(0,y))
    maps={}
    for i in angle:
        if round(float(msg.ranges[i]),2)==0:
            value=float(5)
        else:
            value=round(float(msg.ranges[i]),2)
        maps[i]=value
    return maps


def direction(maps,max_map):
    dist=0
    alpha=0
    flag=False
    for value in max_map.items():
        mod=0
        while 90+mod<=180 and 90+mod>=0 and flag==False :
            if 90+mod in maps:
                if value>=maps[90+mod]:
                    flag=True
                    if mod<0:
                        mod=abs(mod)
                    else:
                        mod=-mod-1
                else:
                    flag=False
                    break
        if flag==True:
            dist=value[1]
            alpha=value[0]
            break
    return dist,alpha


def clbk_laser(msg):
    global data
    global x_pos,y_pos,boundaries
    regions=[min(min(msg.ranges[0:79]),10),
    min(min(msg.ranges[80:159]),10),
    min(min(msg.ranges[160:239]),10),
    min(min(msg.ranges[240:319]),10),
    min(min(msg.ranges[320:339]),10),
    min(min(msg.ranges[400:479]),10),
    min(min(msg.ranges[480:559]),10),
    min(min(msg.ranges[560:639]),10),
    min(min(msg.ranges[640:719]),10)]
    data=regions
    move=Twist()
    if rospy.has_param('/'+grp+'_stop') and rospy.has_param('/cnt'):
        if(rospy.get_param('/'+grp+'_stop')==1):
            move.linear.x=0.0
            move.angular.z=0.0
            pub.publish(move)
        
        elif rospy.get_param('/cnt')==1:
            maps,max_map=start(msg)
            maps2=start2(msg,181,181)
            dist,alpha=direction(maps,max_map)
            linear=round(0.26-0.26*exp(-1*(dist[1]-0.3)),2)
            angular=sign(alpha[0])*round(1.8-1.8*exp(-0.35*abs(alpha[0]*pow(abs(linear),1.5))),2)
            linear=linear*2
            angular=angular
            if  (x_pos>boundaries[1] or y_pos>boundaries[3] or x_pos<boundaries[0] or y_pos<boundaries[2]):
                d_x = randrange(boundaries[0], boundaries[1], 1)
                d_y = randrange(boundaries[2], boundaries[3], 1)
                inc_x = d_x -x_pos
                inc_y = d_y -y_pos
                angle = atan2(inc_y, inc_x)
                if(theta!=None):
                    if abs(angle - theta) > 0.3:
                        linear = 0.0
                        angular = 0.6
                    else:
                        linear= 0.8
                        angular = 0.0
                move.linear.x=linear
                move.angular.z=angular
                pub.publish(move)
            else:
                move.linear.x=linear
                move.angular.z=angular
                pub.publish(move)
                rospy.sleep(1)
                
            coordinates=[x_pos,y_pos]
            home_dir=path.expanduser('~')
            desktop_dir=path.join(home_dir,'Desktop')
            with open(path.join(desktop_dir,'map'+grp+'_'+current_robot+'.txt'),'a+') as savefile:
                savefile.write(str(x_pos))
                savefile.write(',')
                savefile.write(str(y_pos))
                savefile.write('\n')
                savefile.close()
        elif rospy.get_param('/cnt')==2:
            move.linear.x=0.0
            move.angular.z=0.0
            pub.publish(move)
            coord_x=[]
            coord_y=[]
            home_dir=path.expanduser('~')
            desktop_dir=path.join(home_dir,'Desktop')
            with open(path.join(desktop_dir,'map'+grp+'_'+current_robot+'.txt'),'r') as csvfile:
                plots=reader(csvfile,delimiter=',')
                for row in plots:
                    coord_x.append(float(row[0]))
                    coord_y.append(float(row[1]))
                csvfile.close()
            plt.plot(coord_x,coord_y,label='Loaded from file!')
            plt.xlabel('x')
            plt.ylabel('y')
            plt.legend()
            plt.show()
            rospy.signal_shutdown("We are done here!")
  
def broadcast_loc(msg1):
    global loc_pub
    global locations
    global robot_idx
    global x_pos
    global y_pos
    global status_map
    if(rospy.has_param('/'+grp+'/fail'+str(robot_idx))):
        status_map[robot_idx]=int(rospy.get_param('/'+grp+'/fail'+str(robot_idx))==0)
        if status_map[robot_idx]!=1:
            return
        locations[current_robot][0]=x_pos
        locations[current_robot][1]=y_pos
        locations_ll[current_robot][0]=msg1.latitude
        locations_ll[current_robot][1]=msg1.longitude
        for publisher in loc_pub:
            msg_t = finalData()
            msg_t.sender=current_robot
            msg_t.group=grp
            msg_t.latitude=msg1.latitude
            msg_t.longitude=msg1.longitude
            msg_t.x = x_pos
            msg_t.y = y_pos
            msg_t.loc_data=1
            now = rospy.get_rostime()
            msg_t.header.stamp = now
            publisher.publish(msg_t)
        #time.sleep(2+int(10*random()))
        

def main():
    global pub
    global pub1
    global pub2
    global sub2
    global centers
    global boundaries
    global grp
    global current_robot
    global robot_idx
    global num_robot
    global visited_and_distance
    global distance
    global election_pub
    global loc_pub
    global status_map
    global term_pub
    global drone_pub
    global own_comm
    global leader
    rospy.init_node('reading_laser')
    current_robot = rospy.get_param('~robot').split('_')[1]
    grp=rospy.get_param('~robot').split('_')[0]
    robot_idx  =  int(current_robot.split('r')[1])
    initialize(num_robot)
    boundary = rospy.get_param('~boundary')
    boundary = boundary.split(',')
    boundaries[0]=float(boundary[0])
    boundaries[1]=float(boundary[1])
    boundaries[2]=float(boundary[2])
    boundaries[3]=float(boundary[3])
    pub=cmd_vel_pubs[grp][robot_idx]
    sub2=rospy.Subscriber('odom',Odometry,newOdom)
    sub1=rospy.Subscriber('laser',LaserScan,clbk_laser)
    sub3=rospy.Subscriber('fix',NavSatFix,broadcast_loc)
    main_sub = rospy.Subscriber('comm_channel',finalData,comm_channel)
    pub2=rospy.Publisher('/send_gs',finalData,queue_size=5)
    term_pub = rospy.Publisher("/"+grp+"_r"+str(robot_idx)+'/terminal',String,queue_size=10)
    drone_pub = rospy.Publisher('/drone/comm_channel',finalData,queue_size=5)
    own_comm = rospy.Publisher('/'+grp+'_r'+str(robot_idx)+'/comm_channel',finalData,queue_size=5)
    status_map[robot_idx]=1
    for i in range (0,num_robot):
        if i != robot_idx:
            t = rospy.Publisher('/'+grp+'_r'+str(i)+'/comm_channel',finalData,queue_size=5)
            loc_pub.append(t)
    if(rospy.has_param('/'+grp+'_leader')):
        leader = rospy.get_param('/'+grp+'_leader')
    x = Thread(target=send_sensor_data)
    x.start()
    rospy.spin()
if __name__=='__main__':
    main()
