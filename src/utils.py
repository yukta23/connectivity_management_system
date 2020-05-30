#!/usr/bin/env python
from math import sqrt
from geometry_msgs.msg import Twist
import rospy
import random
import string

lat0 = 49.8999326533
lon0 = 8.89988639629

cmd_vel_pubs={'grp1':[rospy.Publisher('/grp1_r0/cmd_vel_robot',Twist,queue_size=1),rospy.Publisher('/grp1_r1/cmd_vel_robot',Twist,queue_size=1),rospy.Publisher('/grp1_r2/cmd_vel_robot',Twist,queue_size=1)],'grp2':[rospy.Publisher('/grp2_r0/cmd_vel_robot',Twist,queue_size=1),rospy.Publisher('/grp2_r1/cmd_vel_robot',Twist,queue_size=1),rospy.Publisher('/grp2_r2/cmd_vel_robot',Twist,queue_size=1)]}

def latlon2xy(lat,lon):
    x = (lon - lon0)*71846.89402644958
    y = (lat - lat0)*111227.30432309427
    return x,y


def distance_lat_lon(slat,slon,elat,elon):
    fact = 1
    return sqrt((slat*fact-elat*fact)**2+(slon*fact-elon*fact)**2)


def get_rand_str(l):
    lettersAndDigits = string.ascii_letters + string.digits + string.punctuation
    return ''.join((random.choice(lettersAndDigits) for i in range(l)))
