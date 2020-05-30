#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from m2wr_description.msg import finalData
from os import system,getenv
from subprocess import Popen
from threading import Timer
import rosgraph
timer_grp=[0,0]
queue1_lock=[0,0]
num_robot=3
num_cluster=2
loc_matrix={'grp1':{'r0':[-1,-1],'r1':[-1,-1],'r2':[-1,-1]},'grp2':{'r0':[-1,-1],'r1':[-1,-1],'r2':[-1,-1]}}
grp=""
fst_msg_rcvd=0

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def detect_comm_loss():
    global timer_grp
    global fst_msg_rcvd
    global loc_matrix
    global leader_changed
    global fst_msg_rcvd
    if grp !="":
        for i in range(1,3):
            if timer_grp[i-1]==0 and fst_msg_rcvd and not rospy.get_param('/grp'+str(i)+'_fail'):
                print_term("####################CLUSTER " +str(i)+" comm lost")
                rospy.set_param('/grp'+str(i)+'_stop',1)
                q = rospy.get_param("/queue1")
                q.append(list(['grp'+str(i),loc_matrix['grp'+str(i)]]))
                rospy.set_param("/queue1",q)
        timer_grp=[0,0]
        print_term(str(rospy.get_param('/queue1')))

def print_term(msg):
    global term_pub
    msg = str(msg)
    prefix = "GS : "
    term_pub.publish(prefix+msg)

def init_params():
    rospy.set_param('/base_loc',{'lat':49.8999295935,'lon':8.89989085878})
    rospy.set_param('/grp1_failures',0)
    rospy.set_param('/grp2_failures',0)
    rospy.set_param('/grp1_fail',0)
    rospy.set_param('/grp2_fail',0)
    rospy.set_param('/grp1_leader',0)
    rospy.set_param('/grp2_leader',0)
    rospy.set_param('/cnt',1)
    rospy.set_param('/grp1_stop',0)
    rospy.set_param('/grp2_stop',0)
    rospy.set_param('/grp1',{'fail0':0,'fail1':0,'fail2':0})
    rospy.set_param('/grp2',{'fail0':0,'fail1':0,'fail2':0})
    rospy.set_param('/queue1',[])
    rospy.set_param('/bs',{'lat':10,'lon':10})
  
def sendCallback(msg):
    global timer_grp
    global loc_matrix
    global grp
    global fst_msg_rcvd

    fst_msg_rcvd=1
    grp = msg.group #grp1
    rospy.set_param('/'+grp+'_stop',0)
    if grp!='':
        timer_grp[(int)(grp[3])-1]=1
        loc_matrix[grp][msg.origin][0]=msg.x
        loc_matrix[grp][msg.origin][1]=msg.y
        q=rospy.get_param("/queue1")
        for ele in q:
            if ele[0]==grp:
                q.remove(ele)
        rospy.set_param("/queue1",q)

    print_term("Message From Group : {}".format(msg.group))
    print_term("Sender : {}".format(msg.sender))
    print_term("Origin : {}".format(msg.origin))
    print_term("Data : {}".format(msg.data))
    print_term("Lat : {}".format(msg.latitude))
    print_term("Lon : {}".format(msg.longitude))

if __name__=='__main__':
    rospy.init_node('gsNode')
    rospy.loginfo("working")
    init_params()
    term_pub = rospy.Publisher('/gs/terminal',String,queue_size=10)
    rt = RepeatedTimer(10, detect_comm_loss)
    sub=rospy.Subscriber('/send_gs',finalData,sendCallback)
    rospy.spin()
