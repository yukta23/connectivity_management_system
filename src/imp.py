#!/usr/bin/env python  
import subprocess
import rospy
if __name__ == '__main__':
    rospy.init_node("terminals")
    subprocess.call(['gnome-terminal','-e','rostopic echo /gs/terminal'])
    subprocess.call(['gnome-terminal','-e','rostopic echo /drone/terminal'])
    subprocess.call(['gnome-terminal','-e','rostopic echo /grp1_r0/terminal'])
    subprocess.call(['gnome-terminal','-e','rostopic echo /grp2_r0/terminal'])
    #subprocess.call(['gnome-terminal','-e','rostopic echo /grp1_r1/terminal'])
    subprocess.call(['gnome-terminal','-e','rostopic echo /grp1_r2/terminal'])
    subprocess.call(['gnome-terminal','-e','rostopic echo /grp2_r1/terminal'])
    #subprocess.call(['gnome-terminal','-e','rostopic echo /grp2_r2/terminal'])
   
