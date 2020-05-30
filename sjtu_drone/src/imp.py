#!/usr/bin/env python  
import subprocess
if __name__ == '__main__':
    subprocess.call(['gnome-terminal','-e','rosrun sjtu_drone multiple_listener.py'])
    subprocess.call(['gnome-terminal','-e','rosrun sjtu_drone gs_robot2.py'])
    subprocess.call(['gnome-terminal','-e','rosrun sjtu_drone gs_robot3.py'])
    subprocess.call(['gnome-terminal','-e','rosrun key_teleop key_teleop2.py key_vel2:=/robot3/cmd_vel_robot'])
    subprocess.call(['gnome-terminal','-e','rosrun key_teleop key_teleop.py key_vel:=/robot2/cmd_vel_robot'])