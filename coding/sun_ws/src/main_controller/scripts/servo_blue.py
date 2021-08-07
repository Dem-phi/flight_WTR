#!/usr/bin/env python
# -*- coding: utf-8 -*-
import bluetooth
import rospy
from time import sleep
from geometry_msgs.msg import Vector3

def servo_ctl_Callback(msg):
    if(msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0):
        s.close()
        return
    string = str(int(msg.x))+','+str(int(msg.y))+','+str(int(msg.z))+';'
    s.send(string)
    sleep(0.5)
    print(s.recv(20))

rospy.init_node("servo_blue", anonymous=True)
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect(('98:D3:91:FD:9E:49', 1))
print("Connect successfully!")
rospy.Subscriber("/sun/servo", Vector3, servo_ctl_Callback)
rospy.spin()