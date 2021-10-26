#!/usr/bin/env python
import rospy
import covlowvel
import threading
import time
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import Int32

flag = 0
height = 0
h_msg = Int32()
servo1 = 10
servo2 = 10
servo3 = 10

def servoCallback(data):
    global servo1, servo2, servo3
    servo1 = int(data.x)
    servo2 = int(data.y)
    servo3 = int(data.z)
    # print("Servo command is %d %d %d" % (servo1, servo2, servo3))

# def pub_thread():
#     pub = rospy.Publisher("/sun/gogogo", Bool, queue_size=10)
#     height_pub = rospy.Publisher("/sun/height", Int32, queue_size=10)
#     rate = rospy.Rate(50) #50HZ
#     while not rospy.is_shutdown():
#         height = covlowvel.getLaser()
#         h_msg.data = height
#         height_pub.publish(h_msg)
#         if(covlowvel.getButton()):
#             _str = True
#             pub.publish(_str)
#         else:
#             _str = False
#             pub.publish(_str)
#         rate.sleep()

# class MyThread(threading.Thread):
#     def __init__(self,name):
#         threading.Thread.__init__(self,name=name)
#         self.pub = rospy.Publisher("/sun/gogogo", Bool, queue_size=10)
#         self.height_pub = rospy.Publisher("/sun/height", Int32, queue_size=10)
#     def run(self):
#         while not rospy.is_shutdown():
#             height = covlowvel.getLaser()
#             covlowvel.setServo(servo1, servo2, servo3)
#             h_msg.data = height
#             self.height_pub.publish(h_msg)
#             if(covlowvel.getButton()):
#                 _str = True
#                 self.pub.publish(_str)
#             else:
#                 _str = False
#                 self.pub.publish(_str)
#             time.sleep(0.02)

def init_node():
    rospy.init_node("Raspberry_Pi", anonymous=True)
    covlowvel.init("/dev/ttyACM1", "dji")
    servo_sub = rospy.Subscriber("/sun/servo_ctl", Vector3, servoCallback)
    pub = rospy.Publisher("/sun/gogogo", Bool, queue_size=10)
    height_pub = rospy.Publisher("/sun/height", Int32, queue_size=10)
    # thread_pub = threading.Thread(target=pub_thread)
    # thread_pub.start()
    # T1=MyThread("thread 1")
    # T1.start()
    # rospy.spin()
    global servo1, servo2, servo3
    rate = rospy.Rate(30) #30HZ
    while not rospy.is_shutdown():
        height = covlowvel.getLaser()
        h_msg.data = height
        height_pub.publish(h_msg)
        if(covlowvel.getButton()):
            _str = True
            pub.publish(_str)
            servo1 = 120
            servo2 = 120
            servo3 = 120
        else:
            _str = False
            pub.publish(_str)
        covlowvel.setServo(servo1, servo2, servo3)
        rate.sleep()



if __name__ ==  '__main__':
    try:
        print("Serial init successful")
        init_node()
    except rospy.ROSInterruptException:
        print("Serial init error!")


    

    
