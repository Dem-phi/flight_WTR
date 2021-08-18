#!/usr/bin/env python
import rospy
import covlowvel
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
lock_angle = 60
flag = 0
def init_node():
    servo_sub = rospy.Subscriber("/sun/servo_ctl", Vector3, servoCallback)
    pub = rospy.Publisher("/sun/gogogo", Bool, queue_size=10)
    rospy.init_node("Raspberry_Pi", anonymous=True)
    rate = rospy.Rate(20) #20HZ
    covlowvel.init("/dev/ttyACM1", "dji")
    while not rospy.is_shutdown():
        if(covlowvel.getButton()):
            _str = True
            pub.publish(_str)
        else:
            _str = False
            pub.publish(_str)
        rate.sleep()


def servoCallback(msg):
    # if msg.x != 0:
    #     covlowvel.setServo(msg.x, lock_angle, lock_angle)
    #     flag = 1
    # elif msg.y != 0:
    #     covlowvel.setServo(lock_angle, msg.y, lock_angle)
    #     flag = 2
    # elif msg.z != 0:
    #     covlowvel.setServo(lock_angle, lock_angle, msg.z)
    #     flag = 3
    # else:
    #     covlowvel.setServo(lock_angle, lock_angle, lock_angle)
    #     return
    # print("Servo %d is releasing" % flag)
    covlowvel.setServo(int(msg.x), int(msg.y), int(msg.z))
    print("Servo command is %d %d %d" % (msg.x, msg.y, msg.z))
    return


if __name__ ==  '__main__':
    try:
        print("Serial init successful")
        init_node()
    except rospy.ROSInterruptException:
        print("Serial init error!")


    

    