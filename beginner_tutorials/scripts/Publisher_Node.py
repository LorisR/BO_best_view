#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from beginner_tutorials.msg import prova_msg

def talker():
    pub = rospy.Publisher('chatter', prova_msg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    msg_da_pubb = prova_msg()
    while not rospy.is_shutdown():
        msg_da_pubb.consenso = "ciao sono Loris"
        msg_da_pubb.x = 6.45893
        msg_da_pubb.y = 5
        msg_da_pubb.z = 2.463
        rospy.loginfo("\n messaggio pubblicato \n")
        pub.publish(msg_da_pubb)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
