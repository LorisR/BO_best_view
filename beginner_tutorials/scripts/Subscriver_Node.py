#!/usr/bin/env python
 #!/usr/bin/env python
import rospy
from std_msgs.msg import String
from beginner_tutorials.msg import prova_msg

global var
var = "vero"
def callback(prova_msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard a message")
    rospy.sleep(.05)
    print(prova_msg.consenso)
    global var 
    var = "falso"
    
def listener():
    global var
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.


    while(var == "vero"and not rospy.is_shutdown()):

        rospy.Subscriber("chatter", prova_msg, callback)
        
        rospy.sleep(1)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    print("sono pronto a ricevere messagggi, entro nella callback \n")
    listener()
    print("sono nel main")