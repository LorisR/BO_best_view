#!/usr/bin/env python3


import rospy
import sys
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
global fatt_1

fatt_1=[]

class Pass(object):
        
    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        global fatt_1
        fatt_1=data.data
        fatt_1=np.asarray(fatt_1,dtype="float32")
        fatt_1 = np.reshape(fatt_1, (-1,6))

    def listener(self):

        rospy.init_node('Listener', anonymous=True)
    
        sub=rospy.Subscriber("sphere_values_normals", Float32MultiArray,self.callback)
        rospy.sleep(2)
        sub.unregister()

    
if __name__ == '__main__':
    
    hiv=Pass()
    hiv.listener()
    
        
    
   
       
       
       
    
   
   

