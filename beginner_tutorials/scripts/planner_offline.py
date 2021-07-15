#!/home/loris/anaconda3/envs/py2/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from math import pi
from std_msgs.msg import Float64MultiArray
from moveit_commander.conversions import pose_to_list
from beginner_tutorials.msg import coord_msg_checked
from beginner_tutorials.msg import o3d_coord_msg
import numpy as np
from scipy.spatial.transform import Rotation as R


global s
 


def all_close(goal, actual, tolerance):
  """
  Function for testing if a list of values are within a tolerance range
  @returns: bool
  """
  #all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
     return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
     return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def fromSENSORtoEEF(pos,quat):
    
    r = R.from_quat(quat)
    rot_mat = np.matrix(r.as_dcm(), 'float')
    hom = np.matrix(([0,0,0, 1]), 'float')
    pos1 = np.hstack((rot_mat, pos))
    pose = np.vstack((pos1, hom))
    
    R1 = np.matrix(([1,0,0,-0.02],
                    [0, 1, 0, 0],
                    [0,0,1,0.05285],
                    [0,0,0,1]), 'float')
    R1_inv = np.linalg.inv(R1)
    
    plan_matrix = pose*R1_inv
    
    r = R.from_dcm(plan_matrix[:3,:3])
    quat2 = np.array(r.as_quat(), 'float')
    pos2 = np.array((plan_matrix[0,3],plan_matrix[1,3],plan_matrix[2,3] ), 'float')
    
    return pos2, quat2, plan_matrix


class MoveGroupPythonIntefaceTutorial(object):
  coordinates_new = [0,0,0,0,0,0,0]
  coordinates=[0,0,0,0,0,0,0] #class variable to insert both joint and cartesian coordinates
  i = 0
  
  def __init__(self):
    
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)#inizializza il nodo move_group_...
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print "-- Planning frame: %s --" % planning_frame
    eef_link = move_group.get_end_effector_link()
    print "-- End effector link: %s --" % eef_link
    group_names = robot.get_group_names()
    print "-- Available Planning Groups:", robot.get_group_names()
    print "-- Printing robot state"
    print robot.get_current_state()
    print ""

    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.types =''
    self.types_new =''
    self.pianificato = []

  def plan_to_pose_goal(self,pos_vett,quat):

    move_group = self.move_group
    print('sono in plan to pose goal')

    pose_goal = geometry_msgs.msg.Pose()
    
    
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = pos_vett[0]
    pose_goal.position.y = pos_vett[1]
    pose_goal.position.z = pos_vett[2]
    
    # pose_goal.orientation.x = 1
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = 0
    # pose_goal.position.x = 0.5
    # pose_goal.position.y = 0
    # pose_goal.position.z = 0.5

    move_group.set_pose_target(pose_goal)

    self.pianificato = move_group.plan()
    move_group.stop()

    global s 
    s = self.pianificato[-1]
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.1), self.pianificato


def main():
  try:
    
    tutorial = MoveGroupPythonIntefaceTutorial()
    #tutorial.types = ''
    pub = rospy.Publisher('/coord_msg_checked', coord_msg_checked, queue_size=20)
    msg_to_o3d = coord_msg_checked()
    
    #rospy.init_node('VERIFICATION', anonymous=True)

    while not rospy.is_shutdown():

        msg_sphere = rospy.wait_for_message("/o3d_coord_msg", o3d_coord_msg)
        print("\n ------------------------------------------\n\n ho ricevuto il messaggio da sphere generator")
        print(msg_sphere)
        print(type(msg_sphere.x))
        pos_vett = np.array([msg_sphere.x,msg_sphere.y,msg_sphere.z],'float64')
        #pos_vett_trasp = np.matrix(([1],[1],[1]))
        #quat = np.array((0,0,0,1))
        pos_vett_trasp = np.matrix(([msg_sphere.x],[msg_sphere.y],[msg_sphere.z]))
        rot_matrix = np.matrix([[msg_sphere.a1,msg_sphere.a2,msg_sphere.a3],[msg_sphere.a4,msg_sphere.a5,msg_sphere.a6],[msg_sphere.a7,msg_sphere.a8,msg_sphere.a9]],'float64')
        r = R.from_dcm(rot_matrix)
        quat = r.as_quat()
        print(quat)
        pos2, quat2, plan_matrix_2 = fromSENSORtoEEF(pos_vett_trasp,quat)
        tutorial.plan = tutorial.plan_to_pose_goal(pos2,quat2)
        if s.val == -1:
            print(s.val) 
            
            print('\nERRORE IN PIANIFICAZIONE') 
        else:
            print(s.val) 
            print('\nHO PIANIFICATO')
        msg_to_o3d.x = pos_vett[0]
        msg_to_o3d.y = pos_vett[1]
        msg_to_o3d.z = pos_vett[2]
        msg_to_o3d.qx = quat[0]
        msg_to_o3d.qy = quat[1]
        msg_to_o3d.qz = quat[2]
        msg_to_o3d.qw = quat[3]
        msg_to_o3d.check = s.val
        rospy.loginfo(msg_to_o3d)
        pub.publish(msg_to_o3d)

          
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    main()

