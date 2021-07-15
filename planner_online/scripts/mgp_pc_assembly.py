#!/home/marco/anaconda3/envs/py27/bin/python

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
global s
import numpy as np
from planner_online.msg import start_sensor
from planner_online.msg import stop_sensor
global s
from planner_online.msg import coord_to_reach
from std_msgs.msg import String
import tf


#CHANGE ROOT
root = '/home/marco'

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


class MoveGroupPythonIntefaceTutorial(object):
  coordinates_new = [0,0,0,0,0,0,0]
  coordinates=[0,0,0,0,0,0,0] #class variable to insert both joint and cartesian coordinates
  i = 0
  
  def __init__(self):
    
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)#inizializza il nodo wrapper
   #inizializza il nodo move_group_...

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
   # print "-- Planning frame: %s --" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print "-- End effector link: %s --" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
   # print "-- Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print "-- Printing robot state"
    # print robot.get_current_state()
    # print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
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
    self.pianificato1 = []
    
  def OctoMap_Creation(self):
    
    move_group = self.move_group
    
    # positions = np.matrix(([0,0,0,-pi/6,0,pi/6,pi/4],
    #                        [-pi/2,0,0,-pi/6,0,pi/6,pi/4],
    #                        [pi/2,0,0,-pi/6,0,pi/6,pi/4],
    #                        [pi/2,0,0,-pi/6,0,pi/3,pi/4],
    #                        [-pi/2,0,0,-pi/6,0,pi/3,pi/4],
    #                        [-pi/2,0,0,-pi/6,0,pi/2,pi/4],
    #                        [pi/2,0,0,-pi/6,0,pi/2,pi/4],
    #                        [pi/2,-pi/4,0,-2.356,0,pi,pi/4],
    #                        [-pi/2,-pi/4,0,-2.356,0,pi,pi/4],
    #                        [0,-pi/4,0,-2.356,0,pi,pi/4]), 'float')
    
    positions = np.matrix(([0,0,0,-pi/6,0,pi/6,pi/4],
                           [-pi/2,0,0,-pi/6,0,pi/6,pi/4],
                           [0,-pi/4,0,-2.356,0,pi,pi/4]), 'float')
    
    for i in range (0, len(positions)):
        for j in range (0,6):
            joint_goal = move_group.get_current_joint_values()
            joint_goal[j] = positions[i,j]
    
            move_group.go(joint_goal, wait=True)
        
            move_group.stop()
        
            current_joints = move_group.get_current_joint_values()
            
            rospy.sleep(1)
    
    return all_close(joint_goal, current_joints, pi/2)

  def plan_to_pose_goal(self, coord,i):

    move_group = self.move_group


    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.position.x = coord[i,0]
    pose_goal.position.y = coord[i,1]
    pose_goal.position.z = coord[i,2]
    pose_goal.orientation.x = coord[i,3]
    pose_goal.orientation.y = coord[i,4]
    pose_goal.orientation.z = coord[i,5]
    pose_goal.orientation.w = coord[i,6]

    move_group.set_pose_target(pose_goal)

    self.pianificato = move_group.plan()
    move_group.stop()
    global s 
    s = self.pianificato[-1]
    
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.1), self.pianificato

  def execute_plan(self, plan):

    move_group = self.move_group
    move_group.execute(plan)
      

if __name__ == '__main__':
    
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    print("ROBOT CONTROLLER NODE\n")
    
    pub = rospy.Publisher('sensor_start', start_sensor, queue_size=10)
    pub1 = rospy.Publisher('sensor_stop', stop_sensor, queue_size=10)
    pub2 = rospy.Publisher('Request_New_Coord', String, queue_size=10)
    pub3 = rospy.Publisher('Acquire_Scan', String, queue_size=10)
    pubesc = rospy.Publisher('/close_nodes', String, queue_size =10)
    pub4 = rospy.Publisher('ply_matcher', String, queue_size=10)
    
    coord_quat= np.matrix(([0.5, 0.5, 0.5, 0, 0, 0, 1],
                           [0.2, 0.2, 0.2, 1, 0, 0, 0]),'float')
    
    # while not rospy.is_shutdown():

    tutorial = MoveGroupPythonIntefaceTutorial()
    
    rospy.sleep(1)
    pubesc.publish('OPEN')
    
    print('\nVERIFY THAT ROBOT HAS ENOUGH SPACE TO MOVE, Than press ENTER') 
    raw_input() 
    print('\nSTARTING SENSOR') 
    sensor_start = start_sensor()
    sensor_start.start = 1
    pub.publish(sensor_start)
    rospy.sleep(2)
    print('\nOCTOMAP CREATION')
    aa=0
    list_pose=[]
    indexes=[]
    
    for i in range(0, len(coord_quat)):
        tutorial.plan = tutorial.plan_to_pose_goal(coord_quat,i)
        if s.val==1:
            aa=aa+1
            indexes.append(i)
            print('\nPLAN READY')
            print('Press ENTER to EXECUTE') 
            raw_input()
            tutorial.execute_plan(tutorial.pianificato[1])
            print('EXECUTING TRAJECTORY')
            rospy.sleep(1)
            print('ACQUIRING POINTCLOUD')
            pub3.publish('ACQUIRE_NOW')
            rospy.sleep(2)
            #i=0
  
            print('POINTCLOUD SAVED')
            print('LOADING NEW SET OF COORDINATES')
            print('--------------------------------------------')
         
        if s.val==-1:
            print('\nPLAN FAILED: COORDINATE NOT REACHABLE')
            print('LOADING NEW SET OF COORDINATES')
            print('--------------------------------------------')
        if i==len(coord_quat)-1:
            print('ALL COORDINATES EXECUTED!')
            
    # rospy.sleep(5)
    print('\nOCTOMAP CREATION COMPLETED!')  
    sensor_stop = stop_sensor()
    sensor_stop.stop = 1
    pub1.publish(sensor_stop)
    print('ELABORATING A UNIQUE PLY FILE')
    
    coord_planned = coord_quat[indexes,:]
    txt = root+'/catkin_ws/src/planner_online/Pointcloud_Acquired/position_list.txt'
    np.savetxt(txt, coord_planned)
    pub4.publish('MATCHING')
    
    end = rospy.wait_for_message('/close_nodes', String)
    print('CLOSING NODE')
    rospy.sleep(5)
    sys.exit
        


        
        


