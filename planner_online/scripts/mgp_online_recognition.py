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

  def plan_to_pose_goal(self, coord):

    move_group = self.move_group


    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.orientation.x = coord.qx
    pose_goal.orientation.y = coord.qy
    pose_goal.orientation.z = coord.qz
    pose_goal.orientation.w = coord.qw
    pose_goal.position.x = coord.x
    pose_goal.position.y = coord.y
    pose_goal.position.z = coord.z

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
    
    while not rospy.is_shutdown():

        tutorial = MoveGroupPythonIntefaceTutorial()
        rospy.sleep(1)
        
        print('\nVERIFY THAT ROBOT HAS ENOUGH SPACE TO MOVE, Than press ENTER') 
        raw_input() 
        print('\nSTARTING SENSOR') 
        sensor_start = start_sensor()
        sensor_start.start = 1
        pub.publish(sensor_start)
        rospy.sleep(2)
        print('\nSTARTING CREATING OCTOMAP')      
       # tutorial.OctoMap_Creation()
       # rospy.sleep(5)
        print('\nOCTOMAP CREATION COMPLETED!')  
        sensor_stop = stop_sensor()
        sensor_stop.stop = 1
        rospy.sleep(1)
        pub1.publish(sensor_stop)
        j=0
        while not rospy.is_shutdown(): 
                    
            if j==0:            
                print('\nREQUESTED NEW SET OF COORDINATES')
                pub2.publish('\nREQUESTING NEW SET OF COORDINATES')
             
            j=1
            coord = coord_to_reach()
            print('\nWAITING FOR THE SET OF COORDINATES AND ORIENTATIONS') 
            end = rospy.wait_for_message('/close_nodes', String)
            if end.data == 'CLOSE':
                    sys.exit()
            coord = rospy.wait_for_message('/position_and_orientations', coord_to_reach)
            print('x: %1.4f' %coord.x,
                  'y: %1.4f' %coord.y,
                  'z: %1.4f' %coord.z,
                  'qx: %1.4f' %coord.qx,
                  'qy: %1.4f' %coord.qy,
                  'qz: %1.4f' %coord.qz,
                  'qw: %1.4f' %coord.qw)
            tutorial.plan = tutorial.plan_to_pose_goal(coord)
            
            if s.val == -1:
                print('\nPLAN ERROR: POSITION NOT REACHABLE, press ENTER to TRY ANOTHER ONE')
                raw_input()

                
            elif s.val == 1:
                print('\nPLAN COMPLETED SUCCESSFULLY, press ENTER to EXECUTE')
                raw_input()
                print('EXECUTING TRAJECTORY')
                tutorial.execute_plan(tutorial.pianificato[1])
                print('\nSTARTING SENSOR')
                sensor_start.start = 0
                pub.publish(sensor_start)
                print('\nFINDING OBJECT')
                rospy.wait_for_message("/object_found", String)
                sensor_stop.stop = 1
                pub1.publish(sensor_stop)
                print('\nSTOPPING SENSOR: POINTCLOUD SAVED')

