#! /usr/bin/env python
import rospy
import copy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, PlanningScene
import actionlib
from actionlib_msgs.msg import *
import tf
from tf import transformations as tft
from grab_tool_action.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
from hrl_haptic_manipulation_in_clutter_srvs.srv import *
from hrl_haptic_manipulation_in_clutter_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
import hrl_msgs.msg
import numpy as np
from math import cos, sin, pi

class toolPickingServer(): #object):
    _feedback=GrabToolActionFeedback()
    _result=GrabToolActionResult()

    def __init__(self, arm='l'):
        self.rate=20
        self.timeout=rospy.Duration(120)
        #Arm should be 'r' for right arm or 'l' for left arm
        #Turn on MoveIt and groups for arms and grippers
        robot=moveit_commander.RobotCommander() #interface to robot
        self.scene=moveit_commander.PlanningSceneInterface() #interface to world around the robot
        #self.group_rightarm=moveit_commander.MoveGroupCommander('right_arm') #interface to joints of the right arm             
        self.group_leftarm=moveit_commander.MoveGroupCommander('left_arm') #interface to joints of the left arm        
        #self.group_leftgripper=moveit_commander.MoveGroupCommander('left_gripper')        
        #self.group_rightgripper=moveit_commander.MoveGroupCommander('right_gripper')      
        self.group_leftarm.set_planner_id("RRTConnectkConfigDefault")
        #self.group_rightarm.set_planner_id("RRTConnectkConfigDefault")
       
        #Start action clients for grippers
        self.r_close_gripper_client=actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.l_close_gripper_client=actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
      
        #Services and publishers for Haptic MPC
        #self.enable_haptic=rospy.ServiceProxy('/haptic_mpc/enable_mpc', EnableHapticMPC)
        self.r_enable_haptic=rospy.ServiceProxy('/haptic_mpc/enable_mpc', EnableHapticMPC)
        #self.l_goal_pub=rospy.Publisher('/haptic_mpc/goal_pose', PoseStamped)
        self.r_goal_pub=rospy.Publisher('/haptic_mpc/goal_pose', PoseStamped)
        #self.l_goal_posture_pub = rospy.Publisher("/haptic_mpc/goal_posture", hrl_msgs.msg.FloatArray)
        self.r_goal_posture_pub = rospy.Publisher("/haptic_mpc/goal_posture", hrl_msgs.msg.FloatArray) 
        #self.l_haptic_weights = rospy.Publisher("/haptic_mpc/weights", HapticMpcWeights)
        self.r_haptic_weights = rospy.Publisher("/haptic_mpc/weights", HapticMpcWeights)

        #Subscribers for MPC
        #rospy.Subscriber('/haptic_mpc/gripper_pose', PoseStamped, self.lgripperPoseCallback)
        rospy.Subscriber('/haptic_mpc/gripper_pose', PoseStamped, self.rgripperPoseCallback)
        rospy.Subscriber('/l_arm_controller/state', JointTrajectoryControllerState, self.ljointStateCallback)
        rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, self.rjointStateCallback)

        #Weights for MPC
        #Orient weight for gripper pose
        self.orientation_weights = HapticMpcWeights()
        self.orientation_weights.position_weight = 1
        self.orientation_weights.orient_weight = 1
        self.orientation_weights.posture_weight = 0
        #self.execute_callback(goal)

        #Posture weight for posture poses
        self.posture_weights = HapticMpcWeights()
        self.posture_weights.position_weight = 0
        self.posture_weights.orient_weight = 0
        self.posture_weights.posture_weight = 1
       

        self.tool_listener=tf.TransformListener()
        markers=['ar_marker_1', 'ar_marker_2', 'ar_marker_3', 'ar_marker_4']
        #for marker in markers:        
            #Remove markers from last run
        #    self.scene. remove_attached_object('base_footprint', marker)
        #self.add_tools_to_scene(markers)

        #Set up action server
        self._as=actionlib.SimpleActionServer('grab_tool', GrabToolAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
    #def lgripperPoseCallback(self, msg):
    #    self.lgripperPose=msg

    def ljointStateCallback(self, msg):
        self.ljointState=msg.actual.positions

    def rgripperPoseCallback(self, msg):
        self.rgripperPose=msg

    def rjointStateCallback(self, msg):
        self.rjointState=msg.actual.positions
 
    def execute_cb(self, goal):
        goal_topic=goal.tag_topic
        print(goal_topic)
        arm=goal.arm
        print(arm)
        #TODO: figure out how to pass in arm into action server (list of strings?)
        success=self.move_gripper(True, arm) #open gripper in preparation for moving the arm
        rospy.loginfo("Opening gripper")        
        plan=self.plan_approach(arm, goal_topic, success)
        if plan is False:
            rospy.logerr("Planning Failed. Aborting")
            self._as.set_aborted()
        else:
            rospy.loginfo("Planning successful, executing plan")
            print plan            
        #TODO: make sure it aborts of no plan is found cuz then the rest is useless
        success=self.execute_plan(plan, arm)
        plan=self.approach_tool(arm, goal_topic, success)       
        succeeded=self.execute_plan(plan, arm)        
        if succeeded:           
            success=self.move_gripper(False, arm)
            if success:
                rospy.loginfo("Tool Successfully Grasped!")
                self._as.set_success()
            else:
                rospy.logerr("Tool picking failed. Tool not grasped. Aborting")
                self._as.set_aborted()        
        else:
            #TODO: insert recovery behaviers?
            rospy.logerr("Tool picking approach failed: aborting")
            self._as.set_aborted()
        self._as.set_aborted()
    def add_tools_to_scene(self, markers):
        #Add tools to the planning scene
        for marker_name in markers:
            try:
                now=rospy.Time(0)
                self.tool_listener.waitForTransform('base_footprint', marker_name, now, rospy.Duration(10)) #look up transform between given tool and the torso (torso won't move)
                trans, rot=self.tool_listener.lookupTransform('base_footprint', marker_name, now)
                mesh_pose=PoseStamped()
                mesh_pose.header.stamp=rospy.Time.now()
                mesh_pose.header.frame_id='base_footprint'
                mesh_pose.pose.position.x=trans[0]-0.03
                mesh_pose.pose.position.y=trans[1]-0.02
                mesh_pose.pose.position.z=trans[2]+0.05
                mesh_pose.pose.orientation.x=0 #rot[0]
                mesh_pose.pose.orientation.y=0 #rot[1]
                mesh_pose.pose.orientation.z=0 #rot[2]
                mesh_pose.pose.orientation.w=1 #rot[3]           
                self.scene.attach_box('base_footprint', marker_name, mesh_pose, [0.1, 0.03, 0.075], ['r_gripper_r_finger_tip_link','r_gripper_r_finger_link', 'r_gripper_l_finger_tip_link', 'r_gripper_l_finger_link', 'l_gripper_r_finger_tip_link', 'l_gripper_r_finger_link', 'l_gripper_l_finger_tip_link', 'l_gripper_l_finger_link'] ) #[l, w, h] of tools
                rospy.loginfo("Added %s to scene" % marker_name) #TODO: turn these into roslog info things
               
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):               
                print "TF Exception!"

    def plan_approach(self, arm, goal_topic, succeed_gripper):
        g=PoseStamped()

        if arm is 'r'and succeed_gripper:
            self.group_rightarm.set_start_state_to_current_state()
            self.group_rightarm.clear_pose_targets()
            ee_link=self.group_rightarm.get_end_effector_link()
            print (ee_link)        
        elif arm is 'l' and succeed_gripper:
            self.group_leftarm.set_start_state_to_current_state()
            self.group_leftarm.clear_pose_targets()
            ee_link=self.group_leftarm.get_end_effector_link()
            print (ee_link)
    
        if succeed_gripper and not self._as.is_preempt_requested():
            try:
                now=rospy.Time(0)
                self.tool_listener.waitForTransform('base_footprint', goal_topic, now, rospy.Duration(10)) #look up transform between given tool and the base_footprint
                trans, rot=self.tool_listener.lookupTransform('base_footprint', goal_topic, now)
                g.header.stamp=rospy.Time.now()
                g.header.frame_id='base_footprint'
                q1=np.array([rot[0], rot[1], rot[2], rot[3]])
                q1_con=tft.quaternion_conjugate(q1) #get the conjugate
                rot_y_axis=np.array([1.0*sin(-pi/4), 1.0*sin(pi/4), 0.0*sin(pi/4), cos(pi/4)])
                q3=tft.quaternion_multiply(q1_con, q1)
                q4=tft.quaternion_multiply(rot_y_axis, q3)
                q_fin=tft.quaternion_multiply(q1, q4)
                g.pose.position.x =trans[0]-0.4 #325
                g.pose.position.y =trans[1]-0.04
                g.pose.position.z=trans[2]+0.025
                g.pose.orientation.x =0 #q_fin[0] #should be close to [0, 0, 0, 1] when base is pointed at tools
                g.pose.orientation.y =0 #q_fin[1]
                g.pose.orientation.z =0 #q_fin[2]
                g.pose.orientation.w =1 #q_fin[3]
                print(trans[0])
                print(trans[1])
                print(trans[2])
                print(q_fin[0])
                print(q_fin[1])
                print(q_fin[2])
                print(q_fin[3])
   
                if arm is 'r':
                #Adjust planning parameters for highest likelihood of finding a plan                
                    self.group_rightarm.set_goal_tolerance(0.017)
                    self.group_rightarm.allow_replanning(True)
                    self.group_rightarm.set_pose_reference_frame('base_footprint')
                    self.group_rightarm.set_planning_time(8)
                    self.group_rightarm.set_pose_target(g.pose, ee_link)
                    plan1=self.group_rightarm.plan()
                    #create and publish goal pose
                    if len(plan1.joint_trajectory.points)<1:
                        return False
                    else:
                        return plan1
                elif arm is 'l':
                    self.group_leftarm.set_goal_tolerance(0.017)
                    self.group_leftarm.allow_replanning(True)
                    self.group_leftarm.set_pose_reference_frame('base_footprint')
                    self.group_leftarm.set_planning_time(10)
                    self.group_leftarm.set_pose_target(g.pose, ee_link)
                    plan1=self.group_leftarm.plan()
                    #create and publish goal pose
                    if len(plan1.joint_trajectory.points)<1:
                        return False
                    else:
                        return plan1
            
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):               
                return False 
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()        
        elif not succeed_gripper:
            return False
        else:
            return False

    def execute_plan(self, plan, arm):
        if plan is not False and not self._as.is_preempt_requested():
            last_time=rospy.Time.now()+self.timeout
            goal=hrl_msgs.msg.FloatArray()            
            if arm is 'r':
                self.r_haptic_weights.publish(self.posture_weights)
                for pose in plan.joint_trajectory.points:
                    goal.header.stamp=rospy.Time.now()
                    goal.data=pose.positions
                    self.r_goal_posture_pub.publish(goal)
                    postureErr=np.linalg.norm(np.subtract(goal.data, self.rjointState))
                    while not postureErr<0.1 and not self._as.is_preempt_requested():
                        goal.header.stamp=rospy.Time.now()                        
                        self.r_goal_posture_pub.publish(goal)
                        postureErr=np.linalg.norm(np.subtract(goal.data,self.rjointState))
                        if rospy.Time.now()>last_time:
                           self._feedback.feedback='Timeout exceeded'
                           self._as.publish_feedback(self._feedback)
                           return False
                        elif rospy.Time.now()<last_time:
                            continue
                        self.rate.sleep()
                return True
            elif arm is 'l':
                self.l_haptic_weights.publish(self.posture_weights)
                for pose in plan.joint_trajectory.points:
                    goal.header.stamp=rospy.Time.now()
                    goal.data=pose.positions
                    self.l_goal_posture_pub.publish(goal)
                    postureErr=np.linalg.norm(np.subtract(goal.data, self.ljointState))
                    while not postureErr<0.1 and not self._as.is_preempt_requested():
                        goal.header.stamp=rospy.Time.now()                        
                        self.l_goal_posture_pub.publish(goal)
                        postureErr=np.linalg.norm(np.subtract(goal.data,self.ljointState))
                        if rospy.Time.now()>last_time:
                           self._feedback.feedback='Timeout exceeded'
                           self._as.publish_feedback(self._feedback)
                           return False
                        elif rospy.Time.now()<last_time:
                            continue
                        self.rate.sleep()             
                return True
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:
            return False
    
    def approach_tool(self, arm, goal_topic, success):
        if arm is 'r' and success and not self._as.is_preempt_requested():
            waypoints=self.rgripperPose
            self.group_rightarm.set_pose_reference_frame(waypoints.header.frame_id)
            waypoints.pose.position.x=0.1
            waypoints.pose.position.y=0.0
            waypoints.pose.position.z=0.0
            waypoints.pose.orientation.x=0  
            waypoints.pose.orientation.y=0
            waypoints.pose.orientation.z=0
            waypoints.pose.orientation.w=0 
            print waypoints.pose
            eef_step=0.01 #choose a step of 1 cm increments
            jump_threshold=0.0    
            avoid_collisions=False   
            plan, fraction=self.group_rightarm.compute_cartesian_path([waypoints.pose], eef_step, jump_threshold, avoid_collisions )
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                return plan
        elif arm is 'l' and success and not self._as.is_preempt_requested():
            waypoints=self.lgripperPose
            self.group_leftarm.set_pose_reference_frame(waypoints.header.frame_id)
            waypoints.pose.position.x=0.1
            waypoints.pose.position.y=0.0
            waypoints.pose.position.z=0.0
            waypoints.pose.orientation.x=0  
            waypoints.pose.orientation.y=0
            waypoints.pose.orientation.z=0
            waypoints.pose.orientation.w=0 
            print waypoints.pose
            eef_step=0.01 #choose a step of 1 cm increments
            jump_threshold=0.0    
            avoid_collisions=False   
            plan, fraction=self.group_leftarm.compute_cartesian_path([waypoints.pose], eef_step, jump_threshold, avoid_collisions )
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                return plan
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:
            return False

    def move_gripper(self, open_status, arm):
        if open_status and not self._as.is_preempt_requested():  #true is open the gripper
            if arm is 'r':            
                self.r_close_gripper_client.wait_for_server()
                self.r_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.08, max_effort = -1))) #open the gripper with max effort
                self.r_close_gripper_client.wait_for_result()
                result = self.r_close_gripper_client.get_result()
                state=self.r_close_gripper_client.get_state()
            elif arm is 'l':
                self.l_close_gripper_client.wait_for_server()
                self.l_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.08, max_effort = -1))) #open the gripper with max effort
                self.l_close_gripper_client.wait_for_result()
                result = self.l_close_gripper_client.get_result()
                state=self.l_close_gripper_client.get_state()           
            if state == GoalStatus.SUCCEEDED:
                #self._feedback.feedback='Open gripper succeeded' 
                #self._as.publish_feedback(self._feedback)               
                return True
            else:
                if result.stalled:
                    #self._feedback.feedback='Obstacle preventing gripper from opening'
                    #self._as.publish_feedback(self._feedback)          
                    return False
                else:
                    return False 
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()         
        else:
            if arm is 'r':   #false is close the gripper
                self.r_close_gripper_client.wait_for_server()
                self.r_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.0, max_effort = 50))) #close the gripper gently
                self.r_close_gripper_client.wait_for_result()
                result = self.r_close_gripper_client.get_result()
                state=self.r_close_gripper_client.get_state()
            elif arm is 'l':
                self.l_close_gripper_client.wait_for_server()
                self.l_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.0, max_effort = 50))) #close the gripper gently
                self.l_close_gripper_client.wait_for_result()
                result = self.l_close_gripper_client.get_result()
                state=self.l_close_gripper_client.get_state() 
            if state == GoalStatus.SUCCEEDED:
                #self._feedback.feedback="Did not succeed closing gripper around object try again" #make this a message sent to user                
                #self._as.publish_feedback(self._feedback)
                return False
            else:
                if result.stalled:
                    #self._feedback.feedback="Gripper successfully closed aroud an object"   #make this a message sent to user                     
                    #self._as.publish_feedback(self._feedback)                    
                    return True #this is the one we want--->means that the gripper closed around something
                elif result.reached_goal: 
                    #self._feedback.feedback="Did not grasp tool correctly, try again"  #make this a message sent to user   
                    #self._as.publish_feedback(self._feedback)                    
                    return False 
                else:
                    return False
def main():
#if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grab_tool_server', anonymous=True)
    toolPickingServer()
    while not rospy.is_shutdown():
        rospy.spin()


