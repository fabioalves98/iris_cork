
import sys, time, copy, socket, struct, fcntl, xmlrpclib
import subprocess
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi, cos, sin
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Empty, Trigger
from ur_msgs.srv import SetSpeedSliderFraction



class ArmControl:
    
    def __init__(self, robot_ip='10.1.0.2'):
        
        self.ip = robot_ip
        self.gripper_connection_port = 44221
        
        self.currentSpeed = None

        ## Connecting to gripper server proxy (should work for sim and live)
        connstr = "http://{}:{}/RPC2".format(self.ip, self.gripper_connection_port)
        try:
            self.grpc = xmlrpclib.ServerProxy(connstr)
            self.gid = self.grpc.GetGrippers()[0]
        except Exception as e:
            rospy.logerr("[CORK-IRIS] Couldn't connect to gripper xmlrpc proxy")
            rospy.logerr(e)
            # sys.exit(0)

        try:        
            release_limit = self.grpc.GetReleaseLimit(self.gid, 1)
            if(release_limit < 65.0):
                rospy.logwarn("[CORK-IRIS] Gripper release limit might be too low!!")
        except Exception as e:
            rospy.logwarn("[CORK-IRIS] Couldn't get gripper release limit")
            # rospy.logerr(e)
            # sys.exit(0)

        # moveit_commander.roscpp_initialize(sys.argv)
        try:
            self.robot = moveit_commander.RobotCommander()
            self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        except Exception as e:
            rospy.logerr("[CORK-IRIS] Couldn't load robot arm or move_group")
            rospy.logerr(e)
            # sys.exit(0)
            return
        rospy.loginfo("[CORK-IRIS] Starting robot arm control!")

        
    def printGeneralStatus(self):
        # We can get the name of the reference frame for this robot:
        print("============ Planning frame: ", self.move_group.get_planning_frame())

        # We can also print the name of the end-effector link for this group:
        print("============ End effector link: ", self.move_group.get_end_effector_link())

        # We can get a list of all the groups in the robot:
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state:", self.robot.get_current_state())



    def cartesianGoal(self, waypoints):
        self.move_group.set_start_state(self.robot.get_current_state())

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.05,        # eef_step
            0.0)         # jump_threshold
        time.sleep(1.0) # Gives time for plan to show

        self.move_group.execute(plan, wait=True)

    def jointGoal(self, joints):
        joint_goal = self.move_group.get_current_joint_values()

        for i in range(0, len(joints)):
            if (joints[i] != None):
                joint_goal[i] = joints[i]
        
        # joint_goal = joints

        # # The go command can be called with joint values, poses, or without any
        # # parameters if you have already set the pose or joint target for the group
        rospy.loginfo("Sending Joint Action - %s", joints)

        # STOMP
        self.move_group.set_start_state(self.robot.get_current_state())
        plan = self.move_group.plan(joint_goal)
        self.move_group.execute(plan)

        # # OMPL
        # plan = self.move_group.go(joint_goal, wait=True)
        
        # # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()


    def poseGoal(self, coordinates, orientation):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = coordinates[0] if coordinates[0] != None else self.getPose().position.x
        pose_goal.position.y = coordinates[1] if coordinates[1] != None else self.getPose().position.y
        pose_goal.position.z = coordinates[2] if coordinates[2] != None else self.getPose().position.z

        # pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(orientation[0], orientation[1], orientation[2]))
        pose_goal.orientation.x = orientation[0] if orientation[0] != None else self.getPose().orientation.x
        pose_goal.orientation.y = orientation[1] if orientation[1] != None else self.getPose().orientation.y
        pose_goal.orientation.z = orientation[2] if orientation[2] != None else self.getPose().orientation.z
        pose_goal.orientation.w = orientation[3] if orientation[3] != None else self.getPose().orientation.w
        
        rospy.loginfo("Sending Pose goal - %s \n %s", coordinates, orientation)

        # STOMP
        self.move_group.set_start_state(self.robot.get_current_state())
        plan = self.move_group.plan(pose_goal)
        self.move_group.execute(plan)
        
        # # OMPL
        # self.move_group.set_pose_target(pose_goal)
        # plan = self.move_group.go(wait=True)
        
        
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()


    def simpleMove(self, movement, direction=pi/4):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        joint_values = self.move_group.get_current_joint_values()

        if movement[0] != 0:
            wpose.position.x += movement[0] * cos(direction)
            wpose.position.y += movement[0] * sin(direction)
        if movement[1] != 0:
            wpose.position.x += movement[1] * -sin(direction)
            wpose.position.y += movement[1] * cos(direction)
        if movement[2] != 0:  
            wpose.position.z += movement[2]

        waypoints.append(copy.deepcopy(wpose))
        
        print("Sending Move Action - ", movement)
        self.cartesianGoal(waypoints)
    
    def simpleRotate(self, rotation):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        orignal = quaternion_from_euler(0, 0, 0)
        orignal[0] = wpose.orientation.x
        orignal[1] = wpose.orientation.y
        orignal[2] = wpose.orientation.z
        orignal[3] = wpose.orientation.w

        quat = quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        
        wpose.orientation = geometry_msgs.msg.Quaternion(*quaternion_multiply(orignal, quat))
        waypoints.append(copy.deepcopy(wpose))
        
        print("Sending Rotate Action - ", rotation)
        self.cartesianGoal(waypoints)

    def getJointValues(self):
        return self.move_group.get_current_joint_values()

    def getPose(self):
        return self.move_group.get_current_pose().pose

    def grip(self):
        self.grpc.Grip(self.gid, 1)

    def release(self):
        self.grpc.Release(self.gid, 1)

    def config_gripper(self, new_limit):
        ''' Configs. the release limit so that the gripper opens up to "new_limit" on release '''
        print('Current state: ', self.grpc.GetState(self.gid))
        print('Current release limit: ', self.grpc.GetReleaseLimit(self.gid, 1))
        print(self.grpc.SetReleaseLimit(self.gid, 1, new_limit))
        print('New release limit: ', self.grpc.GetReleaseLimit(self.gid, 1))


    def setSpeed(self, speed):
        try:
            rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=2.5)
        except Exception as e:
            rospy.logwarn("[CORK-IRIS] Service for setting speed is not available!")
            return
        set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        ans = set_speed(speed)
        if ans.success:
            self.currentSpeed = speed
            return self.currentSpeed

    def getSpeed(self):
        if(not self.currentSpeed):
            print("Current speed unknown!")
            return
        return self.currentSpeed

    def saveJointPosition(self, path, position_name):
        hs = open(path,"a")
        hs.write("\n" + position_name + " : " + str(self.getJointValues()))
        hs.close()
        rospy.loginfo("Successfully saved position '" + position_name + "' to '" + path + "'")

    def getLinks(self, group="endeffector"):
        return self.robot.get_link_names(group=group)