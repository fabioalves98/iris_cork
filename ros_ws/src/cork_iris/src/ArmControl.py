import moveit_commander
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Empty, Trigger
from ur_msgs.srv import SetSpeedSliderFraction


import sys, time, copy, socket, struct, fcntl

from math import pi, cos, sin
import subprocess



class ArmControl:
    
    def __init__(self, rospy):
        self.rospy = rospy
        print("Starting robot arm control!")
        self.currentSpeed = None
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            self.robot = moveit_commander.RobotCommander()
            self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        except Exception as e:
            print("Couldn't load robot arm or move_group")
            print(e)
            sys.exit(0)


    def cartesianGoal(self, waypoints):
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
        self.rospy.loginfo("Sending Joint Action - %s", joints)
        plan = self.move_group.go(joint_goal, wait=True)

        # # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()


    def poseGoal(self, coordinates, orientation):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = coordinates[0]
        pose_goal.position.y = coordinates[1]
        pose_goal.position.z = coordinates[2]

        pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(orientation[0], orientation[1], orientation[2]))

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
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

    def load_and_play_program(self, program_filename):
        try:
            self.rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program', timeout=2.5)
        except Exception as e:
            self.rospy.logwarn("[CORK-IRIS] Service for loading a program is not available. Can't load '%s'", program_filename)
            return
        load_program = self.rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        print(load_program(program_filename))

        try:
            self.rospy.wait_for_service('/ur_hardware_interface/dashboard/play', timeout=2.5)
        except Exception as e:
            self.rospy.logwarn("[CORK-IRIS] Service for starting a program is not available. Can't start '%s'", program_filename)
            return
        play_program = self.rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        print(play_program()) 

    def grip(self):
        self.load_and_play_program('grip.urp')

    def release(self):
        self.load_and_play_program('release.urp')
  
    def setSpeed(self, speed):
        try:
            self.rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=2.5)
        except Exception as e:
            self.rospy.logwarn("[CORK-IRIS] Service for setting speed is not available!")
            return
        set_speed = self.rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        print(set_speed(speed))
        ## TODO: Check if the message returned success and only update the speed after that
        self.currentSpeed = speed

    def getSpeed(self):
        if(not self.currentSpeed):
            print("Current speed unknown!")
            return
        return self.currentSpeed

    def saveJointPosition(self, path, position_name):
        hs = open(path,"a")
        hs.write(position_name + " : " + str(self.getJointValues()))
        hs.close() 



  
   