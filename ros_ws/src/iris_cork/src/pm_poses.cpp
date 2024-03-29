#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pm_poses");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Information about robot and joints
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Get Pose Transform
    geometry_msgs::TransformStamped del_90_pose_tf;
    while(true)
    {
        try
        {
            del_90_pose_tf = tfBuffer.lookupTransform("world", "del_180_pose", ros::Time(0));
            break;      
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    std::cout << del_90_pose_tf << std::endl;

    geometry_msgs::Pose pose;
    pose.position.x = del_90_pose_tf.transform.translation.x;
    pose.position.y = del_90_pose_tf.transform.translation.y;
    pose.position.z = del_90_pose_tf.transform.translation.z;

    pose.orientation.x = del_90_pose_tf.transform.rotation.x;
    pose.orientation.y = del_90_pose_tf.transform.rotation.y;
    pose.orientation.z = del_90_pose_tf.transform.rotation.z;
    pose.orientation.w = del_90_pose_tf.transform.rotation.w;


    ros::Rate rate(50);
    while(ros::ok())
    {
        move_group.setStartState(*move_group.getCurrentState());
        
        move_group.setGoalPositionTolerance(0.01);
        move_group.setGoalOrientationTolerance(0.01);
        move_group.setPlanningTime(1);
        move_group.setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        std::cout << (success == 1 ? "SUCCESS" : "FAILED") << std::endl;

        if (success)
        {
            move_group.execute(my_plan);
            break;
        }
    }

    std::vector<double> joint_values;
    // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }

    // // Testing joint limits
    // joint_values[0] = 5.57;
    // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    // kinematic_state->enforceBounds();
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


    // // Forward Kinematics
    // kinematic_state->setToRandomPositions(joint_model_group);
    // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }

    // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_link");

    // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    // geometry_msgs::Pose pose;
    // pose.position.x = transform.getOrigin().getX();
    // pose.position.y = transform.getOrigin().getY();
    // pose.position.z = transform.getOrigin().getZ();
    // pose.orientation.x = transform.getRotation().getX();
    // pose.orientation.y = transform.getRotation().getY();
    // pose.orientation.z = transform.getRotation().getZ();
    // pose.orientation.w = transform.getRotation().getW();

    // // Inverse Kinematics
    // double timeout = 0.1;
    // bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

    // if (found_ik)
    // {
    //     kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    //     for (std::size_t i = 0; i < joint_names.size(); ++i)
    //     {
    //         ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    //     }
    // }
    // else
    // {
    //     ROS_INFO("Did not find IK solution");
    // }

    // std::cout << joint_model_group->getLinkModelNames().back() << std::endl;

    // // Jacobian
    // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    // Eigen::MatrixXd jacobian;
    // kinematic_state->getJacobian(joint_model_group,
    //                         kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
    //                         reference_point_position, jacobian);
    // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    
    ros::shutdown();

    return 0;
}