#include "../include/circle.hpp"
#include <cmath>
// https://www.desmos.com/calculator/mphgdijqgb

int main(int argc, char **argv){
    // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Create Planning Options
    MoveitPlanning::PlanningOptions planning_options = MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create an instance of MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    // Create instance of joint target plan
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 0.94;
    joint_targets["shoulder_lift_joint"] = -1.13;
    joint_targets["shoulder_pan_joint"] = 1.13;
    joint_targets["wrist_1_joint"] = -1.51;
    joint_targets["wrist_2_joint"] = 2.51;
    joint_targets["wrist_3_joint"] = 3.39;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    // Create instance of pose target plan
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target;
    
    pose_target.position.x = 0.25;
    pose_target.position.y = 0.2;
    pose_target.position.z = 1.1;
    pose_target.orientation.x = 0.0;
    pose_target.orientation.y = 0.0;
    pose_target.orientation.z = 3.14;
    pose_target.orientation.w = 0.0;

    bool pose_plan_success;
    std::string reference_frame = "base_link";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }


    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;
    geometry_msgs::Pose end_pose = start_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    double z = 1.05;
    double xyDist = 0.54;

    end_pose.position.x = xyDist * cos(0.0);
    end_pose.position.y = xyDist * sin(0.0);
    end_pose.position.z = z;

    waypoints.push_back(end_pose);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);


    std::vector<double> current_angles = arm_move_group.getCurrentJointValues();
    joint_targets["elbow_joint"] = current_angles[0];
    joint_targets["shoulder_lift_joint"] = current_angles[1];
    joint_targets["shoulder_pan_joint"] = 0;
    joint_targets["wrist_1_joint"] = current_angles[3];
    joint_targets["wrist_2_joint"] = current_angles[4];
    joint_targets["wrist_3_joint"] = current_angles[5];
    

    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);
    if (joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    joint_targets["shoulder_pan_joint"] = (3.0 * M_PI) / 2.0;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if (joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    
    // Define waypoints for the cartesian path

    // // generate waypoints for first half of circle (10 points toatl 0 <= tehta <= pi)
    // for (int i = 1; i < 10; i++) {
    //     end_pose.position.x = xyDist * cos(M_PI / 10 * i);
    //     end_pose.position.y = xyDist * sin(M_PI / 10 * i);
    //     // end_pose.position.z += 0.01;

    //     waypoints.push_back(end_pose);
    // }

    // // do the same in opposite direction (from pi to 2pi)
    // for (int i = 10; i < 20; i++) {
    //     end_pose.position.x = xyDist * cos(M_PI / 10 * i);
    //     end_pose.position.y = xyDist * sin(M_PI / 10 * i);
    //     // end_pose.position.z -= 0.01;
        
    //     waypoints.push_back(end_pose);
    // }

    // end_pose.position.x = xyDist * cos(0.0);
    // end_pose.position.y = xyDist * sin(0.0);
    // end_pose.position.z = z;

    // waypoints.push_back(end_pose);

    // moveit_msgs::RobotTrajectory trajectory;
    // trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    // n.setParam("/record_pose", true);
    // arm_move_group.execute(trajectory);
    // n.setParam("/record_pose", false);

}