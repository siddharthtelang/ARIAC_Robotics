#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>

/**
 * @brief Construct a new Gantry Control:: Gantry Control object
 * 
 * @param node 
 */
GantryControl::GantryControl(ros::NodeHandle &node) : node_("/ariac/gantry"),
                                                      planning_group_("/ariac/gantry/robot_description"),
                                                      full_robot_options_("Full_Robot", planning_group_, node_),
                                                      left_arm_options_("Left_Arm", planning_group_, node_),
                                                      right_arm_options_("Right_Arm", planning_group_, node_),
                                                      left_ee_link_options_("Left_Endeffector", planning_group_, node_),
                                                      right_ee_link_options_("Right_Endeffector", planning_group_, node_),
                                                      full_robot_group_(full_robot_options_),
                                                      left_arm_group_(left_arm_options_),
                                                      right_arm_group_(right_arm_options_),
                                                      left_ee_link_group_(left_ee_link_options_),
                                                      right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}

////////////////////////////
void GantryControl::init()
{
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();

    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    // preset locations

    // joint positions to go to start location
    start_.gantry = {0, 0, 0};
    start_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin3
    bin3_.gantry = {4.0, -1.1, 0.};
    bin3_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin3_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv2, given
    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv1
    agv1_.gantry = {0.6, -6.9, PI};
    agv1_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv1_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup *joint_model_group =
        full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

    gantry_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
        "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);

    while ((current_gantry_controller_state_.joint_names.size() == 0) || 
    (current_left_arm_controller_state_.joint_names.size() == 0) || 
    (current_right_arm_controller_state_.joint_names.size() == 0))
    {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}


////////////////////////////
geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv)
{
    
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1") == 0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = kit_tray;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;

    
    for (int i{0}; i < 15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Rate rate(10);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i = 0; i < 10; i++)
    {
        try
        {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
            // ROS_WARN_STREAM("target in world frame: " << world_target_tf.transform.rotation.x <<" "<<world_target_tf.transform.rotation.y<<" "<<world_target_tf.transform.rotation.z<<" "<<world_target_tf.transform.rotation.w);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    for (int i = 0; i < 10; i++)
    {
        try
        {
            ee_target_tf = tfBuffer.lookupTransform("world", "left_ee_link",
                                                    ros::Time(0), timeout);
            // ROS_WARN_STREAM("left_ee_link in target frame: " << ee_target_tf.transform.rotation.x <<" "<<ee_target_tf.transform.rotation.y<<" "<<ee_target_tf.transform.rotation.z<<" "<<ee_target_tf.transform.rotation.w);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

/**
 *ee_target_tf and  world_target_tf are expressed in the same frame: "world"
 We want to find the relative rotation, q_r, to go from ee_target_tf to world_target_tf
 q_r = world_target_tf*ee_target_tf_inverse
 * 
 */
    tf2::Quaternion ee_target_tf_inverse(
        ee_target_tf.transform.rotation.x,
        ee_target_tf.transform.rotation.y,
        ee_target_tf.transform.rotation.z,
        -ee_target_tf.transform.rotation.w);

    tf2::Quaternion part_in_tray(world_target_tf.transform.rotation.x,
                                 world_target_tf.transform.rotation.y,
                                 world_target_tf.transform.rotation.z,
                                 world_target_tf.transform.rotation.w);

    tf2::Quaternion qr = part_in_tray * ee_target_tf_inverse;
    qr.normalize();

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    // ros::Duration(10).sleep();
    return world_target;
}

// ////////////////////////////
// bool GantryControl::pickPart_smooth(part part)
// {
//     //--Activate gripper
//     activateGripper("left_arm");
//     geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//     std::vector<geometry_msgs::Pose> waypoints;
//     // waypoints.push_back(currentPose); // add waypoint, initial position. want straight line motion. update: according to moveit documentation, don't need initial point


//     part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON_ONE_MOTION_GRASP;
//     part.pose.orientation.x = currentPose.orientation.x;
//     part.pose.orientation.y = currentPose.orientation.y;
//     part.pose.orientation.z = currentPose.orientation.z;
//     part.pose.orientation.w = currentPose.orientation.w;
//     //    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);

//     waypoints.push_back(part.pose); // add waypoint, move down

//     waypoints.push_back(currentPose); // add waypoint, move back up



//     moveit_msgs::RobotTrajectory trajectory;
//     const double jump_threshold = 0.0;
//     const double eef_step = 0.1; // changed from 0.01 to 0.1, fixed some errors
//     double fraction = left_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//     ROS_INFO_STREAM("trajectory computed!");
//     // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

//     // // also calculate path from part and back
//     // std::vector<geometry_msgs::Pose> waypoints_back;
//     // waypoints_back.push_back(part.pose); // add waypoint
//     // waypoints_back.push_back(currentPose); // add waypoint, want straight line
//     // moveit_msgs::RobotTrajectory trajectory_back;
//     // double fraction_back = left_arm_group_.computeCartesianPath(waypoints_back, eef_step, jump_threshold, trajectory_back);
//     // ROS_INFO_STREAM("trajectory back computed!");

//     auto state = getGripperState("left_arm");
//     if (state.enabled) {

//         ROS_INFO_STREAM("[Gripper] = enabled");

//         for (int i=1; i<=2; i++) { // Two tries: execute trajectory, try to pick up part.
//             //--Move arm to part
//             left_arm_group_.execute(trajectory);
//             ROS_INFO_STREAM("trajectory forward executed!!!!");


//             auto aa = left_arm_group_.execute(trajectory); // an error code, 1 if good, usually -4 if bad

//             while( aa != 1 ) { // while error code indicates failure to move along trajectory. Keep trying until it does, it should be able to simply move.
//                 // geometry_msgs::Pose nowPose = left_arm_group_.getCurrentPose().pose; // check that trajectory back was actually executed, in case of "path plan succeeded but controller failed"
//                 std::vector<geometry_msgs::Pose> waypoints_back_again;

//                 // waypoints_back_again.push_back(nowPose); // add waypoint, current position
//                 // waypoints_back_again.push_back(part.pose); // add waypoint, move down
//                 // waypoints_back_again.push_back(nowPose); // add waypoint, move back to current position
//                 waypoints_back_again.push_back(currentPose); // add waypoint, move back to original position

//                 moveit_msgs::RobotTrajectory trajectory_back_again;
//                 double fraction_back_again = left_arm_group_.computeCartesianPath(waypoints_back_again, eef_step, jump_threshold, trajectory_back_again);
//                 ROS_INFO_STREAM("trajectory back again computed!");
//                 aa = left_arm_group_.execute(trajectory_back_again);
//                 ROS_INFO_STREAM("trajectory back again executed (second+ try) !!!!============");

//             }

//             auto state = getGripperState("left_arm");
//             if (state.attached) {
//                 ROS_INFO_STREAM("[Gripper] = object attached");
//                 return true;
//             }
        
//         }

//         return false; // if this returns false, means we have tried 1-2 times to pick the part, and failed both times.

//     }
//     else {
//         ROS_INFO_STREAM("[Gripper] = not enabled");
//         return false;
//     }

// }


////////////////////////////
bool GantryControl::pickPart(part part)
{
    //--Activate gripper
    activateGripper("left_arm");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(currentPose); // add waypoint


    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
    //    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);

    waypoints.push_back(part.pose); // add waypoint, want straight line


    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1; // changed from 0.01 to 0.1, fixed some errors
    double fraction = left_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_STREAM("trajectory computed!");
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // also calculate path from part and back
    std::vector<geometry_msgs::Pose> waypoints_back;
    // waypoints_back.push_back(part.pose); // add waypoint
    waypoints_back.push_back(currentPose); // add waypoint, want straight line
    moveit_msgs::RobotTrajectory trajectory_back;
    double fraction_back = left_arm_group_.computeCartesianPath(waypoints_back, eef_step, jump_threshold, trajectory_back);
    ROS_INFO_STREAM("trajectory back computed!");

    auto state = getGripperState("left_arm");
    if (state.enabled) {

        ROS_INFO_STREAM("[Gripper] = enabled");

        for (int i=1; i<=2; i++) { // Two tries: execute trajectory, try to pick up part.
            //--Move arm to part
            left_arm_group_.execute(trajectory);
            ROS_INFO_STREAM("trajectory forward executed!!!!");


            auto aa = left_arm_group_.execute(trajectory_back); // an error code, 1 if good, usually -4 if bad

            while( aa != 1 ) { // while error code indicates failure to move along trajectory
                geometry_msgs::Pose nowPose = left_arm_group_.getCurrentPose().pose; // check that trajectory back was actually executed, in case of "path plan succeeded but controller failed"
                std::vector<geometry_msgs::Pose> waypoints_back_again;
                // waypoints_back_again.push_back(nowPose); // add waypoint
                waypoints_back_again.push_back(currentPose); // add waypoint, want straight line
                moveit_msgs::RobotTrajectory trajectory_back_again;
                double fraction_back_again = left_arm_group_.computeCartesianPath(waypoints_back_again, eef_step, jump_threshold, trajectory_back_again);
                ROS_INFO_STREAM("trajectory back again computed!");
                aa = left_arm_group_.execute(trajectory_back_again);
                ROS_INFO_STREAM("trajectory back again executed (second try) !!!!============");

            }

            auto state = getGripperState("left_arm");
            if (state.attached) {
                ROS_INFO_STREAM("[Gripper] = object attached");
                return true;
            }
        
        }

        return false; // if this returns false, means we have tried 1-2 times to pick the part, and failed both times.

    }
    else {
        ROS_INFO_STREAM("[Gripper] = not enabled");
        return false;
    }

}

////////////////////////////
void GantryControl::placePart(part part, std::string agv)
{
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);

    ros::Duration(2.0).sleep();
    //--TODO: Consider agv1 too
    if (agv == "agv2")
        goToPresetLocation(agv2_);
    else if (agv == "agv1")
        goToPresetLocation(agv1_);
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);

    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();

    deactivateGripper("left_arm");
    auto state = getGripperState("left_arm");
//    if (state.attached)
        // ;// pass, don't necesarily go back to start in case of faulty part
        //goToPresetLocation(start_);
}

bool GantryControl::replaceFaultyPart(part part, std::string agv)
{
    activateGripper("left_arm");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);
    ros::Duration(2.0).sleep();
    //--TODO: Consider agv1 too
/*    if (agv == "agv2")
        goToPresetLocation(agv2_);
    else if (agv == "agv1")
        goToPresetLocation(agv1_);*/
//    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);
    target_pose_in_tray.position.z += model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();
    activateGripper("left_arm");
    auto state = getGripperState("left_arm");

//    auto state = getGripperState("left_arm");
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
//        left_arm_group_.setPoseTarget(part.pose);
//        left_arm_group_.move();
//        auto state = getGripperState("left_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
//            currentPose.position.z =
            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
            ros::Duration(1.0).sleep(); // try to get it to lift before doing anything else!

            if (agv == "agv2")
                goToPresetLocation(agv2_);
            else if (agv == "agv1")
                goToPresetLocation(agv1_);

            PresetLocation start_a;
            start_a.gantry = {0, 0, 0};
            start_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
            start_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
            goToPresetLocation(start_a);
            ros::Duration(1.0).sleep(); // try to get it to actually go to start....
            deactivateGripper("left_arm");
            // goToPresetLocation(start_); // having this line here is great, but does not work for shelf, since path is impeded
            // move this to main code, but maybe let's keep the delay below?
            ros::Duration(1.0).sleep(); // try to get it to actually go to start....

            return true;
        }
        else
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt<max_attempts)
            {
                left_arm_group_.setPoseTarget(currentPose);
                left_arm_group_.move();
                ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
                activateGripper("left_arm");
                current_attempt++;
            }
            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
            ros::Duration(1.0).sleep(); // try to get it to lift before doing anything else! upped to 1.0 from 0.5
        }
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }

    // joint positions to go to start location
    PresetLocation start_a;
    start_a.gantry = {0, 0, 0};
    start_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    goToPresetLocation(start_a);
    deactivateGripper("left_gripper");
    ros::Duration(1.0).sleep();
//    auto state = getGripperState("left_arm");
    return false;
//    if (state.attached)
        // ;// pass, don't necesarily go back to start in case of faulty part
        //goToPresetLocation(start_);
}


////////////////////////////
void GantryControl::goToPresetLocation(PresetLocation location)
{
    ROS_INFO_STREAM("goToPresetLocation Executed!!!============");
    //--gantry
    joint_group_positions_.at(0) = location.gantry.at(0);
    joint_group_positions_.at(1) = location.gantry.at(1);
    joint_group_positions_.at(2) = location.gantry.at(2);
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);
    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

////////////////////////////
void GantryControl::activateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm")
    {
        left_gripper_control_client.call(srv);
    }
    else
    {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
void GantryControl::deactivateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm")
    {
        left_gripper_control_client.call(srv);
    }
    else
    {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name)
{
    if (arm_name == "left_arm")
    {
        return current_left_gripper_state_;
    }
    else
    {
        return current_right_gripper_state_;
    }
}

////////////////////////////
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

////////////////////////////
void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

////////////////////////////
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    if (joint_state_msg->position.size() == 0)
    {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}

////////////////////////////
void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

////////////////////////////
void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

////////////////////////////
void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}

////////////////////////////
bool GantryControl::sendJointPosition(trajectory_msgs::JointTrajectory command_msg)
{
    // ROS_INFO_STREAM("[gantry_control][sendJointPosition] called.");

    if (command_msg.points.size() == 0)
    {
        ROS_WARN("[gantry_control][sendJointPosition] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0, 4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0, 5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] right_arm command published!");
        return true;
    }
    else
    {
        return false;
    }
}
