#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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

    // joint positions to go to agv2_a, for right arm
    agv2_a.gantry = {-0.6, 6.9, PI};
    agv2_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv1
    agv1_.gantry = {0.6, -6.9, PI};
    agv1_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv1_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv1_a, for right arm
    agv1_a.gantry = {-0.6, -6.9, PI};
    agv1_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv1_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

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
geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target, std::string agv)
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

    for (int i = 0; i < 15; ++i)
    {
        br.sendTransform(transformStamped);
    }

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(1.0);

    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;

    for (int i = 0; i < 10; i++)
    {
        try
        {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
    // world_target.position.x = world_target_tf.transform.translation.x;
    // world_target.position.y = world_target_tf.transform.translation.y;
    // world_target.position.z = world_target_tf.transform.translation.z;
    // world_target.orientation.x = ee_target_tf.transform.rotation.x;
    // world_target.orientation.y = ee_target_tf.transform.rotation.y;
    // world_target.orientation.z = ee_target_tf.transform.rotation.z;
    // world_target.orientation.w = ee_target_tf.transform.rotation.w;

    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = world_target_tf.transform.rotation.x;
    world_target.orientation.y = world_target_tf.transform.rotation.y;
    world_target.orientation.z = world_target_tf.transform.rotation.z;
    world_target.orientation.w = world_target_tf.transform.rotation.w;

    return world_target;
}

////////////////////////////
bool GantryControl::pickPart(part part, std::string arm)
{
    moveit::planning_interface::MoveGroupInterface* temp_arm_group;
    if (arm == "left_arm") temp_arm_group = &left_arm_group_;
    else if (arm == "right_arm") temp_arm_group = &right_arm_group_;
    else {
        ROS_INFO_STREAM("Please select arm to use.");
        throw 1;
    }

    //--Activate gripper
    activateGripper(arm);
    geometry_msgs::Pose currentPose = temp_arm_group->getCurrentPose().pose;

    // update the z if the part if flipped
    double roll, pitch, yaw;
    tf2::Quaternion q_current(
        part.pose.orientation.x,
        part.pose.orientation.y,
        part.pose.orientation.z,
        part.pose.orientation.w);
    tf2::Matrix3x3 m(q_current);
    m.getRPY(roll, pitch, yaw);
    if (std::abs(roll) > 3.0 && part.type.find("pulley") == 0)
        part.pose.position.z = part.pose.position.z + GRIPPER_HEIGHT - EPSILON;
    else
        part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;

    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
    //    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);

    auto state = getGripperState(arm);
    //keep trying till state is enabled
    while (!state.enabled)
    {
        activateGripper(arm);
        state = getGripperState(arm);
    }
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        temp_arm_group->setPoseTarget(part.pose);
        temp_arm_group->move();
        auto state = getGripperState(arm);
        ros::Duration(0.5).sleep(); // Commented for speed Human Obstacles
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
            temp_arm_group->setPoseTarget(currentPose);
            temp_arm_group->move();
            ros::Duration(0.5).sleep(); // try to get it to lift before doing anything else! // Commented for speed Human Obstacles

            // goToPresetLocation(start_); // having this line here is great, but does not work for shelf, since path is impeded
            // move this to main code, but maybe let's keep the delay below?
            ros::Duration(1.0).sleep(); // try to get it to actually go to start....
            return true;
        }
        else
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            //modify max attempts to 2 from 5
            int max_attempts{2};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt<max_attempts)
            {
                temp_arm_group->setPoseTarget(currentPose);
                temp_arm_group->move();
                ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
                temp_arm_group->setPoseTarget(part.pose);
                temp_arm_group->move();
                ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
                activateGripper(arm);
                current_attempt++;
                state = getGripperState(arm);
                if (state.attached) return true;
            }
            temp_arm_group->setPoseTarget(currentPose);
            temp_arm_group->move();
            ros::Duration(1.0).sleep(); // try to get it to lift before doing anything else! upped to 1.0 from 0.5
        }
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}

////////////////////////////
void GantryControl::placePart(part part, std::string agv, std::string arm)
{
    moveit::planning_interface::MoveGroupInterface *temp_arm_group;
    if (arm == "left_arm")
        temp_arm_group = &left_arm_group_;
    else if (arm == "right_arm")
        temp_arm_group = &right_arm_group_;
    else
    {
        ROS_INFO_STREAM("Please select arm to use.");
        throw 1;
    }

    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);

    ros::Duration(.5).sleep();

    if (agv == "agv2")
    {
        if (arm == "right_arm")
            goToPresetLocation(agv2_a);
        else
            goToPresetLocation(agv2_);
    }
    else if (agv == "agv1")
    {
        if (arm == "right_arm")
            goToPresetLocation(agv1_a);
        else
            goToPresetLocation(agv1_);
    }

    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);

    /* Start: Set the correct orientation */

    tf2::Quaternion q_pitch(0, 0.7071068, 0, 0.7071068); // Gantry has pi/2 in its end effector
    tf2::Quaternion q_pi(0, 0, 1, 0);                    // Gantry always moves by 180 degrees from start to agv
    //set the initial pose
    tf2::Quaternion q_init_part(part.initial_pose.orientation.x,
                                part.initial_pose.orientation.y,
                                part.initial_pose.orientation.z,
                                part.initial_pose.orientation.w);
    //set the final pose
    tf2::Quaternion q_final_part(target_pose_in_tray.orientation.x,
                                 target_pose_in_tray.orientation.y,
                                 target_pose_in_tray.orientation.z,
                                 target_pose_in_tray.orientation.w);
    //calculate the resultant rotation
    tf2::Quaternion q_rslt = q_init_part.inverse() * q_final_part * q_pi * q_pitch;
    target_pose_in_tray.orientation.x = q_rslt.x();
    target_pose_in_tray.orientation.y = q_rslt.y();
    target_pose_in_tray.orientation.z = q_rslt.z();
    target_pose_in_tray.orientation.w = q_rslt.w();

    /* End: Set the correct orientation */

    /* Start: Fix Bug - When there is a faulty gripper, gantry tries to place part on tray without any part on it */
    auto state = getGripperState(arm);
    if (state.attached)
    {
        temp_arm_group->setPoseTarget(target_pose_in_tray);
        temp_arm_group->move();
        state = getGripperState(arm);
        if (state.attached)
        {
            deactivateGripper(arm);
        }
    }
    else
    {
        ROS_INFO("Object is not attached to gripper, do not move");
    }
    /* End: Fix Bug - When there is a faulty gripper, gantry tries to place part on tray without any part on it */

    //auto state = getGripperState(arm);
    //if (state.attached)
    // pass, don't necesarily go back to start in case of faulty part
    //goToPresetLocation(start_);
    ROS_INFO("Place OK");
}

////////////////////////////
void GantryControl::placePartAtCorrectPose(part part, std::string agv, std::string arm)
{
    moveit::planning_interface::MoveGroupInterface* temp_arm_group;
    if (arm == "left_arm") temp_arm_group = &left_arm_group_;
    else if (arm == "right_arm") temp_arm_group = &right_arm_group_;
    else {
        ROS_INFO_STREAM("Please select arm to use.");
        throw 1;
    }

    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);

    //ros::Duration(2.0).sleep();

    if (agv == "agv2")
    {
        if (arm == "right_arm")
            goToPresetLocation(agv2_a);
        else
            goToPresetLocation(agv2_);
    }
    else if (agv == "agv1")
    {
        if (arm == "right_arm")
            goToPresetLocation(agv1_a);
        else
            goToPresetLocation(agv1_);
    }

    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);

    /* Start: Set the correct orientation */

    tf2::Quaternion q_pitch(0, 0.7071068, 0, 0.7071068); // Gantry has pi/2 in its end effector
    tf2::Quaternion q_pi(0, 0, 1, 0); // Gantry always moves by 180 degrees from start to agv
    //set the initial pose
    tf2::Quaternion q_init_part(part.initial_pose.orientation.x,
                                part.initial_pose.orientation.y,
                                part.initial_pose.orientation.z,
                                part.initial_pose.orientation.w);

    tf2::Matrix3x3 m(q_init_part);
    tf2::Quaternion new_init_orientation;
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (std::abs(roll) > 3.0)
    {
        roll = 0.0;
        new_init_orientation.setRPY(roll, pitch, yaw);
        new_init_orientation.normalize();
        q_init_part = new_init_orientation;
    }

    //set the final pose
    tf2::Quaternion q_final_part(target_pose_in_tray.orientation.x,
                                target_pose_in_tray.orientation.y,
                                target_pose_in_tray.orientation.z,
                                target_pose_in_tray.orientation.w);
    //calculate the resultant rotation
    tf2::Quaternion q_rslt = q_init_part.inverse() * q_final_part * q_pitch;
    target_pose_in_tray.orientation.x = q_rslt.x();
    target_pose_in_tray.orientation.y = q_rslt.y();
    target_pose_in_tray.orientation.z = q_rslt.z();
    target_pose_in_tray.orientation.w = q_rslt.w();

    /* End: Set the correct orientation */

    /* Start: Fix Bug - When there is a faulty gripper, gantry tries to place part on tray without any part on it */
    auto state = getGripperState(arm);
    if (state.attached)
    {
        temp_arm_group->setPoseTarget(target_pose_in_tray);
        temp_arm_group->move();
        state = getGripperState(arm);
        if (state.attached){
            deactivateGripper(arm);
        }
    }
    else
    {
        ROS_INFO("Object is not attached to gripper, do not move");
    }
    /* End: Fix Bug - When there is a faulty gripper, gantry tries to place part on tray without any part on it */
    ROS_INFO("Place OK");
}

bool GantryControl::replaceFaultyPart(part Part, std::string agv, std::string arm)
{
    auto target_pose_in_tray = getTargetWorldPose(Part.pose, agv);
    part partToPick = Part;
    partToPick.pose = target_pose_in_tray;
    bool success = pickPart(partToPick, arm);
    if (success)
    {
        ROS_INFO("Pick faulty part success, now proceed to throw");
	goToPresetLocation(agv == "agv1" ? agv1_ : agv2_);
        goToPresetLocation(start_);
        ROS_INFO("Drop Part Now");
        deactivateGripper(arm);
    }
    return success;
}

////////////////////////////
void GantryControl::goToPresetLocation(PresetLocation location)
{
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
        // ros::Duration(1.0).sleep(); // Commented for speed Human Obstacles
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

bool GantryControl::flipPart(part part, std::string agv)
{
    ROS_INFO("now proceed to flip");
    if (agv == "agv2")
        goToPresetLocation(agv2_);
    else if (agv == "agv1")
        goToPresetLocation(agv1_);
    PresetLocation pose_change_1_agv1, pose_change_2_agv1, pose_change_1_agv2, pose_change_2_agv2;
    pose_change_1_agv1.gantry = { 0.0, -6.9, PI };
    pose_change_1_agv1.left_arm = { PI / 4, -0.2, 1.3, 0.5, PI / 2, 0.00 };
    pose_change_1_agv1.right_arm = { -PI / 4, -3, -PI / 2, -0.1, PI / 2, -0.79 };
    // switching waypoint
    pose_change_2_agv1.gantry = { 0.0, -6.9, PI };
    pose_change_2_agv1.left_arm = { 0.77, -0.2, 1.3, 0.49, 1.59, 0.00 };
    pose_change_2_agv1.right_arm = { -PI / 4, -3.2, -1.5, -0.02, PI / 2, -PI / 4 };
    // pose change waypoint
    pose_change_1_agv2.gantry = { 0.0, 5, PI };
    pose_change_1_agv2.left_arm = { PI / 4, -0.2, 1.3, 0.5, PI / 2, 0.00 };
    pose_change_1_agv2.right_arm = { -PI / 4, -3, -PI / 2, -0.1, PI / 2, -0.79 };
    // switching waypoint
    pose_change_2_agv2.gantry = { 0.0, 5, PI };
    pose_change_2_agv2.left_arm = { 0.77, -0.2, 1.3, 0.49, 1.59, 0.00 };
    pose_change_2_agv2.right_arm = { -PI / 4, -3.2, -1.5, -0.02, PI / 2, -PI / 4 };
    // set the arms target pose
    //geometry_msgs::Pose target_pose = getTargetWorldPose(part.pose, agv);
    if (agv == "agv1") {
        goToPresetLocation(pose_change_1_agv1);
        goToPresetLocation(pose_change_2_agv1);
    }
    else {
        ROS_INFO("Pose Change 1");
        goToPresetLocation(pose_change_1_agv2);
        ROS_INFO("Pose Change 2");
        goToPresetLocation(pose_change_2_agv2);
    }
    activateGripper("right_arm");
    auto state = getGripperState("right_arm");

    int max_attempts{10};
    int current_attempt{0};
    while (!state.attached && current_attempt < max_attempts)
    {
        ROS_INFO("Tring to activate right gripper");
        activateGripper("right_arm");
        state = getGripperState("right_arm");
        ros::Duration(0.5).sleep();
        current_attempt++;
    }
    ros::Duration(0.5).sleep();
    deactivateGripper("left_arm");
}
