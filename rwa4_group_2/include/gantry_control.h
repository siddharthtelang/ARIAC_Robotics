#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"


/**
 * \brief: Controlling for gantry movement
 * \param: reference Ros Node Handle
 * \result: 1 or 0 whether part is picked
 */
class GantryControl {

  public:
    GantryControl(ros::NodeHandle & node);
    /**
    * \brief: Create moveit joint groups, subscriptions and publishers
    */
    void init();

    /**
     * \brief: Returns true or false depending on whether part was successfully picked.
     * \param: part to pick
     * \result: true false success boolean
     */
    bool pickPart(part part, std::string arm);
    
    /**
     * \brief: Returns true or false depending on whether part was successfully placed.
     * \param: part to place
     * \result: true false success boolean
     */
    void placePart(part part, std::string agv, std::string arm);

    void placePartAtCorrectPose(part part, std::string agv, std::string arm);

    bool replaceFaultyPart(part part, std::string agv, std::string arm);


    bool flipPart(part part, std::string agv);

    
    /**
     * \brief: Send command message to robot controller
     * \param: trajectory message
     * \result: true or false success boolean
     */
    bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
    
    /**
     * \brief: goes to location
     * \param: location to move to
     * \result: Moves gantry to location
     */
    void goToPresetLocation(PresetLocation location);

    /**
     * \brief: activates vacuum gripper
     * \param: gripper's id
     * \result: picks up object
     */
    void activateGripper(std::string gripper_id);

    /**
     * \brief: deactivates vacuum gripper
     * \param: gripper's id
     * \result: releases object
     */
    void deactivateGripper(std::string gripper_id);
    
    /**
     * \brief: Getter for gripper state
     * \param: arm_name
     * \result: returns gripper state
     */
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    
    /**
     * \brief: Getter for world pose
     * \param: target
     * \param: agv
     * \result: returns world pose
     */
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
    //--preset locations;
    start start_;
    bin3 bin3_;
    agv2 agv2_; // given
    agv1 agv1_; // the one closer to us

    geometry_msgs::Pose getGantryPose()
    {
      ROS_INFO_STREAM("insde getGantryPose !");
      // return full_robot_group_.getCurrentPose().pose;
      return left_arm_group_.getCurrentPose().pose; // for some reason cannot get full robot group's current pose, no end effector specified
    }

    std::vector<double> getGantryJointPositionsDoubleVector()
    {

      //--Raw pointers are frequently used to refer to the planning group for improved performance.
      //--To start, we will create a pointer that references the current robot’s state.
      const moveit::core::JointModelGroup *joint_model_group =
          full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

      //--Let’s set a joint space goal and move towards it.
      moveit::core::RobotStatePtr current_state_a = full_robot_group_.getCurrentState();

      std::vector<double> joint_group_positions;
      current_state_a->copyJointGroupPositions(joint_model_group, joint_group_positions);

      return joint_group_positions;
    }

  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);

};

#endif
