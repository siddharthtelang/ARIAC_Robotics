#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>
#include <boost/bind.hpp>
#include <array>

#include "camera_listener.h"

#include <ros/ros.h>

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/Proximity.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef struct Shipment shipment; // forward declarations
typedef struct Order order;
typedef struct Product product;

const double PI = 3.141592; // TODO correct!

// Logical cameras
const int MAX_NUMBER_OF_CAMERAS = 17;



const int MAX_PICKING_ATTEMPTS = 3; // for pickup
const double ABOVE_TARGET = 0.2; // above target z pos when picking/placing part
const double PICK_TIMEOUT = 4.0;
const double RETRIEVE_TIMEOUT = 2.0;

const double BELT_SPEED = 0.2; // m/s

const double GRIPPER_HEIGHT = 0.01;
// const double EPSILON = 0.008; // for the gripper to firmly touch
// const double EPSILON = 0.007; // prevent mashing into and "deviated joint" error maybe
const double EPSILON = 0.0065; // prevent mashing into and "deviated joint" error maybe

const double BIN_HEIGHT = 0.724;
const double TRAY_HEIGHT = 0.755;
const double RAIL_HEIGHT = 0.95;

const double PLANNING_TIME = 20; // for move_group
const int MAX_EXCHANGE_ATTEMPTS = 6; // Pulley flip

extern std::string action_state_name[];
extern std::unordered_map<std::string, double> model_height;

enum PartStates {FREE, BOOKED, UNREACHABLE, ON_TRAY, GRIPPED, GOING_HOME,
  REMOVE_FROM_TRAY, LOST};


/**
 * @brief Struct for preset locations
 * @todo Add new preset locations here
 * 
 */
typedef struct PresetLocation {
    std::vector<double> gantry;
    std::vector<double> left_arm;
    std::vector<double> right_arm;
    std::string name; // a string, the exact same as the struct's variable name itself.
} start, bin3, agv2, agv1;

/**
 * @brief Struct to store part information
 * 
 */
typedef struct Part {
  std::string type; // model type
  geometry_msgs::Pose pose; // model pose (in frame)
  geometry_msgs::Pose initial_pose; // save the initial world pose on the bin/shelf/conveyor
  geometry_msgs::Pose target_pose; // save the final target pose in tray in world frame
  std::string frame; // model frame (e.g., "logical_camera_1_frame")
  ros::Time time_stamp;
  std::string id;
  bool faulty;
} part;

/**
 * @brief Struct to store joint positions for each group
 * 
 */
typedef struct Position {
    std::vector<double> gantry;
    std::vector<double> left;
    std::vector<double> right;
} position;

/**
 * @brief Struct to store shipments
 * 
 */
typedef struct Shipment {
    std::string shipment_type;
    std::string agv_id;
    std::vector<Product> products;
} shipment;

/**
 * @brief Struct to store products
 * 
 */
typedef struct Product {
    std::string type;
    geometry_msgs::Pose pose;
    part p; // NEW here!
    // std::string frame_of_origin;
    geometry_msgs::Pose actual_pose;
    std::string actual_pose_frame;
    std::string agv_id;
    std::string tray;
    std::string arm_name;
    std::string shipment_type;
    CameraListener::ModelInfo designated_model;
    bool get_from_conveyor{false};
} product;


/**
 * @brief struct to parse and store orders published on /ariac/orders
 * 
 */
typedef struct Order {
    std::string order_id;
    std::vector<Shipment> shipments;
} order;

class LaneBreakbeamPair {

public:
    void subscribePair(ros::NodeHandle& node, int br_closer, int br_further) {

        std::string br_topic_closer = "/ariac/breakbeam_" + std::to_string(br_closer) + "_change";
        std::string br_topic_further = "/ariac/breakbeam_" + std::to_string(br_further) + "_change";

        close_breakbeam_subscriber_ = node.subscribe<nist_gear::Proximity>(br_topic_closer, 10,
                                                                        boost::bind(&LaneBreakbeamPair::breakbeam_callback, this, _1, 0));

        far_breakbeam_subscriber_ = node.subscribe<nist_gear::Proximity>(br_topic_further, 10,
                                                                          boost::bind(&LaneBreakbeamPair::breakbeam_callback, this, _1, 1));
    }

    /**
     * @brief query pair of breakbeam sensors in a given lane
     * 
     */
    int queryPair() {
        return towards_conveyor_;
    }

    void breakbeam_callback(const nist_gear::Proximity::ConstPtr &msg, const int breakbeam);

    int prev_hit_ = -1;

private:
    ros::Subscriber close_breakbeam_subscriber_;
    ros::Subscriber far_breakbeam_subscriber_;

    std::array<double, 2> time_hit_{}; 
    int towards_conveyor_{-1}; 
    const int breakbeam_rate_{1};
    bool has_obstacle_{false};
};

class AllLanesHandler {

public:
    AllLanesHandler() : lanes_{} {}

    void setNode(ros::NodeHandle& node)
    {
        lanes_[LEFT].subscribePair(node, 7,8);
        lanes_[MID_LEFT].subscribePair(node, 5,6);
        lanes_[MID_RIGHT].subscribePair(node, 3,4);
        lanes_[RIGHT].subscribePair(node, 1,2);
    }

    std::array<int, 4> queryLanes();
    std::array<bool, 5> clearLanes();
    bool waitForOpening(int lane);
    enum LaneNumbers{LEFT, MID_LEFT, MID_RIGHT, RIGHT};

private:
    std::array<LaneBreakbeamPair, 4> lanes_{};
    
};

#endif