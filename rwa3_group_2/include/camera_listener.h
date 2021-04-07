#include <algorithm>
#include <vector>
#include <string>
#include <array>
#include <unordered_map>

#include <ros/ros.h>
#include <boost/bind.hpp>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3


/**
 * \brief: Defines class for camera ROS interactions
 * \param: reference to ROS Node Handle
 * \result: object
 */
class CameraListener
{
public:
  explicit CameraListener(ros::NodeHandle &node);
//       : tfBuffer(), tfListener(tfBuffer) // create buffer and listener once
//   {
//       node_ = node;
//   }

struct ModelInfo
{

  int cam_index;
  geometry_msgs::Pose camera_pose;

  std::string color;
  geometry_msgs::Pose model_pose;
  geometry_msgs::Pose world_pose;
  geometry_msgs:: TransformStamped transformStamped;
  std::string type;
  std::string id;
};

   /**
  * \brief: Defines camera subscriber callback
  * \param: pointer to ROS message
  * \param: camera index
  * \result: Adds models to camera parts list
  */
  void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int cam_idx);

   /**
  * \brief: Function to perform transform without passing actual frame ids
  * \param: Individual model
  * \result: Updated model pose
  */
  void performTransform(ModelInfo *model);

  /**
  * \brief: Sorts the camera parts by type and color
  * \result: Updates hash map with ordered parts
  */
  void sort_camera_parts_list();

  /**
  * \brief: Checks AGV for faulty parts
  * \param: reference to ROS node handle
  * \param: agv id
  * \result: Applies callback function below
  */
  void checkFaulty(ros::NodeHandle &node,std::string agv_id);

  /**
  * \brief: Callback function for AGV faulty part check
  * \param: pointer to ROS msg
  * \param: sensor index
  * \result: Updated faulty part list
  */
  void quality_control_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int q_sensor);

  /**
  * \brief: Getter for camera part list after querying all cams
  * \param: reference to ROS node handle
  * \result: returns camera part list
  */
  std::array<std::vector<ModelInfo>,17> fetchParts(ros::NodeHandle &node);
  ros::NodeHandle node_;

  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ModelInfo>>> getPartsMap(){
      return ordered_color_type;
  }

  std::vector<ModelInfo> faulty_parts_list;  // to check if there are faulty parts in either tray
  bool faulty_parts;  // to check if there are faulty parts in either tray
  std::array<std::vector<ModelInfo>, 17> camera_parts_list_; // ADD NUM CAMS
  tf2_ros::Buffer tfBuffer; // keep this in scope, buffer holds last 10 seconds of tf frames
  tf2_ros::TransformListener tfListener; // keep this in scope, and only create it once
  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ModelInfo>>> ordered_color_type;

  //ros::Subscriber logical_camera_subscriber;

};
