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

  void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int cam_idx);
  void performTransform(ModelInfo *model);
  void sort_camera_parts_list();
  std::array<std::vector<ModelInfo>,16> fetchParts(ros::NodeHandle &node);
  ros::NodeHandle node_;

  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ModelInfo>>> getPartsMap(){
      return ordered_color_type;
  }

  std::array<std::vector<ModelInfo>, 16> camera_parts_list_; // ADD NUM CAMS
  tf2_ros::Buffer tfBuffer; // keep this in scope, buffer holds last 10 seconds of tf frames
  tf2_ros::TransformListener tfListener; // keep this in scope, and only create it once
  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ModelInfo>>> ordered_color_type;

  //ros::Subscriber logical_camera_subscriber;

};