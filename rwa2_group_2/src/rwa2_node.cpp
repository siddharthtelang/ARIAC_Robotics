// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <vector>
#include <string>
#include <array>
#include <unordered_map>

#include <ros/ros.h>

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
 * @brief Start the competition
 * Create a service client to /ariac/start_competition
 */
void start_competition(ros::NodeHandle &node)
{
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv); // Call the start Service.
  if (!srv.response.success)
  { // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("Competition started!");
  }
}

struct ModelInfo
{

  int cam_index;
  geometry_msgs::Pose camera_pose;

  std::string color;
  geometry_msgs::Pose model_pose;
  geometry_msgs::Pose world_pose;
  std::string type;
  std::string id;
};


/**
 * @brief A simple competition class.
 * 
 * This class can hold state and provide methods that handle incoming data.
 * 
 */
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle &node)
      : current_score_(0)
  {
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/current_score
   * 
   * This function sets the value of the attribute current_score_ 
   * @param msg Message used to set the state of the competition.
   */
  void current_score_callback(const std_msgs::Float32::ConstPtr &msg)
  {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /**
   * @brief Called when a new Message is received on /ariac/competition_state
   * 
   * This function sets the state of the competition to 'done'.
   * @param msg Message used to set the state of the competition.
   */
  void competition_state_callback(const std_msgs::String::ConstPtr &msg)
  {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/orders
   * 
   * This function adds the Message to received_orders_
   * 
   * @param msg Message containing information on the order.
   */
  void order_callback(const nist_gear::Order::ConstPtr &msg)
  {
    ROS_INFO_STREAM("Received order:\n" << *msg);
    received_orders_.push_back(*msg);
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/logical_camera_x
   * 
   * This function reports the number of objects detected by a logical camera.
   * 
   * @param msg Message containing information on objects detected by the camera.
   */
  // void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg, int cam_idx){
  //   ROS_INFO_STREAM(cam_idx);
  // }

  void logical_camera_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg, int cam_idx)
  {
    // ROS_INFO_STREAM("================== Logical camera " << cam_idx << " ==================");
    // ROS_INFO_STREAM("Logical camera: '" << msg->models.size());

    std::unordered_map<std::string, int> part_count;
    camera_parts_list_[cam_idx].clear();

    for (int i = 0; i < msg->models.size(); i++) {

      // ROS_INFO_STREAM("Model = " << msg->models[i].type << "\nPose = "
      //     << msg->models[i].pose << "\n");

      ModelInfo temp_model;
      temp_model.cam_index = cam_idx;
      temp_model.camera_pose = msg->pose;
      temp_model.model_pose = msg->models[i].pose;

      std::string color = msg->models[i].type;
      std::string delimiter = "_";

      temp_model.type = color.substr(0, color.find(delimiter));

      if(part_count.find(msg->models[i].type) == part_count.end()) {
        part_count[msg->models[i].type] = 1;
      } else {
        part_count[msg->models[i].type]++;
      }
      int frame_number_to_append{part_count[msg->models[i].type]};

      int pos{};
    
      while ((pos = color.find(delimiter)) != std::string::npos) {
         color.erase(0,pos+delimiter.length());
      }
      temp_model.color = color;
            
      temp_model.id = "logical_camera_" + std::to_string(cam_idx) + "_" + msg->models[i].type + "_" + std::to_string(frame_number_to_append) + "_frame";
      camera_parts_list_[cam_idx].push_back(temp_model); // add copy of our struct to array of vectors 
      // ROS_INFO_STREAM(temp_model.id);
    }
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/break_beam_x_change
   * 
   * This function reports when an object crossed the beam.
   * 
   * @param msg Message of Boolean type returning true if an object crossed the beam.
   */
  void break_beam_callback(const nist_gear::Proximity::ConstPtr &msg)
  {
    if (msg->object_detected) // If there is an object in proximity.
      ROS_WARN("Break beam triggered.");
  }

  void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr &msg)
  {
    if ((msg->max_range - msg->range) > 0.01)
    { // If there is an object in proximity.
      ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
    }
  }

  void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    size_t number_of_valid_ranges = std::count_if(
        msg->ranges.begin(), msg->ranges.end(), [](const float f) { return std::isfinite(f); });
    if (number_of_valid_ranges > 0)
    {
      ROS_INFO_THROTTLE(5, "Laser profiler sees something.");
    }
  }

  void sort_camera_parts_list() {

    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ModelInfo>>> ordered_color_type;
    for(auto row: camera_parts_list_) {
      for(auto model: row) {
        ordered_color_type[model.color][model.type].push_back(model);
      }
    }

    std::array<std::string, 3> colors{"red", "blue", "green"};
    std::array<std::string, 4> types{"disk", "pulley", "gasket", "piston"};

    for(auto color: colors) {
      for(auto type: types) {
        ROS_INFO_STREAM("\n====================== " << color << " " << type << " ======================");
        for (auto model : ordered_color_type[color][type]) {
          ROS_INFO_STREAM(model.id);
          ROS_INFO_STREAM(model.world_pose); // THIS MUST BE POPULATED BEFORE IT CAN RUN!
        }
      }
    }
  }

private:
  std::string competition_state_;
  double current_score_;
  std::vector<nist_gear::Order> received_orders_;
  std::array<std::vector<ModelInfo>, 7> camera_parts_list_; // ADD NUM CAMS

  // const nist_gear::LogicalCameraImage camera_msg_;
  // std::vector<nist_gear::Model_<std::allocator<void>>, std::allocator<nist_gear::Model_<std::allocator<void>>>> = camera_msg_;
  // const nist_gear::LogicalCameraImage::Ptr camera_msg_;


};

int main(int argc, char **argv)
{
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' Topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
      "/ariac/current_score", 10,
      &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' Topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
      "/ariac/competition_state", 10,
      &MyCompetitionClass::competition_state_callback, &comp_class);

  // Subscribe to the '/ariac/orders' Topic.
  ros::Subscriber orders_subscriber = node.subscribe(
      "/ariac/orders", 10,
      &MyCompetitionClass::order_callback, &comp_class);

  // Subscribe to the '/ariac/range_finder_0' Topic.
  //ros::Subscriber proximity_sensor_subscriber = node.subscribe(
  //    "/ariac/range_finder_0", 
  //    10, 
  //    &MyCompetitionClass::proximity_sensor_callback,
  //    &comp_class);


  // Subscribe to the '/ariac/breakbeam_0_change' Topic.
  ros::Subscriber break_beam_subscriber = node.subscribe(
      "/ariac/breakbeam_0_change", 10,
      &MyCompetitionClass::break_beam_callback,
      &comp_class);

  // Subscribe to the '/ariac/logical_camera_12' Topic.
  // ros::Subscriber logical_camera_subscriber = node.subscribe(
  //     "/ariac/logical_camera_12", 10,
  //     &MyCompetitionClass::logical_camera_callback, &comp_class);

  const int num_cams = 7; // ADD NUM CAMS
  std::array<ros::Subscriber, num_cams> logical_camera_subscriber{};
  for(int i = 0; i < num_cams; i++) {
      logical_camera_subscriber[i] = node.subscribe<nist_gear::LogicalCameraImage> (
        "/ariac/logical_camera_"+std::to_string(i), 10,
        boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, i));
  }

  // Subscribe to the '/ariac/laser_profiler_0' Topic.
  // ros::Subscriber laser_profiler_subscriber = node.subscribe(
  //    "/ariac/laser_profiler_0", 10, &MyCompetitionClass::laser_profiler_callback, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);
  while(ros::ok()) {
    comp_class.sort_camera_parts_list();
    ros::spinOnce(); // This executes callbacks on new data until ctrl-c.
  }
  
  return 0;
}

