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

class CameraListenerClass
{
public:
  explicit CameraListenerClass(ros::NodeHandle &node)
      : tfBuffer(), tfListener(tfBuffer) // create buffer and listener once
  {
  }

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

      /* Start: Get TF of camera */
      geometry_msgs::TransformStamped transformStamped;
      if (msg->models.size()) {

        ros::Duration timeout(1.0);
        bool transform_detected = false;
        std::string cam_id = "logical_camera_" + std::to_string(cam_idx) + "_frame";

        while (!transform_detected) {
          try{
            transformStamped = tfBuffer.lookupTransform("world", cam_id,
                                     ros::Time(0), timeout);
            transform_detected = true;
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
          }
        }
      }
    //ROS_INFO_STREAM("transformStamped = "<< transformStamped);
    /* End: Get TF of camera */

    for (int i = 0; i < msg->models.size(); i++) {

      // ROS_INFO_STREAM("Model = " << msg->models[i].type << "\nPose = "
      //     << msg->models[i].pose << "\n");

      ModelInfo temp_model;
      temp_model.cam_index = cam_idx;
      temp_model.camera_pose = msg->pose;
      temp_model.model_pose = msg->models[i].pose;
      temp_model.transformStamped = transformStamped;

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

  // Function to perform transform without passing actual frame ids
  void performTransform(ModelInfo *model) {

    geometry_msgs::PoseStamped pose_target, pose_rel;
    std::string cam_id = "logical_camera_" + std::to_string(model->cam_index) + "_frame";
    pose_rel.header.frame_id = cam_id;
    pose_rel.pose = model->model_pose;

    tf2::doTransform(pose_rel, pose_target, model->transformStamped);
    (*model).world_pose.position.x = pose_target.pose.position.x;
    (*model).world_pose.position.y = pose_target.pose.position.y;
    (*model).world_pose.position.z = pose_target.pose.position.z;
    (*model).world_pose.orientation = pose_target.pose.orientation;
    //ROS_INFO_STREAM("transform  for  " << msg->models[i].type << " = " << pose_target);

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
          // Call the transform on model pose to world pose
          //get_world_pose(&model);
          performTransform(&model);
          
          tf2::Quaternion q(
             model.world_pose.orientation.x,
             model.world_pose.orientation.y,
             model.world_pose.orientation.z,
             model.world_pose.orientation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          ROS_INFO("Position(XYZ): [%f,%f,%f] || Orientation(RPY): [%f,%f,%f]",model.world_pose.position.x,
          model.world_pose.position.y,
          model.world_pose.position.z,
          roll,
          pitch,
          yaw);
          
        }
      }
    }
  }

private:
  std::array<std::vector<ModelInfo>, 16> camera_parts_list_; // ADD NUM CAMS
  tf2_ros::Buffer tfBuffer; // keep this in scope, buffer holds last 10 seconds of tf frames
  tf2_ros::TransformListener tfListener; // keep this in scope, and only create it once

};


int main(int argc, char **argv)
{
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  CameraListenerClass cam_listener(node);

  const int num_cams = 16; // ADD NUM CAMS

  std::array<ros::Subscriber, num_cams> logical_camera_subscriber{};
  for(int i = 0; i < num_cams; i++) {
      logical_camera_subscriber[i] = node.subscribe<nist_gear::LogicalCameraImage> (
        "/ariac/logical_camera_"+std::to_string(i), 10,
        boost::bind(&CameraListenerClass::logical_camera_callback, &cam_listener, _1, i));
  }

std::string fetch{""};
//   ros::Rate loop_rate(0.5); // update from cameras and print sorted list 0.5 times per second
  while(ros::ok()) {
    ros::param::get("/fetch_part", fetch);
    if (fetch == "fetch") {
      ROS_INFO("Param is set");
      ros::spinOnce(); // This executes callbacks on new data until ctrl-c.
      cam_listener.sort_camera_parts_list();
      ros::param::set("/fetch_part", "");
    }
    // loop_rate.sleep();
  }

  return 0;
}