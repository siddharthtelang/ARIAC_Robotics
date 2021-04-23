#include "camera_listener.h"

void CameraListener::logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int cam_idx)
// void CameraListener::logical_camera_callback(
//     const nist_gear::LogicalCameraImage::ConstPtr &msg)
{

  std::unordered_map<std::string, int> part_count;
  camera_parts_list_[cam_idx].clear();
  if (msg->models.size())
  {
    // ROS_INFO_STREAM("================== Logical camera " << cam_idx << " ==================");
    // ROS_INFO_STREAM("Logical camera: '" << msg->models.size());

    /* Start: Get TF of camera */
    geometry_msgs::TransformStamped transformStamped;
    if (msg->models.size())
    {

      ros::Duration timeout(1.0);
      bool transform_detected = false;
      std::string cam_id = "logical_camera_" + std::to_string(cam_idx) + "_frame";

      while (!transform_detected)
      {
        try
        {
          transformStamped = tfBuffer.lookupTransform("world", cam_id,
                                                      ros::Time(0), timeout);
          transform_detected = true;
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
      }
    }
    //ROS_INFO_STREAM("transformStamped = "<< transformStamped);
    /* End: Get TF of camera */

    for (int i = 0; i < msg->models.size(); i++)
    {

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
      if (temp_model.type == "piston")
          temp_model.type = "piston_rod";

      if (part_count.find(msg->models[i].type) == part_count.end())
      {
        part_count[msg->models[i].type] = 1;
      }
      else
      {
        part_count[msg->models[i].type]++;
      }
      int frame_number_to_append{part_count[msg->models[i].type]};

      int pos{};

      while ((pos = color.find(delimiter)) != std::string::npos)
      {
        color.erase(0, pos + delimiter.length());
      }
      temp_model.color = color;

      //perform TF
      performTransform(&temp_model);

      temp_model.id = "logical_camera_" + std::to_string(cam_idx) + "_" + msg->models[i].type + "_" + std::to_string(frame_number_to_append) + "_frame";
      camera_parts_list_[cam_idx].push_back(temp_model); // add copy of our struct to array of vectors
      // ROS_INFO_STREAM(temp_model.id);
    }
  }
}

// Function to perform transform without passing actual frame ids
void CameraListener::performTransform(ModelInfo *model)
{

  geometry_msgs::PoseStamped pose_target, pose_rel;
  std::string cam_id = "logical_camera_" + std::to_string(model->cam_index) + "_frame";
  pose_rel.header.frame_id = cam_id;
  pose_rel.pose = model->model_pose;

  tf2::doTransform(pose_rel, pose_target, model->transformStamped);
  (*model).world_pose.position.x = pose_target.pose.position.x;
  (*model).world_pose.position.y = pose_target.pose.position.y;
  (*model).world_pose.position.z = pose_target.pose.position.z;
  (*model).world_pose.orientation = pose_target.pose.orientation;
}

void CameraListener::sort_camera_parts_list()
{

  //std::unordered_map<std::string, std::unordered_map<std::string, std::vector<ModelInfo>>> ordered_color_type;
  ordered_color_type.clear();

  for (auto row : camera_parts_list_)
  {
    for (auto model : row)
    {
      ordered_color_type[model.color][model.type].push_back(model);
    }
  }

  for (auto color : colors_)
  {
    for (auto type : types_)
    {
      // ROS_INFO_STREAM("\n====================== " << color << " " << type << " ======================");
      ROS_INFO_STREAM(ordered_color_type[color][type].size() << " " << color << " " << type << " in environment.");
      for (auto model : ordered_color_type[color][type])
      {
        // ROS_INFO_STREAM(model.id);

        tf2::Quaternion q(
            model.world_pose.orientation.x,
            model.world_pose.orientation.y,
            model.world_pose.orientation.z,
            model.world_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // ROS_INFO("Position(XYZ): [%f,%f,%f] || Orientation(RPY): [%f,%f,%f]",
        //          model.world_pose.position.x,
        //          model.world_pose.position.y,
        //          model.world_pose.position.z,
        //          roll,
        //          pitch,
        //          yaw);
      }
    }
  }
}

std::unordered_map<std::string, std::unordered_map<std::string, std::priority_queue<CameraListener::ModelInfo, std::vector<CameraListener::ModelInfo>, CameraListener::CompareDists>>> CameraListener::sortPartsByDist()
{
  // priority_queue<Person, vector<Person>, CompareAge>
  std::unordered_map<std::string, std::unordered_map<std::string, std::priority_queue<CameraListener::ModelInfo, std::vector<CameraListener::ModelInfo>, CameraListener::CompareDists>>> sorted_by_dist;
  for (auto color : colors_)
  {
    for (auto type : types_)
    {
      for (auto model : ordered_color_type[color][type])
      {
        sorted_by_dist[color][type].push(model);
      }
    }
  }

  return sorted_by_dist;
}

CameraListener::CameraListener(ros::NodeHandle &node)
    : tfBuffer(), tfListener(tfBuffer)
{
  node_ = node;
}

//int main(int argc, char **argv)
std::array<std::vector<CameraListener::ModelInfo>, 17> CameraListener::fetchParts(ros::NodeHandle &node)
{

  const int num_cams = 17; // ADD NUM CAMS
  ROS_INFO("Subscribing for camera");

  std::array<ros::Subscriber, num_cams> logical_camera_subscriber{};
  for (int i = 0; i < num_cams; i++)
  {
    logical_camera_subscriber[i] = node.subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_" + std::to_string(i), 10,
        boost::bind(&CameraListener::logical_camera_callback, this, _1, i));
  }

  // ROS_INFO("Param is set. Query the logical cameras");
  ros::Duration(0.2).sleep();
  //  CameraListener::sort_camera_parts_list();

  return camera_parts_list_;
}

//std::vector<CameraListener::ModelInfo> CameraListener::checkFaulty(ros::NodeHandle &node,std::string agv_id)
//bool CameraListener::checkFaulty(ros::NodeHandle &node,std::string agv_id)
bool CameraListener::checkFaulty(ros::NodeHandle &node, std::string agv_id)
{
  queried = false;
  int q_sensor = 1;
  ROS_INFO("Subscribing to Quality Sensor above %s", agv_id.c_str());
  if (agv_id == "agv1")
  {
    q_sensor = 2;
  }
  else
  {
    q_sensor = 1;
  }
  faulty_parts = false;
  ros::Subscriber quality_sensor_subscriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/quality_control_sensor_" + std::to_string(q_sensor), 1000, boost::bind(&CameraListener::quality_control_callback, this, _1, q_sensor));
  ROS_INFO("Param is set. Query the Quality Control camera");
  //    ros::spinOnce();
  ros::Duration(1.2).sleep();
  return queried;
}

void CameraListener::quality_control_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int q_sensor)
{ 
  queried = true;
  std::string sensor_tf_frame = "quality_control_sensor_" + std::to_string(q_sensor) + "_frame";
  if (msg->models.size() > 0)
  { faulty_parts_list.clear();
    ROS_INFO("Detected faulty part(s)! ");
    faulty_parts = true;
    for (auto model : msg->models)
    {
      geometry_msgs::Pose model_pose = model.pose;
      geometry_msgs::TransformStamped transformStamped;
      //            tf2_ros::Buffer tfBuffer;
      //            tf2_ros::TransformListener tfListener(tfBuffer);
      ros::Duration timeout(1.0);
      bool transform_exists = tfBuffer.canTransform("world", sensor_tf_frame, ros::Time(0), timeout);
      if (transform_exists)
        transformStamped = tfBuffer.lookupTransform("world", sensor_tf_frame, ros::Time(0));
      else
      {
        ROS_INFO("Cannot trans  faulty_parts_list.push_back(temp_model);form from %s to world", sensor_tf_frame.c_str());
        break;
      }
      ModelInfo temp_model;
      temp_model.cam_index = -q_sensor; // it collides with cam_index but we do not use it to so it works
      temp_model.camera_pose = msg->pose;
      temp_model.model_pose = model.pose;
      temp_model.transformStamped = transformStamped;

      geometry_msgs::PoseStamped pose_target, pose_rel;
      pose_rel.header.seq = 1;
      pose_rel.header.frame_id = sensor_tf_frame;
      pose_rel.header.stamp = ros::Time(0);
      pose_rel.pose = model.pose;

      tf2::doTransform(pose_rel, pose_target, transformStamped);
      temp_model.world_pose.position.x = pose_target.pose.position.x;
      temp_model.world_pose.position.y = pose_target.pose.position.y;
      temp_model.world_pose.position.z = pose_target.pose.position.z;
      temp_model.world_pose.orientation = pose_target.pose.orientation;

      // TODO:get the model color and type to be replaced
      faulty_parts_list.push_back(temp_model);
      ROS_INFO("Detected faulty part added to the list");
    }
  }
  else
  {
    faulty_parts = false;
  }
  ROS_INFO_STREAM("faulty parts: " << faulty_parts);
}

void CameraListener::breakbeam_callback(const nist_gear::Proximity::ConstPtr &msg)
{
  if (msg->object_detected == true)
  {
    ros::Time t_hit_breakbeam = ros::Time::now();
    load_time_on_conveyor_.push(t_hit_breakbeam);
    auto cam_parts = fetchPartsFromCamera(node_, 5);
    if (!cam_parts.empty())
        parts_on_conveyor_.push_back(cam_parts[0]);

    // ROS_INFO_STREAM("\n=========================================\n"
    // << "Objected detected from breakbeam! Time: " << t_hit_breakbeam << "\n=========================================\n");
  }
}

//Function to fetch parts from specific camera
std::vector<CameraListener::ModelInfo> CameraListener::fetchPartsFromCamera(ros::NodeHandle &node, int cam_idx)
{
  camera_parts_list_[cam_idx].clear();
  ROS_INFO_STREAM("Subscribing only for camera " << cam_idx);

  ros::Subscriber logical_camera_subscriber;
  logical_camera_subscriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_" + std::to_string(cam_idx), 10,
      boost::bind(&CameraListener::logical_camera_callback, this, _1, cam_idx));

  ros::Duration(0.2).sleep();

  return camera_parts_list_[cam_idx];
}
