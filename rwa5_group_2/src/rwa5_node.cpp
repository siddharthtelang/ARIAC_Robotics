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
#include <deque>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "agv_control.h"
#include "camera_listener.h"
#include "rwa_implementation.h"

#include <tf2/LinearMath/Quaternion.h>

#include <unordered_map>
#include <vector>
#define GET_VARIABLE_NAME(Variable) (#Variable)


template <typename Container> // we can make this generic for any container [1]
struct container_hash {
    std::size_t operator()(Container const& c) const {
        return boost::hash_range(c.begin(), c.end());
    }
};


int main(int argc, char ** argv) {

    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    CameraListener cam_listener(node);
    AGVControl agv_control(node);
    RWAImplementation rwa(node, cam_listener, gantry, comp, agv_control);

    bool gaps_found = false;

    if (!gaps_found) // this if block needs to be before rwa.InitRegionDictionaryDependingOnSituation() call
    {
        gaps_found = rwa.detectGaps();
    }
    
    while(ros::ok()) {
        ROS_INFO_STREAM("Starting while loop...");
        rwa.processOrder();
        ros::Duration(1.0).sleep(); // added for RWA5, try to deal with incosistency conveyor==1 when not and also bouncing to agv1 and start

        rwa.InitRegionDictionaryDependingOnSituation(); // Initialize region (shelf + upper or lower) dictionary depending on situation.
        if (rwa.checkConveyor()) continue;       
        if (rwa.buildKit()) continue;
        rwa.checkAgvErrors();
        ros::Duration(0.5).sleep();
        if (rwa.competition_over()) break;

        // auto dirs = rwa.lane_handler.queryLanes();
        // ROS_INFO_STREAM("--------------");
        // for (int i{AllLanesHandler::LEFT}; i <= AllLanesHandler::RIGHT; i++) {
        //     std::string prompt{""};
        //     if (dirs[i] == 0) prompt = "Traveling away from conveyor";
        //     else if (dirs[i] == 1) prompt = "Traveling towards conveyor";
        //     else prompt = "Direction unknown. Likely no obstacle in lane";
        //     ROS_INFO_STREAM("Lane " << i << ", Dir " << dirs[i] << ", " << prompt);
        // }
        // ROS_INFO_STREAM("--------------");
        // ros::Duration(1).sleep();
    }

    comp.endCompetition();

    spinner.stop();
    ros::shutdown();
    return 0;
}
