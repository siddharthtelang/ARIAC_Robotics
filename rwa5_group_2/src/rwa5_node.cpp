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
    /////////////////////////////////////////////////////////////////////////////
    
    
    // preset locations
    PresetLocation start_a;
    PresetLocation bin3_a;
    PresetLocation bingreen_a;
    PresetLocation binblue_a;
    PresetLocation agv2_a;
    PresetLocation agv1_staging_a;
    PresetLocation bottom_left_staging_a;
    PresetLocation shelf5_a;
    PresetLocation shelf5_spun_a;


    // joint positions to go to start location
    start_a.gantry = {0, 0, 0};
    start_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin3
    bin3_a.gantry = {4.0, -1.1, 0.};
    bin3_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin3_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bingreen
    bingreen_a.gantry = {4.0, -1.1, 0.};
    bingreen_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bingreen_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to binblue
    binblue_a.gantry = {2.96, -1.1, 0.};
    binblue_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    binblue_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv2
    agv2_a.gantry = {0.6, 6.9, PI};
    agv2_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv1
    agv1_staging_a.gantry = {0.6, -6.9, 0.00};
    agv1_staging_a.left_arm = {-PI/2, -1.01, 1.88, -1.13, 0.00, 0.00};
    agv1_staging_a.right_arm = {PI/2, -1.01, 1.88, -1.13, 0.00, 0.00}; // same except for joint 0

    bottom_left_staging_a.gantry = {-14.22, -6.75, 0.00};
    bottom_left_staging_a.left_arm = {-PI/2, -1.01, 1.88, -1.13, 0.00, 0.00};
    bottom_left_staging_a.right_arm = {PI/2, -1.01, 1.88, -1.13, 0.00, 0.00}; // same except for joint 0

    // shelf5_a.gantry = {-14.22, -4.15, 0.00};
    shelf5_a.gantry = {-14.42, -4.30, 0.00}; // WORKS FOR LEFT SHELF PULLEY NOT RIGHT
    // shelf5_a.gantry = {-14.72, -4.30, 0.00};
    // shelf5_a.gantry = {-14.42 - .897919, -4.30 - .853737, 0.00};
    // shelf5_a.gantry = {-15.42, -4.30, 0.00}; // WORKS FOR RIGHT SHELF PULLEY
    // shelf5_a.left_arm = {-PI/2, -1.01, 1.88, -1.13, 0.00, 0.00}; // higher up
    // shelf5_a.left_arm = {-1.64, -0.99, 1.84, -.85, -.08, -.26};
    shelf5_a.left_arm = {-1.76, -1.00, 1.86, -.85, -.20, -.26};
    shelf5_a.right_arm = {PI/2, -1.01, 1.88, -1.13, 0.00, 0.00}; // same except for joint 0

    shelf5_spun_a.gantry = {-15.42, -4.30, 3.14};
    shelf5_spun_a.left_arm = {-PI/2, -1.01, 1.88, -1.13, 0.00, 0.00};
    shelf5_spun_a.right_arm = {PI/2, -1.01, 1.88, -1.13, 0.00, 0.00}; // same except for joint 0

    /////////////////////////////////////////////////////////////////////////////



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
    
    while(ros::ok()) {
        ROS_INFO_STREAM("Starting while loop...");
        if (!gaps_found)
        {
            gaps_found = rwa.detectGaps();
        }
        rwa.processOrder();
        if (rwa.checkConveyor()) continue;       
        rwa.buildKit();
        rwa.checkAgvErrors();
        ros::Duration(0.5).sleep();
        if (rwa.competition_over()) break;
        
        // ROS_INFO_STREAM("----------------------------");
        // ROS_INFO_STREAM("left lane: " << rwa.left_lane.queryPair());
        // ROS_INFO_STREAM("mid left lane: " << rwa.mid_left_lane.queryPair());
        // ROS_INFO_STREAM("mid right lane: " << rwa.mid_right_lane.queryPair());
        // ROS_INFO_STREAM("right lane: " << rwa.right_lane.queryPair());
    }
    // //--1-Read order
    // std::vector<Product> total_products;
    // std::vector<Product> products;
    // product product;
    // CameraListener::ModelInfo pickPart;
    // std::vector<Order> orders;
    // std::vector<Shipment> shipments;
    // std::string shipment_type;
    // std::string agv_id;
    // std::string kit_id;
    // bool partFound = false;

    // bool isPartFaulty = true; // assume part is faulty (repeat while loop) until we see otherwise
    // std::map<int, PresetLocation> cam_to_presetlocation = {
    //     {0, bin3_a},
    //     {7, bin3_a},
    //     {9, shelf5_a},
    //     {12, shelf5_a}
    // };

    // THIS SECTION WILL EVENTUALLY BE IT'S OWN FUNCTION. JUST HERE FOR TESTING.
    // PresetLocation conveyor_belt;
    // conveyor_belt.gantry = {0, 3, PI/2};
    // // conveyor_belt.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    // // conveyor_belt.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    // conveyor_belt.left_arm = {-0.2, -PI / 4, PI / 2, -PI / 4, PI / 2 - 0.2, 0};
    // conveyor_belt.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    
    // ros::Subscriber breakbeam_sub = node.subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, &cam_listener);
    // ros::Time current_part_load_time;
    // CameraListener::ModelInfo current_part_on_conveyor;
    // std::deque<CameraListener::ModelInfo> parts_on_conveyor; 
    // float dx = 3+4.492549-1;
    // bool waiting_for_part{false};
    // while(ros::ok()) {
    //     if (!waiting_for_part && !cam_listener.load_time_on_conveyor_.empty()) {
    //         auto cam_parts = cam_listener.fetchParts(node)[5];
    //         for (auto& found_part : cam_parts) {
    //             bool part_in_queue{false};
    //             for (auto& part: parts_on_conveyor) {
    //                 if (part.id == found_part.id) {
    //                     part_in_queue = true;
    //                     break;
    //                 }
    //             } 
    //             if (!part_in_queue) parts_on_conveyor.push_back(found_part);
    //         }
    //         waiting_for_part = true;
    //         current_part_load_time = cam_listener.load_time_on_conveyor_.front();
    //         current_part_on_conveyor = parts_on_conveyor.front();
    //         cam_listener.load_time_on_conveyor_.pop();
    //         parts_on_conveyor.pop_front();
    //         gantry.goToPresetLocation(conveyor_belt);
    //     } else if (waiting_for_part && (ros::Time::now()-current_part_load_time).toSec() > dx/cam_listener.conveyor_spd_) {
    //         ROS_INFO_STREAM("Pick up part now!");
    //         part temp_part;
    //         temp_part.type = current_part_on_conveyor.type;
    //         temp_part.pose = current_part_on_conveyor.world_pose;
    //         temp_part.pose.position.y = -2.5;
    //         ROS_INFO_STREAM("x: " << temp_part.pose.position.x << ", y: " << temp_part.pose.position.y << ", z: " << temp_part.pose.position.z);
    //         // current_part_on_conveyor.world_pose
    //         waiting_for_part = false;
    //         gantry.pickPart(temp_part, "left_arm");
    //     }
    //     ros::Duration(0.5).sleep();
    // }


//     do{
//     //comp.processOrder();
//     orders = comp.get_orders_list();
//         //total_products = comp.get_product_list();
//     } while (orders.size() == 0);

//     // iterate through each order
//     for (int i = 0; i < orders.size(); i++) {////////// FOR ORDER IN ORDERS
//         shipments = orders[i].shipments;
//         // iterate through each shipment
//         for (int j = 0; j < shipments.size(); j++) { ////////// FOR SHIPMENT IN SHIPMENTS
//         // for (int j = 1; j < 2; j++) { ////////// Testing: FOR FIRST SHIPMENT ONLY (FOR NOW)
//             shipment_type = shipments[j].shipment_type;
//             agv_id = shipments[j].agv_id;
//             products = shipments[j].products;

//             ROS_INFO_STREAM("Shipment type = " << shipment_type);
//             ROS_INFO_STREAM("AGV ID = " << agv_id);

//             ROS_INFO_STREAM("Shipment size = " << shipments.size());

//             // iterate through each product
//             for (int k = 0; k < products.size(); k++) { ////////// FOR PRODUCT IN PRODUCTS
//             // for (int k = 0; k < 2; k++) { ////////// Testing: FOR FIRST 2 PRODUCTS ONLY (FOR NOW)
//                 product = products[k];
//                 partFound = false;
//                 isPartFaulty = true;
//                 ROS_INFO_STREAM("Product type = " << product.type);

//                 while (isPartFaulty == true) // break if part is good (nonfaulty) or if part not found (assume not found == there are no more parts available)
//                 {
//                     // get the parts from camera sensor
//                     int discovered_cam_idx = 0; // the camera we found it on.
//                     auto list = cam_listener.fetchParts(node);

//                     for (auto cam : list) {
//                         if (cam.empty() == true) // check if cam is empty, prevent segfault when doing cam[0] in the else if
//                         {
//                             ROS_INFO("No parts found under this camera. Skip this.");
//                             continue;
//                         }
//                         else if (cam[0].cam_index == 3 || cam[0].cam_index == 4)
//                         {
//                             ROS_INFO("Part found on camera over AGV. Skip this.");
//                             continue;
//                         }
//                         // else if (cam[0].cam_index == 9 || cam[0].cam_index == 12) /////////// TODO: REMOVE
//                         // {
//                         //     ROS_INFO("Skipping cam 9 and 12 for now for testing_________________________________________________________.");
//                         //     continue;
//                         // }

//                         for (auto model : cam) {
//                             if (product.type == (model.type + "_part_" + model.color)) {
//                                 ROS_INFO("Part found. Perform the pickup action");
//                                 pickPart = model;
//                                 partFound = true;
//                                 discovered_cam_idx = cam[0].cam_index;
//                                 break;
//                             }
//                         }
//                         if (partFound)
//                             break;
//                     }

//                     if (partFound) {
//                         if (discovered_cam_idx == 0 || discovered_cam_idx == 7) { // 0 and 7 are bin cameras
//                             ROS_INFO_STREAM("Pose --- " << pickPart.world_pose);
//                             ROS_INFO_STREAM("Found part on camera 0 or camera 7, the bins");

//                             // part to pick
//                             part my_part;
//                             my_part.type = product.type;
//                             my_part.pose = pickPart.world_pose;
//                             // tray location
//                             part part_in_tray;
//                             part_in_tray.type = product.type;
//                             part_in_tray.pose = product.pose;

//                             //TODO: deduce the position based on camera and go to location
//                             // gantry.goToPresetLocation(gantry.bin3_);
//                             double add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
//                             double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y
//                             // double add_to_x = my_part.pose.position.x - 4.665789; // constant is perfect bin red pulley x
//                             // double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y

//                             ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);
//                             gantry.goToPresetLocation(Bump(cam_to_presetlocation[discovered_cam_idx], add_to_x, add_to_y, 0));

//                             //--Go pick the part
//                             if (!gantry.pickPart(my_part, "left_arm")){
//                                 // gantry.goToPresetLocation(gantry.start_);
//                                 // spinner.stop();
//                                 // ros::shutdown();
//                                 ;//pass
//                             }

//                             // go back to start
//                             gantry.goToPresetLocation(start_a);
//                             ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves
//                             // gantry.goToPresetLocation(start_a);

//                             //place the part
//                             gantry.placePart(part_in_tray, agv_id, "left_arm");
//                             cam_listener.checkFaulty(node, agv_id);
//                             ros::Duration(5.0).sleep(); // make sure it actually goes back to start, instead of running into shelves

//                             if (cam_listener.faulty_parts) {
//                                 ROS_INFO("Detected Faulty Part");
//                                 for(int f_p = 0; f_p < cam_listener.faulty_parts_list.size(); ++f_p) {
//                                     CameraListener::ModelInfo faulty = cam_listener.faulty_parts_list.at(2);
//                                     Part faulty_part;
//                                     faulty_part.pose = faulty.model_pose;
//                                     ROS_INFO_STREAM("Faulty product pose:" <<faulty_part.pose);
//                                     for(auto product:products) {
//                                         if (product.pose == faulty_part.pose)
//                                             faulty_part.type = product.type;
//                                         ROS_INFO_STREAM("shipment product pose:" << product.pose);

//                                     }
//                                     faulty_part.type = part_in_tray.type;
//                                     faulty_part.pose = part_in_tray.pose;
//                                     ROS_INFO_STREAM("Faulty Part:"<< faulty_part.type);
//                                     cam_listener.faulty_parts_list.clear();
//                                     bool success = gantry.replaceFaultyPart(faulty_part, agv_id, "left_arm");
//                                     if (success) {
//                                         cam_listener.faulty_parts_list.clear();
//                                         isPartFaulty = false;
//                                         k -= 1;
//                                     }
//                                 }
//                                 // tray location
//                                 // todo: poll quality sensor (eg camera 3 and 4), pick part, gantry go to start, drop part
// //                                gantry.pickPart(part_in_tray);
// //                                gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
// //                                gantry.deactivateGripper("left_arm");
// //                                gantry.deactivateGripper("right_arm");
//                             }
//                             else {
//                                 isPartFaulty = false;
//                                 gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
//                             }
//                             gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start

//                         }
//                         else if (discovered_cam_idx == 9 || discovered_cam_idx == 12) { // 9 and 12 are shelf cameras
//                             // part to pick
//                             part my_part;
//                             my_part.type = product.type;
//                             my_part.pose = pickPart.world_pose;
//                             // tray location
//                             part part_in_tray;
//                             part_in_tray.type = product.type;
//                             part_in_tray.pose = product.pose;

//                             gantry.goToPresetLocation(agv1_staging_a);
//                             gantry.goToPresetLocation(bottom_left_staging_a);
//                             double add_to_x_shelf = my_part.pose.position.x - -13.522081;
//                             gantry.goToPresetLocation( Bump(shelf5_a, add_to_x_shelf, 0, 0) ); // offset gantry from pulley by some units in x direction

//                             //--Go pick the part
//                             if (!gantry.pickPart(my_part, "left_arm")){
//                                 // gantry.goToPresetLocation(gantry.start_);
//                                 // spinner.stop();
//                                 // ros::shutdown();
//                                 ;//pass
//                             }

//                             gantry.goToPresetLocation(shelf5_a); // keep part from sliding across table
//                             gantry.goToPresetLocation(bottom_left_staging_a);
//                             gantry.goToPresetLocation(agv1_staging_a);
//                             gantry.goToPresetLocation(start_a);
//                             //place the part
//                             gantry.placePart(part_in_tray, agv_id, "left_arm");
//                             cam_listener.checkFaulty(node, agv_id);
//                             ros::Duration(5.0).sleep(); // make sure it actually goes back to start, instead of running into shelves

//                             if (cam_listener.faulty_parts_list.size()) {
//                                 ROS_INFO("Detected Faulty Part");
//                                 for(int f_p = 0; f_p < cam_listener.faulty_parts_list.size(); ++f_p) {
//                                     CameraListener::ModelInfo faulty = cam_listener.faulty_parts_list.at(2);
//                                     Part faulty_part;
//                                     faulty_part.pose = faulty.model_pose;
//                                     ROS_INFO_STREAM("Faulty product pose:" <<faulty_part.pose);
//                                     for(auto product:products) {
//                                         if (product.pose == faulty_part.pose)
//                                             faulty_part.type = product.type;
//                                         ROS_INFO_STREAM("shipment product pose:" << product.pose);
//                                     }
//                                     faulty_part.type = part_in_tray.type;
//                                     faulty_part.pose = part_in_tray.pose;
//                                     ROS_INFO_STREAM("Faulty Part:"<< faulty_part.type);
//                                     cam_listener.faulty_parts_list.clear();
//                                     bool success = gantry.replaceFaultyPart(faulty_part, agv_id, "left_arm");
//                                     if (success) {
//                                         cam_listener.faulty_parts_list.clear();
//                                         isPartFaulty = false;
//                                         k -= 1;
//                                     }
//                                 }

//                             }
//                             else {
//                                 isPartFaulty = false;
//                                 gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
//                             }
//                             gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
//                         }

//                     }
//                     else { // else part NOT found, or otherwise,
//                         gantry.goToPresetLocation(start_a);
//                         break; // break out of while loop, (assume not found == there are no more parts available, conveyor might mess with this)
//                     }


//                 }//end while loop
//             }
//             ROS_INFO("\n\n");

//             // one shipment completed. Send to AGV
//             AGVControl agv_control(node);
//             kit_id = (agv_id == "agv1" ? "kit_tray_1" : "kit_tray_2");
//             agv_control.sendAGV(shipment_type, kit_id);

//         }
//     }
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    //gantry.goToPresetLocation(gantry.bin3_);

    //std::vector<Product> list_of_products = comp.get_product_list();
    //--TODO: Parse each product in list_of_products
    //--TODO: For each product in list_of_product find the part in the environment using cameras
    //--TODO: Choose one part and pick it up
    //--Assume the following part was found by a camera
    // part my_part;
    // my_part.type = "pulley_part_red";
    // my_part.pose.position.x = 4.365789;
    // my_part.pose.position.y = 1.173381;
    // my_part.pose.position.z = 0.728011;
    // my_part.pose.orientation.x = 0.012;
    // my_part.pose.orientation.y = -0.004;
    // my_part.pose.orientation.z = 0.002;
    // my_part.pose.orientation.w = 1.000;

    //--Where to place the part in the tray?
    //--TODO: Get this information from /ariac/orders (list_of_products in this case)
    // part part_in_tray;
    // part_in_tray.type = "pulley_part_red";
    // part_in_tray.pose.position.x = -0.1;
    // part_in_tray.pose.position.y = -0.2;
    // part_in_tray.pose.position.z = 0.0;
    // part_in_tray.pose.orientation.x = 0.0;
    // part_in_tray.pose.orientation.y = 0.0;
    // part_in_tray.pose.orientation.z = 0.382683;
    // part_in_tray.pose.orientation.w = 0.92388;

    // //--Go pick the part
    // if (!gantry.pickPart(my_part)){
    //     gantry.goToPresetLocation(gantry.start_);
    //     spinner.stop();
    //     ros::shutdown();
    // }
    
    //--Go place the part
    //--TODO: agv2 should be retrieved from /ariac/orders (list_of_products in this case)
    // gantry.placePart(part_in_tray, "agv2");

    // AGVControl agv_control(node);
    // //--TODO: get the following arguments from the order
    // agv_control.sendAGV("order_0_shipment_0", "kit_tray_2");
    comp.endCompetition();

    spinner.stop();
    ros::shutdown();
    return 0;
}
