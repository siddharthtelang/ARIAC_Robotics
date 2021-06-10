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

/**
 * \brief: Main for rwa3 node
 * \param: argc
 * \param: argv
 * \result: Applies control of gantry robot
 */
PresetLocation Bump(PresetLocation location_to_modify, double small_rail, double large_rail, double torso ) {
    location_to_modify.gantry.at(0) = location_to_modify.gantry.at(0) + small_rail;
    location_to_modify.gantry.at(1) = location_to_modify.gantry.at(1) - large_rail; // Minus now, does simulation switch?
    location_to_modify.gantry.at(2) = location_to_modify.gantry.at(2) + torso;

    return location_to_modify;
}

int main(int argc, char ** argv) {
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
    RWAImplementation rwa(node, cam_listener, gantry);

    //--1-Read order
    std::vector<Product> total_products;
    std::vector<Product> products;
    product product;
    CameraListener::ModelInfo pickPart;
    std::vector<Order> orders;
    std::vector<Shipment> shipments;
    std::string shipment_type;
    std::string agv_id;
    std::string kit_id;
    bool partFound = false;

    bool isPartFaulty = true; // assume part is faulty (repeat while loop) until we see otherwise
    std::map<int, PresetLocation> cam_to_presetlocation = {
        {0, bin3_a},
        {7, bin3_a},
        {9, shelf5_a},
        {12, shelf5_a}
    };

    while(ros::ok()) {
        // Check for new order
        orders = comp.get_orders_list(); 

        // Check conveyor belt
        if(!rwa.checkConveyor(true)) continue;

        // Grab part
        // if(orders.size > 0)

        // Check AGV Errors
    }

    // iterate through each order
    for (int i = 0; i < orders.size(); i++) {////////// FOR ORDER IN ORDERS
        shipments = orders[i].shipments;
        // iterate through each shipment
        for (int j = 0; j < shipments.size(); j++) { ////////// FOR SHIPMENT IN SHIPMENTS
        // for (int j = 1; j < 2; j++) { ////////// Testing: FOR FIRST SHIPMENT ONLY (FOR NOW)
            shipment_type = shipments[j].shipment_type;
            agv_id = shipments[j].agv_id;
            products = shipments[j].products;

            ROS_INFO_STREAM("Shipment type = " << shipment_type);
            ROS_INFO_STREAM("AGV ID = " << agv_id);

            ROS_INFO_STREAM("Shipment size = " << shipments.size());

            // iterate through each product
            for (int k = 0; k < products.size(); k++) { ////////// FOR PRODUCT IN PRODUCTS
                product = products[k];
                partFound = false;
                isPartFaulty = true;
                ROS_INFO_STREAM("Product type = " << product.type);

                while (isPartFaulty == true) // break if part is good (nonfaulty) or if part not found (assume not found == there are no more parts available)
                {
                    // get the parts from camera sensor
                    int discovered_cam_idx = 0; // the camera we found it on.
                    auto list = cam_listener.fetchParts(node);

                    for (auto cam : list) {
                        if (cam.empty() == true) // check if cam is empty, prevent segfault when doing cam[0] in the else if
                        {
                            ROS_INFO("No parts found under this camera. Skip this.");
                            continue;
                        }
                        else if (cam[0].cam_index == 3 || cam[0].cam_index == 4)
                        {
                            ROS_INFO("Part found on camera over AGV. Skip this.");
                            continue;
                        }
                        // else if (cam[0].cam_index == 9 || cam[0].cam_index == 12) /////////// TODO: REMOVE
                        // {
                        //     ROS_INFO("Skipping cam 9 and 12 for now for testing_________________________________________________________.");
                        //     continue;
                        // }

                        for (auto model : cam) {
                            if (product.type == (model.type + "_part_" + model.color)) {
                                ROS_INFO("Part found. Perform the pickup action");
                                pickPart = model;
                                partFound = true;
                                discovered_cam_idx = cam[0].cam_index;
                                break;
                            }
                        }
                        if (partFound)
                            break;
                    }

                    if (partFound) {
                        if (discovered_cam_idx == 0 || discovered_cam_idx == 7) { // 0 and 7 are bin cameras
                            ROS_INFO_STREAM("Pose --- " << pickPart.world_pose);
                            ROS_INFO_STREAM("Found part on camera 0 or camera 7, the bins");

                            // part to pick
                            part my_part;
                            my_part.type = product.type;
                            my_part.pose = pickPart.world_pose;
                            // tray location
                            part part_in_tray;
                            part_in_tray.type = product.type;
                            part_in_tray.pose = product.pose;

                            //TODO: deduce the position based on camera and go to location
                            // gantry.goToPresetLocation(gantry.bin3_);
                            double add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
                            double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y
                            // double add_to_x = my_part.pose.position.x - 4.665789; // constant is perfect bin red pulley x
                            // double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y

                            ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);
                            gantry.goToPresetLocation(Bump(cam_to_presetlocation[discovered_cam_idx], add_to_x, add_to_y, 0));

                            //--Go pick the part
                            if (!gantry.pickPart(my_part, "left_arm")){
                                // gantry.goToPresetLocation(gantry.start_);
                                // spinner.stop();
                                // ros::shutdown();
                                ;//pass
                            }

                            // go back to start
                            gantry.goToPresetLocation(start_a);
                            ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves
                            // gantry.goToPresetLocation(start_a);

                            //place the part
                            gantry.placePart(part_in_tray, agv_id, "left_arm");
                            cam_listener.checkFaulty(node, agv_id);
                            ros::Duration(5.0).sleep(); // make sure it actually goes back to start, instead of running into shelves

                            if (cam_listener.faulty_parts) {
                                ROS_INFO("Detected Faulty Part");
                                for(int f_p = 0; f_p < cam_listener.faulty_parts_list.size(); ++f_p) {
                                    CameraListener::ModelInfo faulty = cam_listener.faulty_parts_list.at(2);
                                    Part faulty_part;
                                    faulty_part.pose = faulty.model_pose;
                                    ROS_INFO_STREAM("Faulty product pose:" <<faulty_part.pose);
                                    for(auto product:products) {
                                        if (product.pose == faulty_part.pose)
                                            faulty_part.type = product.type;
                                        ROS_INFO_STREAM("shipment product pose:" << product.pose);

                                    }
                                    faulty_part.type = part_in_tray.type;
                                    faulty_part.pose = part_in_tray.pose;
                                    ROS_INFO_STREAM("Faulty Part:"<< faulty_part.type);
                                    cam_listener.faulty_parts_list.clear();
                                    bool success = gantry.replaceFaultyPart(faulty_part, agv_id, "left_arm");
                                    if (success) {
                                        cam_listener.faulty_parts_list.clear();
                                        isPartFaulty = false;
                                        k -= 1;
                                    }
                                }
                                // tray location
                                // todo: poll quality sensor (eg camera 3 and 4), pick part, gantry go to start, drop part
//                                gantry.pickPart(part_in_tray);
//                                gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
//                                gantry.deactivateGripper("left_arm");
//                                gantry.deactivateGripper("right_arm");
                            }
                            else {
                                isPartFaulty = false;
                                gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
                            }
                            gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start

                        }
                        else if (discovered_cam_idx == 9 || discovered_cam_idx == 12) { // 9 and 12 are shelf cameras
                            // part to pick
                            part my_part;
                            my_part.type = product.type;
                            my_part.pose = pickPart.world_pose;
                            // tray location
                            part part_in_tray;
                            part_in_tray.type = product.type;
                            part_in_tray.pose = product.pose;

                            gantry.goToPresetLocation(agv1_staging_a);
                            gantry.goToPresetLocation(bottom_left_staging_a);
                            double add_to_x_shelf = my_part.pose.position.x - -13.522081;
                            gantry.goToPresetLocation( Bump(shelf5_a, add_to_x_shelf, 0, 0) ); // offset gantry from pulley by some units in x direction

                            //--Go pick the part
                            if (!gantry.pickPart(my_part, "left_arm")){
                                // gantry.goToPresetLocation(gantry.start_);
                                // spinner.stop();
                                // ros::shutdown();
                                ;//pass
                            }

                            gantry.goToPresetLocation(shelf5_a); // keep part from sliding across table
                            gantry.goToPresetLocation(bottom_left_staging_a);
                            gantry.goToPresetLocation(agv1_staging_a);
                            gantry.goToPresetLocation(start_a);
                            //place the part
                            gantry.placePart(part_in_tray, agv_id, "left_arm");
                            cam_listener.checkFaulty(node, agv_id);
                            ros::Duration(5.0).sleep(); // make sure it actually goes back to start, instead of running into shelves

                            if (cam_listener.faulty_parts_list.size()) {
                                ROS_INFO("Detected Faulty Part");
                                for(int f_p = 0; f_p < cam_listener.faulty_parts_list.size(); ++f_p) {
                                    CameraListener::ModelInfo faulty = cam_listener.faulty_parts_list.at(2);
                                    Part faulty_part;
                                    faulty_part.pose = faulty.model_pose;
                                    ROS_INFO_STREAM("Faulty product pose:" <<faulty_part.pose);
                                    for(auto product:products) {
                                        if (product.pose == faulty_part.pose)
                                            faulty_part.type = product.type;
                                        ROS_INFO_STREAM("shipment product pose:" << product.pose);
                                    }
                                    faulty_part.type = part_in_tray.type;
                                    faulty_part.pose = part_in_tray.pose;
                                    ROS_INFO_STREAM("Faulty Part:"<< faulty_part.type);
                                    cam_listener.faulty_parts_list.clear();
                                    bool success = gantry.replaceFaultyPart(faulty_part, agv_id, "left_arm");
                                    if (success) {
                                        cam_listener.faulty_parts_list.clear();
                                        isPartFaulty = false;
                                        k -= 1;
                                    }
                                }

                            }
                            else {
                                isPartFaulty = false;
                                gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
                            }
                            gantry.goToPresetLocation(start_a); // part placed, not faulty, so just go back to start
                        }

                    }
                    else { // else part NOT found, or otherwise,
                        gantry.goToPresetLocation(start_a);
                        break; // break out of while loop, (assume not found == there are no more parts available, conveyor might mess with this)
                    }


                }//end while loop
            }
            ROS_INFO("\n\n");

            // one shipment completed. Send to AGV
            AGVControl agv_control(node);
            kit_id = (agv_id == "agv1" ? "kit_tray_1" : "kit_tray_2");
            agv_control.sendAGV(shipment_type, kit_id);

        }
    }
    // --2-Look for parts in this order
    // --We go to this bin because a camera above
    // --this bin found one of the parts in the order
    gantry.goToPresetLocation(gantry.bin3_);

    std::vector<Product> list_of_products = comp.get_product_list();
    // --TODO: Parse each product in list_of_products
    // --TODO: For each product in list_of_product find the part in the environment using cameras
    // --TODO: Choose one part and pick it up
    // --Assume the following part was found by a camera
    part my_part;
    my_part.type = "pulley_part_red";
    my_part.pose.position.x = 4.365789;
    my_part.pose.position.y = 1.173381;
    my_part.pose.position.z = 0.728011;
    my_part.pose.orientation.x = 0.012;
    my_part.pose.orientation.y = -0.004;
    my_part.pose.orientation.z = 0.002;
    my_part.pose.orientation.w = 1.000;

    // --Where to place the part in the tray?
    // --TODO: Get this information from /ariac/orders (list_of_products in this case)
    part part_in_tray;
    part_in_tray.type = "pulley_part_red";
    part_in_tray.pose.position.x = -0.1;
    part_in_tray.pose.position.y = -0.2;
    part_in_tray.pose.position.z = 0.0;
    part_in_tray.pose.orientation.x = 0.0;
    part_in_tray.pose.orientation.y = 0.0;
    part_in_tray.pose.orientation.z = 0.382683;
    part_in_tray.pose.orientation.w = 0.92388;

    //--Go pick the part
    if (!gantry.pickPart(my_part)){
        gantry.goToPresetLocation(gantry.start_);
        spinner.stop();
        ros::shutdown();
    }
    
    // --Go place the part
    // --TODO: agv2 should be retrieved from /ariac/orders (list_of_products in this case)
    gantry.placePart(part_in_tray, "agv2");

    AGVControl agv_control(node);
    //--TODO: get the following arguments from the order
    agv_control.sendAGV("order_0_shipment_0", "kit_tray_2");
    comp.endCompetition();

    spinner.stop();
    ros::shutdown();
    return 0;
}
