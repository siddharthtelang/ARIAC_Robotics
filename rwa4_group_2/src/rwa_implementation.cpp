#include "rwa_implementation.h"
#include <unordered_map>

#define GET_VARIABLE_NAME(Variable) (#Variable)


void RWAImplementation::processOrder() {
    auto order_list = competition_->get_orders_list();
    if (order_list.size() == prev_num_orders_) return;

    prev_num_orders_++;
    ROS_INFO_STREAM("Processing Order...");

    std::unordered_map<std::string, std::unordered_map<std::string, std::queue<Product>>> total_products;
    std::vector<Product> products;
    std::string shipment_type;

    std::string delimiter = "_";

    cam_listener_->fetchParts(*node_);
    cam_listener_->sort_camera_parts_list();
    auto sorted_map = cam_listener_->sortPartsByDist();

    auto order = order_list.back();
    std::queue<std::vector<Product>> order_task_queue;
    for(const auto& shipment : order.shipments) {
        shipment_type = shipment.shipment_type;
        std::string agv_id = shipment.agv_id;
        products = shipment.products;

        ROS_INFO_STREAM("Shipment type = " << shipment_type);
        ROS_INFO_STREAM("AGV ID = " << agv_id);

        ROS_INFO_STREAM("Shipment size = " << order.shipments.size());

        for (auto& product : products) { ////////// FOR PRODUCT IN PRODUCTS
            ROS_INFO_STREAM("Product type = " << product.type);
            product.agv_id = agv_id;
            std::string color = product.type;
            std::string type = color.substr(0, color.find(delimiter));
            int pos{};
            while ((pos = color.find(delimiter)) != std::string::npos) {
                color.erase(0,pos+delimiter.length());
            }
            if(!sorted_map[color][type].empty()) {
                product.designated_model = sorted_map[color][type].top();
                sorted_map[color][type].pop();

                order_task_queue.push(std::vector<Product>{product});

            } else {
                product.get_from_conveyor = true;  // red pulleys (or any other parts in order which are not in map)
                                                    // are not getting filled out with product.designated_model, hence they have product.designated_model.cam_idx = large number
                // order_task_queue.push_to_the_very_back(std::vector<Product>{product}) ; // need a new data structure, cannot push to very back of C++ queue
            }
            
            ROS_INFO_STREAM("hope the right cam_idx is here ===" << product.designated_model.cam_index);
            // order_task_queue.push(std::vector<Product>{product});
            // total_products[color][type].push(product); // do not uncomment
        }
    }
         
    // std::queue<Product> needs_pair;
    // std::queue<std::vector<Product>> order_task_queue;
    // for(auto color : cam_listener_->colors_) {
    //     for(auto type : cam_listener_->types_) {
    //         auto similar_parts = total_products[color][type];
    //         while(similar_parts.size() > 1){
    //             std::vector<Product> task_pair{};
    //             task_pair.push_back(similar_parts.front());
    //             similar_parts.pop();
    //             task_pair.push_back(similar_parts.front());
    //             similar_parts.pop();
    //             order_task_queue.push(task_pair);
    //         }
    //         if(similar_parts.size() == 1) {
    //             if(needs_pair.size() > 0) {
    //                 std::vector<Product> task_pair{};
    //                 task_pair.push_back(needs_pair.front());
    //                 needs_pair.pop();
    //                 task_pair.push_back(similar_parts.front());
    //                 similar_parts.pop();
    //                 order_task_queue.push(task_pair);
    //             } else needs_pair.push(similar_parts.front());
    //         }
    //     }
    // }

    // while(needs_pair.size() > 0) {
    //     order_task_queue.push(std::vector<Product>{needs_pair.front()});
    //     needs_pair.pop();
    // }
    task_queue_.push(order_task_queue);
    ROS_INFO_STREAM("Completed order processing. All parts on task queue.");

    // // CANNOT ITERATE OVER QUEUE. THIS PART IS JUST FOR TESTING. WILL NOT WORK IF UNCOMMENTED DURING EXECUTION
    // ROS_INFO_STREAM("");
    // auto task_queue = task_queue_.top();
    // while(!task_queue.empty()) {
    //     auto vec = task_queue.front();
    //     ROS_INFO_STREAM("Vector on queue.");
    //     for(auto product : vec) {
    //         ROS_INFO_STREAM("  Product on queue.");
    //         ROS_INFO_STREAM("  Product type: " << product.type);
    //         ROS_INFO_STREAM("  Product target AGV: " << product.agv_id);
    //         ROS_INFO_STREAM("    Designated model id: " << product.designated_model.id);
    //         ROS_INFO_STREAM("    Designated model type: " << product.designated_model.color << " " << product.designated_model.type << "\n");
    //     }
    //     task_queue.pop();
    // }
}

bool RWAImplementation::checkConveyor(bool part_wanted) {
    bool continue_ = false;

    if (!waiting_for_part_ && !cam_listener_->load_time_on_conveyor_.empty()) {
        // auto cam_parts = cam_listener_->fetchPartsFromCamera(*node_, 5);
        auto cam_parts = cam_listener_->fetchParts(*node_)[5];
        for (const auto& found_part : cam_parts) {
            bool part_in_queue{false};
            for (const auto& part : parts_on_conveyor_) {
                if (part.id == found_part.id) {
                    part_in_queue = true;
                    break;
                }
            } 
            if (!part_in_queue) parts_on_conveyor_.push_back(found_part);
        }
        waiting_for_part_ = true;
        current_part_load_time_ = cam_listener_->load_time_on_conveyor_.front();
        current_part_on_conveyor_ = parts_on_conveyor_.front();
        cam_listener_->load_time_on_conveyor_.pop();
        parts_on_conveyor_.pop_front();
        gantry_->goToPresetLocation(conveyor_belt);
    } else if (part_wanted && waiting_for_part_ && (ros::Time::now()-current_part_load_time_).toSec() > dx_/cam_listener_->conveyor_spd_) {
        ROS_INFO_STREAM("Pick up part now!");
        part temp_part;
        temp_part.type = current_part_on_conveyor_.type;
        temp_part.pose = current_part_on_conveyor_.world_pose;
        temp_part.pose.position.y = -2.5;
        ROS_INFO_STREAM("x: " << temp_part.pose.position.x << ", y: " << temp_part.pose.position.y << ", z: " << temp_part.pose.position.z);
        // current_part_on_conveyor.world_pose
        waiting_for_part_ = false;
        gantry_->pickPart(temp_part, "left_arm");
        continue_ = true;
    }
    return continue_;
}

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


void RWAImplementation::initPresetLocs() {
    conveyor_belt.gantry = {0, 3, PI/2};
    conveyor_belt.left_arm = {-0.2, -PI / 4, PI / 2, -PI / 4, PI / 2 - 0.2, 0};
    conveyor_belt.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    conveyor_belt.name = GET_VARIABLE_NAME(conveyor_belt);

    // joint positions to go to start location
    start_a.gantry = {0, 0, 0};
    start_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_a.name = GET_VARIABLE_NAME(start_a);


    // joint positions to go to bin3
    bin3_a.gantry = {4.0, -1.1, 0.};
    bin3_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin3_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin3_a.name = GET_VARIABLE_NAME(bin3_a);


    // joint positions to go to bingreen
    bingreen_a.gantry = {4.0, -1.1, 0.};
    bingreen_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bingreen_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bingreen_a.name = GET_VARIABLE_NAME(bingreen_a);


    // joint positions to go to binblue
    binblue_a.gantry = {2.96, -1.1, 0.};
    binblue_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    binblue_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    binblue_a.name = GET_VARIABLE_NAME(binblue_a);


    // joint positions to go to agv2
    agv2_a.gantry = {0.6, 6.9, PI};
    agv2_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.name = GET_VARIABLE_NAME(agv2_a);


    // joint positions to go to agv1
    agv1_staging_a.gantry = {0.6, -6.9, 0.00};
    agv1_staging_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    agv1_staging_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same except for joint 0
    agv1_staging_a.name = GET_VARIABLE_NAME(agv1_staging_a);


    bottom_left_staging_a.gantry = {-14.22, -6.75, 0.00};
    bottom_left_staging_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    bottom_left_staging_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same except for joint 0
    bottom_left_staging_a.name = GET_VARIABLE_NAME(bottom_left_staging_a);


    // shelf5_a.gantry = {-14.22, -4.15, 0.00};
    shelf5_a.gantry = {-14.42, -4.30, 0.00}; // WORKS FOR LEFT SHELF PULLEY NOT RIGHT
    // shelf5_a.gantry = {-14.72, -4.30, 0.00};
    // shelf5_a.gantry = {-14.42 - .897919, -4.30 - .853737, 0.00};
    // shelf5_a.gantry = {-15.42, -4.30, 0.00}; // WORKS FOR RIGHT SHELF PULLEY
    // shelf5_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // higher up
    // shelf5_a.left_arm = {-1.64, -0.99, 1.84, -.85, -.08, -.26};
    // shelf5_a.left_arm = {-1.76, -1.00, 1.86, -.85, -.20, -.26}; // try fix
    shelf5_a.left_arm = {-1.76, -1.00, 1.86, -.85, -.20, 0.0}; // try fix
    shelf5_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf5_a.name = GET_VARIABLE_NAME(shelf5_a);


    shelf5_spun_a.gantry = {-15.42, -4.30, 3.14};
    shelf5_spun_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf5_spun_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf5_spun_a.name = GET_VARIABLE_NAME(shelf5_spun_a);


    mid_5_8_staging_a.gantry = {0.0, -1.5, 0.00};
    mid_5_8_staging_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.name = GET_VARIABLE_NAME(mid_5_8_staging_a);


    mid_8_11_staging_a.gantry = {0.0, 1.5, 0.00};
    mid_8_11_staging_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.name = GET_VARIABLE_NAME(mid_8_11_staging_a);


    shelf8_a.gantry = {-14.22, -1.5, 0.00};
    shelf8_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf8_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf8_a.name = GET_VARIABLE_NAME(shelf8_a);


    shelf11_a.gantry = {-14.22, 1.5, 0.00};
    shelf11_a.left_arm = {-PI/2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf11_a.right_arm = {PI/2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf11_a.name = GET_VARIABLE_NAME(shelf11_a);


    // joint positions to go to bin11 (and all 8 bins in the entire grouping)
    // bin11_a.gantry = {4.0, 1.1, PI};
    bin11_a.gantry = {4.0, -1.1, 0.}; // the exact same as bin311_a
    bin11_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin11_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin11_a.name = GET_VARIABLE_NAME(bin11_a);



    cam_to_presetlocation = {
        {0, bin3_a},
        {7, bin3_a},
        {9, bottom_left_staging_a},
        {12, bottom_left_staging_a},

        {6, shelf8_a},
        {16, shelf8_a},

        {8, shelf11_a},
        {11, shelf11_a},

        {1, bin11_a},
        {2, bin11_a}
    };

    // vector of some of the locations, when gantry moves anywhere, it first searches through these possible locations,
    // and tries to find the closest one to it. That approximate location then serves as the beginning location for any move lookup.
    preset_locations_list_ = {start_a, bin3_a, agv2_a, agv1_staging_a,
    bottom_left_staging_a, shelf8_a, shelf11_a, bin11_a}; // do not have mid_xyz anything here for now

        PathingLookupDictionary = {
        { {"start_a", "bin3_a"} , std::vector<PresetLocation>{start_a, bin3_a} },
        { {"start_a", "agv2_a"} , std::vector<PresetLocation>{start_a, agv2_a} },
        { {"start_a", "agv1_staging_a"} , std::vector<PresetLocation>{start_a, agv1_staging_a} },
        { {"start_a", "bottom_left_staging_a"} , std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a} },
        { {"start_a", "shelf8_a"} , std::vector<PresetLocation>{start_a, mid_5_8_staging_a, shelf8_a} },
        { {"start_a", "shelf11_a"} , std::vector<PresetLocation>{start_a, mid_8_11_staging_a, shelf11_a} },
        { {"start_a", "bin11_a"} , std::vector<PresetLocation>{start_a, bin11_a} },

        { {"bin3_a", "start_a"} , std::vector<PresetLocation>{bin3_a, start_a} },
        { {"agv2_a", "start_a"} , std::vector<PresetLocation>{agv2_a, start_a} },
        { {"agv1_staging_a", "start_a"} , std::vector<PresetLocation>{agv1_staging_a, start_a} },
        { {"bottom_left_staging_a", "start_a"} , std::vector<PresetLocation>{bottom_left_staging_a, agv1_staging_a, start_a} },
        { {"shelf8_a", "start_a"} , std::vector<PresetLocation>{shelf8_a, mid_5_8_staging_a, start_a} },
        { {"shelf11_a", "start_a"} , std::vector<PresetLocation>{shelf11_a, shelf11_a, shelf11_a, mid_8_11_staging_a, start_a} }, //GO to shelf 11a!
        { {"bin11_a", "start_a"} , std::vector<PresetLocation>{bin11_a, start_a} },

        { {"agv1_staging_a", "bin3_a"} , std::vector<PresetLocation>{start_a, bin3_a} }, // go to start_a first /////////// this block: do not go to first point first
        { {"agv1_staging_a", "agv2_a"} , std::vector<PresetLocation>{agv2_a} },
        { {"agv1_staging_a", "agv1_staging_a"} , std::vector<PresetLocation>{agv1_staging_a} }, // go to itself
        { {"agv1_staging_a", "bottom_left_staging_a"} , std::vector<PresetLocation>{agv1_staging_a, bottom_left_staging_a} },
        { {"agv1_staging_a", "shelf8_a"} , std::vector<PresetLocation>{mid_5_8_staging_a, shelf8_a} },
        { {"agv1_staging_a", "shelf11_a"} , std::vector<PresetLocation>{mid_8_11_staging_a, shelf11_a} },
        { {"agv1_staging_a", "bin11_a"} , std::vector<PresetLocation>{start_a, bin11_a} }, // go to start_a first

        { {"agv2_a", "bin3_a"} , std::vector<PresetLocation>{start_a, bin3_a} }, // go to start_a first /////////////// this block: do not go to first point first
        { {"agv2_a", "agv2_a"} , std::vector<PresetLocation>{agv2_a} }, // go to itself
        { {"agv2_a", "agv1_staging_a"} , std::vector<PresetLocation>{agv1_staging_a} },
        { {"agv2_a", "bottom_left_staging_a"} , std::vector<PresetLocation>{agv1_staging_a, bottom_left_staging_a} },
        { {"agv2_a", "shelf8_a"} , std::vector<PresetLocation>{mid_5_8_staging_a, shelf8_a} },
        { {"agv2_a", "shelf11_a"} , std::vector<PresetLocation>{mid_8_11_staging_a, shelf11_a} },
        { {"agv2_a", "bin11_a"} , std::vector<PresetLocation>{start_a, bin11_a} }, // go to start_a first

    };



    ROS_INFO_STREAM("init() inside rwa implementation.cpp ran");
}


void RWAImplementation::buildKit() {
    if (task_queue_.top().empty()) {
        ROS_INFO("Task queue is empty. Return");
        // agv_control_->sendAGV("order_0_shipment_0", "kit_tray_2");
        return;
    }
    Product product = task_queue_.top().front()[0];
    part my_part;
    my_part.type = product.type;
    my_part.pose = product.designated_model.world_pose;

    part part_in_tray;
    part_in_tray.type = task_queue_.top().front()[0].type;
    part_in_tray.pose = task_queue_.top().front()[0].pose;
    part_in_tray.initial_pose = product.designated_model.world_pose; // save the initial pose



    int discovered_cam_idx = product.designated_model.cam_index;

    if (discovered_cam_idx == 0 || discovered_cam_idx == 7 || discovered_cam_idx == 1 || discovered_cam_idx == 2 ) {

        double add_to_x = 0.0;
        double add_to_y = 0.0;
        if (discovered_cam_idx == 7 || discovered_cam_idx == 0) {
            add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
            add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y
            ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

        }
        else if (discovered_cam_idx == 1 || discovered_cam_idx == 2) {
            add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // Cancel note the --4.36 vs -+4.36
            add_to_y = my_part.pose.position.y - 1.173381; // CANCEL: note the --1.173381 vs -+1.173381
            ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

        }

        // double add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
        // double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y
        // ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

        std::vector<PresetLocation> path = getPresetLocationVector( cam_to_presetlocation[discovered_cam_idx] );
        ROS_INFO_STREAM("getPresetLocationVector executed!");

        executeVectorOfPresetLocations(path);
        ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        gantry_->goToPresetLocation(Bump(cam_to_presetlocation[discovered_cam_idx], add_to_x, add_to_y, 0));
        ROS_INFO_STREAM("goToPresetLocation with bump executed!");

        ROS_INFO_STREAM("hello3");
    }
    else if ( (discovered_cam_idx != 0 || discovered_cam_idx != 7 || discovered_cam_idx != 1 || discovered_cam_idx != 2) && discovered_cam_idx >=0 && discovered_cam_idx <=16 ) { // any other camera
        double add_to_x_shelf = my_part.pose.position.x - -13.522081; // constant is perfect bin red pulley x
        double add_to_y_shelf = my_part.pose.position.y - 3.446263;
        ROS_INFO_STREAM(" x " << add_to_x_shelf << " y " << add_to_y_shelf);

        std::vector<PresetLocation> path = getPresetLocationVector( cam_to_presetlocation[discovered_cam_idx] );
        ROS_INFO_STREAM("getPresetLocationVector executed!");

        executeVectorOfPresetLocations(path);
        ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, 0));
        ROS_INFO_STREAM("goToPresetLocation with bump executed!");

        ROS_INFO_STREAM("hello3");
        ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away

    }
    else {
        ROS_INFO_STREAM("error, the camera idx is not equal to an expected number!!");
    }

    //--Go pick the part
    if (!gantry_->pickPart(my_part, "left_arm")){
        // gantry.goToPresetLocation(gantry.start_);
        // spinner.stop();
        // ros::shutdown();
        //pass
    }

    // go back to start
    // gantry_->goToPresetLocation(start_a);
    if (true) { // if any camera
        std::vector<PresetLocation> path = getPresetLocationVector( start_a );
        executeVectorOfPresetLocations(path);
        ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves

    }


    //place the part
    // gantry_->placePart(part_in_tray, product.agv_id, "left_arm"); // problem when placing green gaskets, part is upsidedown
    gantry_->placePart(part_in_tray, product.agv_id, "left_arm");
    task_queue_.top().pop();
    ROS_INFO("Popped element");
//    gantry_->goToPresetLocation(start_a);
}


void RWAImplementation::checkAgvErrors() {
    /************** Check for Faulty Parts *****************/

}

// bool RWAImplementation::checkAndCorrectPose(){

// }




double RWAImplementation::calcDistanceInXYPlane(geometry_msgs::Pose a, geometry_msgs::Pose b) {
    return std::sqrt( std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2) ); // euclidean distance
}





//////////////////////////////// Get nearest preset location to current gantry position
PresetLocation RWAImplementation::getNearesetPresetLocation() {
    ////// Get current gantry position
    geometry_msgs::Pose current_position = gantry_->getGantryPose();

    ROS_INFO_STREAM("finished getGantry Pose successfully executed!");

    ////// Define custom struct (distance, PresetLocation), see https://www.cplusplus.com/articles/NhA0RXSz/

    ROS_INFO_STREAM("struct defined successfully");


    ////// Build vector of custom structs
    std::vector<distance_and_PresetLocation_struct> vector_of_structs;

    ROS_INFO_STREAM("std::vector of structs ran");

    ROS_INFO_STREAM("size of preset vector ==== " << preset_locations_list_.size());

    ROS_INFO_STREAM("name of 1st of preset vector ==== " << preset_locations_list_.at(0).name);


    for (PresetLocation pset_location : preset_locations_list_) {
        ROS_INFO_STREAM("for loop entered");

        geometry_msgs::Pose  location_converted_to_pose = gantryXY2worldposeXY(pset_location); // convert preset location to pose
        ROS_INFO_STREAM("location_converted_to_pose in loop ran");

        double distance_i = calcDistanceInXYPlane(current_position, location_converted_to_pose); // euclidean dist in xy plane
        ROS_INFO_STREAM("calcDistanceInXYPlane in loop ran");

        distance_and_PresetLocation_struct candidate_struct {distance_i, pset_location}; // make a struct
        ROS_INFO_STREAM("candidate_struct in loop ran");

        vector_of_structs.push_back(candidate_struct); // put struct in list of structs
        ROS_INFO_STREAM("push_back in loop ran");

    }

    ROS_INFO_STREAM("vector of custom structs defined successfully");


    ////// Sort vector of custom structs by distance (need *another* function to do this, will define a lambda function)
    std::sort(vector_of_structs.begin(), vector_of_structs.end(),
        // Lambda expression begins,
        [](const distance_and_PresetLocation_struct &lhs, const distance_and_PresetLocation_struct &rhs) {
            return (lhs.distance < rhs.distance);
        } // end of lambda expression
    );

    ROS_INFO_STREAM("sorted vector successfully");


    ////// Return the nearest PresetLocation
    return vector_of_structs[0].candidate_location;
}



std::vector<PresetLocation> RWAImplementation::getPresetLocationVector(PresetLocation target_preset_location) {
    ROS_INFO_STREAM("inside get Preset Location Vector function executed!");
    PresetLocation approximate_current_position = getNearesetPresetLocation();
    ROS_INFO_STREAM("getNearesetPresetLocation executed! =====");
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location.name}};
    ROS_INFO_STREAM("key executed! ");

    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);

    std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key);
    ROS_INFO_STREAM("path_to_execute lookup executed! =====");
    return path_to_execute;
}


bool RWAImplementation::executeVectorOfPresetLocations( std::vector<PresetLocation> path_to_execute ) {
    for (PresetLocation psetlocation : path_to_execute) {
        gantry_->goToPresetLocation(psetlocation);
    }
    return true;
}



// converts PresetLocation to a Pose, but only look at Pose's x and y, other numbers are meaningless
geometry_msgs::Pose RWAImplementation::gantryXY2worldposeXY(PresetLocation preset_location_2_convert) {
    ROS_INFO_STREAM("gantryXY 2 worldpose XY entered");

    ROS_INFO_STREAM(preset_location_2_convert.name);
    ROS_INFO_STREAM(preset_location_2_convert.gantry.empty());


    double gantry_x = preset_location_2_convert.gantry[0];
    ROS_INFO_STREAM("double gantry_x defined successfully");

    double gantry_y = preset_location_2_convert.gantry[1];
    ROS_INFO_STREAM("double gantry_y defined successfully");

    geometry_msgs::Pose converted_pose;
    ROS_INFO_STREAM("converted_pose defined successfully");

    converted_pose.position.x = gantry_x;
    ROS_INFO_STREAM("set equals defined successfully");

    converted_pose.position.y = gantry_y * -1;
    ROS_INFO_STREAM("multiplication done successfully");

    ROS_INFO_STREAM("gantryXY 2 worldpose XY ran sucessfully");


    return converted_pose;

}