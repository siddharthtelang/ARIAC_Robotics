#include "rwa_implementation.h"
#include <unordered_map>

#define GET_VARIABLE_NAME(Variable) (#Variable)

void RWAImplementation::processOrder()
{
    competition_started_ = true;
    auto order_list = competition_->get_orders_list();
    if (order_list.size() == prev_num_orders_)
        return;

    prev_num_orders_++;
    ROS_INFO_STREAM("Processing Order...");

    std::unordered_map<std::string, std::unordered_map<std::string, std::queue<Product>>> total_products;
    std::queue<std::vector<Product>> shipments;
    std::string shipment_type;

    std::string delimiter = "_";

    auto order = order_list.back();

    std::queue<std::vector<Product>> order_task_queue;
    for (const auto &shipment : order.shipments)
    {
        shipment_type = shipment.shipment_type;
        std::string agv_id = shipment.agv_id;
        std::vector<Product> products = shipment.products;
        std::queue<std::string> current_shipment;

        ROS_INFO_STREAM("Shipment type = " << shipment_type);
        ROS_INFO_STREAM("AGV ID = " << agv_id);

        ROS_INFO_STREAM("Shipment size = " << order.shipments.size());

        std::queue<Product> conveyor_parts;
        std::vector<Product> shipment_task_queue;
        for (auto &product : products)
        { ////////// FOR PRODUCT IN PRODUCTS
            ROS_INFO_STREAM("Product type = " << product.type);
            product.agv_id = agv_id;
            std::string color = product.type;
            std::string type = color.substr(0, color.find(delimiter));
            int pos{};
            product.shipment_type = shipment_type;

            while ((pos = color.find(delimiter)) != std::string::npos)
            {
                color.erase(0, pos + delimiter.length());
            }
            auto full_type = type + "_part_" + color;
            if (!sorted_map[color][type].empty())
            {
                product.designated_model = sorted_map[color][type].top();
                sorted_map[color][type].pop();
                shipment_task_queue.push_back(product);
                ROS_INFO_STREAM(" !sorted_map[color][type].empty() if case entered ");
            }
            else if (full_type == conveyor_type_)
            {
                product.get_from_conveyor = true;
                conveyor_parts.push(product);
                ROS_INFO_STREAM(" full_type == conveyor_type if case entered ");
            }
            else
            {
                product.get_from_conveyor = true;
                conveyor_type_ = full_type;
                conveyor_parts.push(product);
                ROS_INFO_STREAM(" else case entered ");
            }
            // total_products[color][type].push(product);
        }
        while (!conveyor_parts.empty())
        {
            auto product = conveyor_parts.front();
            shipment_task_queue.push_back(product);
            conveyor_parts.pop();
        }
        order_task_queue.push(shipment_task_queue);
    }
    /* ORDERS PRODUCTS IN PAIRS. DON'T DELETE */
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
//     ROS_INFO_STREAM("");
//     auto task_queue = task_queue_.top();
//     while(!task_queue.empty()) {
//         auto vec = task_queue.front();
//         ROS_INFO_STREAM("Vector on queue.");
//         for(auto product : vec) {
//             ROS_INFO_STREAM("  Product on queue.");
//             ROS_INFO_STREAM("  Product type: " << product.type);
//             ROS_INFO_STREAM("  Product target AGV: " << product.agv_id);
//             ROS_INFO_STREAM("    Designated model id: " << product.designated_model.id);
//             ROS_INFO_STREAM("    Designated model type: " << product.designated_model.color << " " << product.designated_model.type << "\n");
//         }
//         task_queue.pop();
//     }
}

/**
 * \brief: Main for rwa3 node
 * \param: argc
 * \param: argv
 * \result: Applies control of gantry robot
 */
PresetLocation Bump(PresetLocation location_to_modify, double small_rail, double large_rail, double torso)
{
    location_to_modify.gantry.at(0) = location_to_modify.gantry.at(0) + small_rail;
    location_to_modify.gantry.at(1) = location_to_modify.gantry.at(1) - large_rail; // Minus now, does simulation switch?
    location_to_modify.gantry.at(2) = location_to_modify.gantry.at(2) + torso;

    return location_to_modify;
}

bool RWAImplementation::checkConveyor()
{
    bool continue_ = false;
    ROS_INFO_STREAM("Checking conveyor...");

    if (!waiting_for_part_ && !cam_listener_->load_time_on_conveyor_.empty() && !cam_listener_->parts_on_conveyor_.empty())
    {
        waiting_for_part_ = true;
        current_part_load_time_ = cam_listener_->load_time_on_conveyor_.front();
        current_part_on_conveyor_ = cam_listener_->parts_on_conveyor_.front();
        cam_listener_->load_time_on_conveyor_.pop();
        cam_listener_->parts_on_conveyor_.pop_front();
    }
    if (buffer_parts_collected < buffer_parts_ && waiting_for_part_)
    {
        ROS_INFO_STREAM("Get Cam Part, buffer parts count: " << buffer_parts_collected);
        if ((ros::Time::now() - current_part_load_time_).toSec() > dx_ / cam_listener_->conveyor_spd_ - 4)
        {
            ROS_INFO_STREAM("Too late for conveyor part. Checking again.");
            waiting_for_part_ = false;
            // return false;
            checkConveyor();
            return true;
        }
        gantry_->goToPresetLocation(conveyor_belt);
        while ((ros::Time::now() - current_part_load_time_).toSec() < dx_ / cam_listener_->conveyor_spd_)
        {
            ros::Duration(0.1).sleep();
        }
        ROS_INFO_STREAM("Pick up part now!");
        part temp_part;
        temp_part.type = current_part_on_conveyor_.type + "_part_" + current_part_on_conveyor_.color;
        temp_part.pose = current_part_on_conveyor_.world_pose;
        temp_part.pose.position.y = -2.5;
        waiting_for_part_ = false;

        if (!gantry_->pickPart(temp_part, "left_arm"))
            return true;

        gantry_->goToPresetLocation(start_a);
        // Place part on shelf
        // gantry_->goToPresetLocation(loc);
        temp_part.initial_pose = current_part_on_conveyor_.world_pose;
        // gantry_->placePart(temp_part, "agv1", "left_arm"); // need to put in drop off part at bins here
        // drop parts into bins,
        PresetLocation drop_location = getNearestBinPresetLocation();
        gantry_->goToPresetLocation(Bump(drop_location, 0.0, -0.8, 0));
        simpleDropPart();

        buffer_parts_collected++;
        continue_ = true;
    }
    return continue_;
}

void RWAImplementation::initPresetLocs()
{
    conveyor_belt.gantry = {0, 3, PI / 2};
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
    agv1_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    agv1_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same except for joint 0
    agv1_staging_a.name = GET_VARIABLE_NAME(agv1_staging_a);

    bottom_left_staging_a.gantry = {-14.22, -6.75, 0.00};
    bottom_left_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    bottom_left_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same except for joint 0
    bottom_left_staging_a.name = GET_VARIABLE_NAME(bottom_left_staging_a);

    // shelf5_a.gantry = {-14.22, -4.15, 0.00};
    shelf5_a.gantry = {-14.42, -4.30, 0.00}; // WORKS FOR LEFT SHELF PULLEY NOT RIGHT
    // shelf5_a.gantry = {-14.72, -4.30, 0.00};
    // shelf5_a.gantry = {-14.42 - .897919, -4.30 - .853737, 0.00};
    // shelf5_a.gantry = {-15.42, -4.30, 0.00}; // WORKS FOR RIGHT SHELF PULLEY
    // shelf5_a.left_arm = {-PI/2, -1.01, 1.88, -1.13, 0.00, 0.00}; // higher up
    // shelf5_a.left_arm = {-1.64, -0.99, 1.84, -.85, -.08, -.26};
    shelf5_a.left_arm = {-1.76, -1.00, 1.86, -.85, -.20, 0.0};     // try fix
    shelf5_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf5_a.name = GET_VARIABLE_NAME(shelf5_a);

    shelf5_spun_a.gantry = {-15.42, -4.30, 3.14};
    shelf5_spun_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf5_spun_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf5_spun_a.name = GET_VARIABLE_NAME(shelf5_spun_a);

    mid_5_8_staging_a.gantry = {0.0, -1.5, 0.00};
    mid_5_8_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.name = GET_VARIABLE_NAME(mid_5_8_staging_a);

    mid_8_11_staging_a.gantry = {0.0, 1.5, 0.00};
    mid_8_11_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.name = GET_VARIABLE_NAME(mid_8_11_staging_a);

    shelf8_a.gantry = {-14.22, -1.5, 0.00};
    shelf8_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf8_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf8_a.name = GET_VARIABLE_NAME(shelf8_a);

    shelf11_a.gantry = {-14.22, 1.5, 0.00};
    shelf11_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf11_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
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
        {9, shelf5_a},
        {12, shelf5_a},

        {6, shelf8_a},
        {16, shelf8_a},

        {8, shelf11_a},
        {11, shelf11_a},

        {1, bin11_a},
        {2, bin11_a},
    };

    agv_to_camera = {
        {"agv1", 3},
        {"agv2", 4}};

    // vector of some of the locations, when gantry moves anywhere, it first searches through these possible locations,
    // and tries to find the closest one to it. That approximate location then serves as the beginning location for any move lookup.
    preset_locations_list_ = {start_a, bin3_a, agv2_a, agv1_staging_a,
                              bottom_left_staging_a, shelf8_a, shelf11_a, bin11_a, shelf5_a}; // do not have mid_xyz anything here for now

    PathingLookupDictionary = {
        {{"start_a", "bin3_a"}, std::vector<PresetLocation>{start_a, bin3_a}},
        {{"start_a", "agv2_a"}, std::vector<PresetLocation>{start_a, agv2_a}},
        {{"start_a", "agv1_staging_a"}, std::vector<PresetLocation>{start_a, agv1_staging_a}},
        {{"start_a", "bottom_left_staging_a"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a}},
        {{"start_a", "shelf8_a"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_a, shelf8_a}},
        {{"start_a", "shelf11_a"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_a, shelf11_a}},
        {{"start_a", "bin11_a"}, std::vector<PresetLocation>{start_a, bin11_a}},

        {{"bin3_a", "start_a"}, std::vector<PresetLocation>{bin3_a, start_a}},
        {{"agv2_a", "start_a"}, std::vector<PresetLocation>{agv2_a, start_a}},
        {{"agv1_staging_a", "start_a"}, std::vector<PresetLocation>{agv1_staging_a, start_a}},
        {{"bottom_left_staging_a", "start_a"}, std::vector<PresetLocation>{bottom_left_staging_a, agv1_staging_a, start_a}},
        {{"shelf5_a", "start_a"}, std::vector<PresetLocation>{bottom_left_staging_a, agv1_staging_a, start_a}}, // added for shelf 5 bumping into shelves, go direct
        {{"shelf8_a", "start_a"}, std::vector<PresetLocation>{shelf8_a, mid_5_8_staging_a, start_a}},
        {{"shelf11_a", "start_a"}, std::vector<PresetLocation>{shelf11_a, shelf11_a, shelf11_a, mid_8_11_staging_a, start_a}}, //GO to shelf 11a!
        {{"bin11_a", "start_a"}, std::vector<PresetLocation>{bin11_a, start_a}},

        {{"agv1_staging_a", "bin3_a"}, std::vector<PresetLocation>{start_a, bin3_a}}, // go to start_a first /////////// this block: do not go to first point first
        {{"agv1_staging_a", "agv2_a"}, std::vector<PresetLocation>{agv2_a}},
        {{"agv1_staging_a", "agv1_staging_a"}, std::vector<PresetLocation>{agv1_staging_a}}, // go to itself
        {{"agv1_staging_a", "bottom_left_staging_a"}, std::vector<PresetLocation>{agv1_staging_a, bottom_left_staging_a}},
        {{"agv1_staging_a", "shelf8_a"}, std::vector<PresetLocation>{mid_5_8_staging_a, shelf8_a}},
        {{"agv1_staging_a", "shelf11_a"}, std::vector<PresetLocation>{mid_8_11_staging_a, shelf11_a}},
        {{"agv1_staging_a", "bin11_a"}, std::vector<PresetLocation>{start_a, bin11_a}}, // go to start_a first

        {{"agv2_a", "bin3_a"}, std::vector<PresetLocation>{start_a, bin3_a}}, // go to start_a first /////////////// this block: do not go to first point first
        {{"agv2_a", "agv2_a"}, std::vector<PresetLocation>{agv2_a}},          // go to itself
        {{"agv2_a", "agv1_staging_a"}, std::vector<PresetLocation>{agv1_staging_a}},
        {{"agv2_a", "bottom_left_staging_a"}, std::vector<PresetLocation>{agv1_staging_a, bottom_left_staging_a}},
        {{"agv2_a", "shelf8_a"}, std::vector<PresetLocation>{mid_5_8_staging_a, shelf8_a}},
        {{"agv2_a", "shelf11_a"}, std::vector<PresetLocation>{mid_8_11_staging_a, shelf11_a}},
        {{"agv2_a", "bin11_a"}, std::vector<PresetLocation>{start_a, bin11_a}}, // go to start_a first

        {{"bin3_a", "bin11_a"}, std::vector<PresetLocation>{bin3_a, bin11_a}},
        {{"bin3_a", "bin3_a"}, std::vector<PresetLocation>{bin3_a, bin3_a}},

    };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RWAImplementation::buildKit()
{
    if (task_queue_.top().empty())
    {
        ROS_INFO("Task queue is empty. Return");
        return;
    }
    // // may need to uncomment this, but likely not. do not need to be at start to pick, only to place,
    // // (since place part returns to start)
    // gantry_->goToPresetLocation(start_a);

    Product product = task_queue_.top().front()[0];
    part my_part;
    my_part.type = product.type;
    my_part.pose = product.designated_model.world_pose;

    part part_in_tray;
    part_in_tray.type = product.type;
    part_in_tray.pose = product.pose;
    part_in_tray.initial_pose = product.designated_model.world_pose;
    part_in_tray.target_pose = gantry_->getTargetWorldPose(part_in_tray.pose, product.agv_id);

    double add_to_x = my_part.pose.position.x - 4.465789; // constant is perfect bin red pulley x
    double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y

    ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

    int discovered_cam_idx = product.designated_model.cam_index;

    ROS_INFO_STREAM("product.type === " << product.type);
    ROS_INFO_STREAM("product.get_from_conveyor === " << product.get_from_conveyor);

    // if (true) { // if part was moved from conveyor to the bin
    if (product.get_from_conveyor == true)
    { // if part was moved from conveyor to the bin
        ROS_INFO_STREAM("Deal with conveyor part _______________________");
        std::vector<CameraListener::ModelInfo> cam_parts_repoll = cam_listener_->fetchPartsFromCamera(*node_, 7);

        if(cam_parts_repoll.empty()) return;
        CameraListener::ModelInfo model_info_conveyor_part = cam_parts_repoll[0]; // todo deal with not enough saved up conveyor parts

        my_part.pose.position.x = model_info_conveyor_part.world_pose.position.x; // seems very redundant to store in the modelInfo and part
        my_part.pose.position.y = model_info_conveyor_part.world_pose.position.y;
        my_part.pose.position.z = model_info_conveyor_part.world_pose.position.z;
        my_part.pose.orientation.x = model_info_conveyor_part.world_pose.orientation.x;
        my_part.pose.orientation.y = model_info_conveyor_part.world_pose.orientation.y;
        my_part.pose.orientation.z = model_info_conveyor_part.world_pose.orientation.z;
        my_part.pose.orientation.w = model_info_conveyor_part.world_pose.orientation.w;

        part_in_tray.initial_pose = model_info_conveyor_part.world_pose; // save the initial pose

        my_part.type = model_info_conveyor_part.type + "_part_" + model_info_conveyor_part.color; // correct mismatch later
        double add_to_x = my_part.pose.position.x - 4.365789 - 0.1;                         // constant is perfect bin red pulley x
        double add_to_y = my_part.pose.position.y - 1.173381;                               // constant is perfect bin red pulley y

        std::vector<PresetLocation> path = getPresetLocationVector(cam_to_presetlocation[7]);
        ROS_INFO_STREAM("getPresetLocationVector executed!");

        executeVectorOfPresetLocations(path);
        ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        gantry_->goToPresetLocation(Bump(cam_to_presetlocation[7], add_to_x, add_to_y, 0));
        ROS_INFO_STREAM("goToPresetLocation with bump executed!");

        buffer_parts_collected--; // this should occur later in future, after part is actually picked up
    }
    else if (discovered_cam_idx == 0 || discovered_cam_idx == 7 || discovered_cam_idx == 1 || discovered_cam_idx == 2)
    {

        double add_to_x = 0.0;
        double add_to_y = 0.0;
        if (discovered_cam_idx == 7 || discovered_cam_idx == 0)
        {
            add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
            add_to_y = my_part.pose.position.y - 1.173381;       // constant is perfect bin red pulley y
            ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);
        }
        else if (discovered_cam_idx == 1 || discovered_cam_idx == 2)
        {
            add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // Cancel note the --4.36 vs -+4.36
            add_to_y = my_part.pose.position.y - 1.173381;       // CANCEL: note the --1.173381 vs -+1.173381
            ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);
        }

        // double add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
        // double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y
        // ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

        std::vector<PresetLocation> path = getPresetLocationVector(cam_to_presetlocation[discovered_cam_idx]);
        ROS_INFO_STREAM("getPresetLocationVector executed!");

        executeVectorOfPresetLocations(path);
        ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        gantry_->goToPresetLocation(Bump(cam_to_presetlocation[discovered_cam_idx], add_to_x, add_to_y, 0));
        ROS_INFO_STREAM("goToPresetLocation with bump executed!");
    }
    else if ((discovered_cam_idx != 0 || discovered_cam_idx != 7 || discovered_cam_idx != 1 || discovered_cam_idx != 2) && discovered_cam_idx >= 0 && discovered_cam_idx <= 16)
    {                                                                 // any other camera
        double add_to_x_shelf = my_part.pose.position.x - -13.522081; // constant is perfect bin red pulley x
        double add_to_y_shelf = my_part.pose.position.y - 3.446263;
        ROS_INFO_STREAM(" x " << add_to_x_shelf << " y " << add_to_y_shelf);

        std::vector<PresetLocation> path = getPresetLocationVector(cam_to_presetlocation[discovered_cam_idx]);
        ROS_INFO_STREAM("getPresetLocationVector executed!");

        executeVectorOfPresetLocations(path);
        ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, 0));
        ROS_INFO_STREAM("goToPresetLocation with bump executed!");

        ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
    }
    else
    {
        ROS_INFO_STREAM("error, the camera idx is not equal to an expected number!!");
    }

    //--Go pick the part
    if (!gantry_->pickPart(my_part, "left_arm"))
    {
        // gantry.goToPresetLocation(gantry.start_);
        // spinner.stop();
        // ros::shutdown();
        //pass
    }

    // go back to start
    // gantry_->goToPresetLocation(start_a);
    if (true)
    { // if any camera
        std::vector<PresetLocation> path = getPresetLocationVector(start_a);
        executeVectorOfPresetLocations(path);
        ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves
    }

    // // testing drop parts into bins, uncomment this whole block, comment out entire next block
    // PresetLocation drop_location = getNearestBinPresetLocation();
    // gantry_->goToPresetLocation( Bump( drop_location, 0.0, -0.8, 0) );
    // simpleDropPart();

    //place the part
    // gantry_->placePart(part_in_tray, product.agv_id, "left_arm"); // problem when placing green gaskets, part is upsidedown

    // check if part needs to be flipped and if yes, go to the agv and flip if before placing
    bool success = checkForFlip(part_in_tray, product);
    if (success)
        gantry_->placePart(part_in_tray, product.agv_id, "right_arm");
    else
        gantry_->placePart(part_in_tray, product.agv_id, "left_arm");

    //    task_queue_.top().front().erase(task_queue_.top().front().begin());
    //    ROS_INFO("Popped element");

    //assign camera an index based on its location on AGVs - either on agv1 or agv2
    int camera = product.agv_id == "agv1" ? 0 : 1;
    //add the placed Product this to array of placed products - for now only consider one part is placed
    parts_in_tray[camera].push_back(part_in_tray);

    //    task_queue_.top().front().erase(task_queue_.top().front().begin());
    //    ROS_INFO("Popped element");
    //    gantry_->goToPresetLocation(start_a); // do not need to go back to start after placing part. only after picking part.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double RWAImplementation::calcDistanceInXYPlane(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    return std::sqrt(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2)); // euclidean distance
}

//////////////////////////////// Get nearest preset location to current gantry position
PresetLocation RWAImplementation::getNearesetPresetLocation()
{
    ////// Get current gantry position
    geometry_msgs::Pose current_position = gantry_->getGantryPose();

    ////// Define custom struct (distance, PresetLocation), see https://www.cplusplus.com/articles/NhA0RXSz/

    ////// Build vector of custom structs
    std::vector<distance_and_PresetLocation_struct> vector_of_structs;

    for (PresetLocation pset_location : preset_locations_list_)
    {
        geometry_msgs::Pose location_converted_to_pose = gantryXY2worldposeXY(pset_location);    // convert preset location to pose
        double distance_i = calcDistanceInXYPlane(current_position, location_converted_to_pose); // euclidean dist in xy plane
        distance_and_PresetLocation_struct candidate_struct{distance_i, pset_location};          // make a struct
        vector_of_structs.push_back(candidate_struct);                                           // put struct in list of structs
    }

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

std::vector<PresetLocation> RWAImplementation::getPresetLocationVector(PresetLocation target_preset_location)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation();
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location.name}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);
    std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key);
    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

bool RWAImplementation::executeVectorOfPresetLocations(std::vector<PresetLocation> path_to_execute)
{
    for (PresetLocation psetlocation : path_to_execute)
    {
        gantry_->goToPresetLocation(psetlocation);
    }
    return true;
}

// converts PresetLocation to a Pose, but only look at Pose's x and y, other numbers are meaningless
geometry_msgs::Pose RWAImplementation::gantryXY2worldposeXY(PresetLocation preset_location_2_convert)
{

    double gantry_x = preset_location_2_convert.gantry[0];
    double gantry_y = preset_location_2_convert.gantry[1];
    geometry_msgs::Pose converted_pose;
    converted_pose.position.x = gantry_x;
    converted_pose.position.y = gantry_y * -1;

    return converted_pose;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PresetLocation RWAImplementation::getNearestBinPresetLocation()
{
    // [xlow, xhigh, ylow, yhigh]
    std::vector<std::vector<double>> TwoPointsList = {
        {2.3, 2.957942, 1.842058, 2.5},
        {2.3, 2.957942, 1.0, 1.657942},
        {2.3, 2.957942, -1.657942, -1.0},
        {2.3, 2.957942, -2.5, -1.842058},

        {3.14, 3.797, 1.842058, 2.5},
        {3.14, 3.797, 1.0, 1.657942},
        {3.14, 3.797, -1.657942, -1.0},
        {3.14, 3.797, -2.5, -1.842058},

        {3.98, 4.638, 1.842058, 2.5},
        {3.98, 4.638, 1.0, 1.657942},
        {3.98, 4.638, -1.657942, -1.0},
        {3.98, 4.638, -2.5, -1.842058},

        {4.82, 5.478, 1.842058, 2.5},
        {4.82, 5.478, 1.0, 1.657942},
        {4.82, 5.478, -1.657942, -1.0},
        {4.82, 5.478, -2.5, -1.842058},
    };

    // poll cameras
    auto cam_parts = cam_listener_->fetchParts(*node_); //  returns std::array<std::vector<CameraListener::ModelInfo>, 17>

    // create a flat parts list vector
    std::vector<CameraListener::ModelInfo> flat_parts_list;
    for (auto cam_part_array : cam_parts)
    {
        for (auto model_info : cam_part_array)
        {
            flat_parts_list.push_back(model_info);
        }
    }

    std::vector<std::vector<double>> ValidPointsList = TwoPointsList;

    PresetLocation dropPresetLocation;
    // dropPresetLocation.gantry = {ValidPointsList[0][0] + 0.3, (ValidPointsList[0][2])*-1.0 , 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
    // dropPresetLocation.gantry = {ValidPointsList[0][0] + 0.0, (ValidPointsList[0][2])*-1.0 , 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
    dropPresetLocation.gantry = {2.3 - 0.3, (1.842058) * -1.0, 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
    dropPresetLocation.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    dropPresetLocation.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // ROS_INFO_STREAM( "target Xcoord == " << ValidPointsList[0][0] + 0.0 << " target Ycoord == " << (ValidPointsList[0][2])*-1.0);
    ROS_INFO_STREAM("target [xlow,xhigh,ylow,yhigh]] == " << ValidPointsList[0][0] << " " << ValidPointsList[0][1] << " " << ValidPointsList[0][2] << " " << ValidPointsList[0][3]);

    return dropPresetLocation;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RWAImplementation::checkAgvErrors()
{
    /************** Check for Faulty Parts *****************/
    if (task_queue_.top().empty())
    {
        std::cout << "[Faulty Parts]: No parts to check" << std::endl;
        return;
    }
    Product product = task_queue_.top().front()[0];
    cam_listener_->checkFaulty(*node_, product.agv_id);
    ros::Duration(0.5).sleep(); // make sure it actually goes back to start, instead of running into shelves

    int camera_index = product.agv_id == "agv1" ? 0 : 1;

    if (cam_listener_->faulty_parts)
    {
        ROS_INFO("Detected Faulty Part");
        for (int f_p = 0; f_p < cam_listener_->faulty_parts_list.size(); ++f_p)
        {
            CameraListener::ModelInfo faulty = cam_listener_->faulty_parts_list.at(2);
            Part faulty_part;
            faulty_part.pose = faulty.model_pose;
            ROS_INFO_STREAM("Faulty product pose:" << faulty_part.pose);
            faulty_part.type = product.type;
            faulty_part.pose = product.pose;
            ROS_INFO_STREAM("Faulty Part:" << faulty_part.type);
            cam_listener_->faulty_parts_list.clear();
            bool success = gantry_->replaceFaultyPart(faulty_part, product.agv_id, "left_arm");
            if (success)
            {
                cam_listener_->faulty_parts_list.clear();
                // look for the new part
                if (!sorted_map[product.designated_model.color][product.designated_model.type].empty())
                {
                    //update the designated model
                    product.designated_model = sorted_map[product.designated_model.color][product.designated_model.type].top();
                    //pick new model
                    sorted_map[product.designated_model.color][product.designated_model.type].pop();
                }
                else
                    product.get_from_conveyor = true;
                // remove the faulty product
                task_queue_.top().front().erase(task_queue_.top().front().begin());
                // look for the replacement product
                task_queue_.top().front().insert(task_queue_.top().front().begin(), product);
                //parts_in_tray[camera] needs to be popped if faulty
                parts_in_tray[camera_index].pop_back();
                // go back to start
                gantry_->goToPresetLocation(start_a);
            }
        }
    }

    /************** Check for Flipped Parts *****************/
    /* Comment this for now. Implement another versio of flip part. Delete this commented part later
    tf2::Quaternion q(
            product.pose.orientation.x,
            product.pose.orientation.y,
            product.pose.orientation.z,
            product.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (abs(roll + pi)< 0.1) {
        // flip the part
        Part part_in_tray;
        part_in_tray.pose = product.pose;
        part_in_tray.type = product.type;
        part_in_tray.initial_pose = product.designated_model.world_pose; // save the initial pose
        gantry_->flipPart(part_in_tray, product.agv_id);
    }
    */

    bool poseUpdated = false;
    if (!cam_listener_->faulty_parts)
    {
        /* if no faulty part, check for poses, if correctly placed then pop from task_queue_.top(),
        pop the products from current shipment list  */
        poseUpdated = checkAndCorrectPose(product.agv_id);
        task_queue_.top().front().erase(task_queue_.top().front().begin());
    }

    if (task_queue_.top().front().empty())
    {
        ros::Duration(1.0).sleep(); // add delay
        // one shipment completed. Send to AGV
        AGVControl agv_control(*node_);
        std::string kit_id = (product.agv_id == "agv1" ? "kit_tray_1" : "kit_tray_2");
        agv_control.sendAGV(product.shipment_type, kit_id);
        task_queue_.top().pop();

        if(task_queue_.top().empty()) 
            task_queue_.top();
    }

    gantry_->goToPresetLocation(start_a);
}

bool RWAImplementation::checkAndCorrectPose(std::string agv_id)
{
    //part(s) part_to_check
    auto placed_parts = parts_in_tray[agv_id == "agv1" ? 0 : 1];
    //current list from camera corresponding to agv
    ros::Duration(0.5).sleep();
    auto list_from_camera = cam_listener_->fetchPartsFromCamera(*node_, agv_to_camera[agv_id]);

    ROS_INFO("---------Check pose in tray------------");
    float delta_x = 0.0, delta_y = 0.0, delta_z = 0.0;
    bool incorrect_pose = false, match = false;

    std::vector<CameraListener::ModelInfo> model_to_reorient;
    std::vector<part> part_to_reorient;
    part Part;
    CameraListener::ModelInfo model;

    ROS_INFO("========================================== COMPARE POSE START ==========================================");

    //for each part reported by logical camera check if there is any part we placed on tray with the same target position
    //if the target position of the part in tray do not match with that of reported by camera, re-orientation is required
    for (auto model : list_from_camera)

    {
        for (int i = 0; i < placed_parts.size(); ++i)
        {
            auto part_to_check = placed_parts[i];
            match = false;
            if ((model.type + "_part_" + model.color) != part_to_check.type)
                continue;
            else
            {
                double roll, roll_, pitch, pitch_, yaw_desired, yaw_current;
                tf2::Quaternion q_current(
                     model.world_pose.orientation.x,
                     model.world_pose.orientation.y,
                     model.world_pose.orientation.z,
                     model.world_pose.orientation.w);
                tf2::Matrix3x3 m(q_current);
                m.getRPY(roll, pitch, yaw_current);
                //ROS_INFO_STREAM("YAW CURRENT = "<< yaw_current);
                //Desired yaw
                tf2::Quaternion q_desired(
                     part_to_check.target_pose.orientation.x,
                     part_to_check.target_pose.orientation.y,
                     part_to_check.target_pose.orientation.z,
                     part_to_check.target_pose.orientation.w);
                tf2::Matrix3x3 m_(q_desired);
                m_.getRPY(roll_, pitch_, yaw_desired);
                //ROS_INFO_STREAM("YAW DESIRED = "<< yaw_desired);

                ROS_INFO_STREAM("\nCHECK FOR PART " << part_to_check.type);
                //correct world pose in tray as per order
                auto correct_world_pose = part_to_check.target_pose;
                //check delta between the above and the actual position determined by camera

                auto delta_x = std::abs(std::abs(model.world_pose.position.x) - std::abs(correct_world_pose.position.x));
                auto delta_y = std::abs(std::abs(model.world_pose.position.y) - std::abs(correct_world_pose.position.y));
                auto delta_z = std::abs(std::abs(model.world_pose.position.z) - std::abs(correct_world_pose.position.z));
                //auto delta_deg = std::abs(model.world_pose.orientation.z) - std::abs(correct_world_pose.orientation.z);
                auto delta_deg = std::abs(std::abs(yaw_desired) - std::abs(yaw_current));

                ROS_INFO_STREAM("CURRENT WORLD POSE IN TRAY = " << model.world_pose);
                ROS_INFO_STREAM("DESIRED WORLD POSE = " << correct_world_pose);
                ROS_INFO_STREAM("Deltas = " << delta_x << " ; " << delta_y << " ; " << delta_z << " ; " << delta_deg);

                /* if (delta_x <= 0.15 && delta_y <= 0.15 && delta_z <= 0.20 && delta_deg <= 0.02)  */
                // update this to previous version of x,y,z delta conditions if regression fails
                // As per competition, the max delta can be 3cm and 5 degrees to get pose score
                if (delta_x <= 0.03 && delta_y <= 0.03 && delta_z <= 0.20 && delta_deg <= 0.1) // 0.1 radians = 5 degrees
                {
                    match = true;
                    ROS_INFO("Position is matched ; skip this and continue to next\n");
                    //remove this part from list to be checked. The one remaining will be the part to be reoriented
                    placed_parts.erase(placed_parts.begin() + i);
                    --i;
                    break;
                }
            }
        }
        if (match == false)
        { // no match found for this model reported by camera. This needs to be re-oriented
            model_to_reorient.push_back(model);
        }
    }

    ROS_INFO("GO TO PRESET LOCATION");
    PresetLocation agv = agv_id == "agv1" ? gantry_->agv1_ : gantry_->agv2_;
    gantry_->goToPresetLocation(agv);

    ROS_INFO_STREAM("Size of final lists = " << model_to_reorient.size() << " and " << placed_parts.size());

    for (int i = 0; i < model_to_reorient.size(); i++)
    {                                     //----
        gantry_->goToPresetLocation(agv); //----
        for (int j = 0; j < placed_parts.size(); j++)
        {
            if (placed_parts[j].type == model_to_reorient[i].type + "_part_" + model_to_reorient[i].color)
            {
                ROS_INFO_STREAM(" PART AND POSE TO ORIENT = " << placed_parts[j].type << " ; " << model_to_reorient[i].world_pose);
                //setup the part to be picked - update the pose as the current world pose in tray
                Part = placed_parts[j];
                Part.pose = model_to_reorient[i].world_pose;
                ROS_INFO("PICKING NOW");
                if (gantry_->pickPart(Part, "left_arm"))
                {
                    ROS_INFO("PICK SUCCESS, NOW PLACE");
                    ROS_INFO_STREAM("World Orientation before = " << Part.pose.orientation);
                    //setup the part to be placed - now the initial pose would be the current pose on the tray in world frame
                    //previously initial_pose was the pose on bin/shelf/pulley, which needs to be updated
                    part temp = placed_parts[j];
                    temp.initial_pose.orientation = model_to_reorient[i].world_pose.orientation;
                    ROS_INFO_STREAM("Desired Orientation = " << temp.pose.orientation);
                    gantry_->placePartAtCorrectPose(temp, agv_id, "left_arm");
                }
            }
        }
    }

    ROS_INFO("========================================== COMPARE POSE END ==========================================");

    return true;
}

bool RWAImplementation::competition_over() {
    if (competition_started_ && task_queue_.empty()){
        ROS_INFO_STREAM("Competition ending...");
        return true;
    }
    return false;
}

bool RWAImplementation::checkForFlip(part &part_in_tray, Product product)
{
    bool success = false;
    if (product.type == "pulley_part_red")
    {
        double roll, pitch, yaw, roll_desired;
        tf2::Quaternion q_current(
            product.designated_model.world_pose.orientation.x,
            product.designated_model.world_pose.orientation.y,
            product.designated_model.world_pose.orientation.z,
            product.designated_model.world_pose.orientation.w);
        tf2::Matrix3x3 m(q_current);
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_desired(
            product.pose.orientation.x,
            product.pose.orientation.y,
            product.pose.orientation.z,
            product.pose.orientation.w);
        tf2::Matrix3x3 m_(q_desired);
        m_.getRPY(roll_desired, pitch, yaw);

        tf2::Quaternion final_orientation;

        if (std::abs(roll - roll_desired) > 3.0)
        {
            ROS_INFO("Flip Part needed");
            double roll_zero = 0.0;
            final_orientation.setRPY(roll_zero, pitch, yaw);
            final_orientation.normalize();

            if (roll > 3.0)
            {
                ROS_INFO("Part was flipped at origin, need to correct at destination");
                product.designated_model.world_pose.orientation.x = final_orientation[0];
                product.designated_model.world_pose.orientation.y = final_orientation[1];
                product.designated_model.world_pose.orientation.z = final_orientation[2];
                product.designated_model.world_pose.orientation.w = final_orientation[3];
            }
            else
            {
                ROS_INFO("Part is to be flipped at destination");
                product.pose.orientation.x = final_orientation[0];
                product.pose.orientation.y = final_orientation[1];
                product.pose.orientation.z = final_orientation[2];
                product.pose.orientation.w = final_orientation[3];
            }
            part_in_tray.pose = product.pose;
            part_in_tray.type = product.type;
            part_in_tray.initial_pose = product.designated_model.world_pose;
            gantry_->flipPart(part_in_tray, product.agv_id);

            success = true;
        }
    }
    return success;
}