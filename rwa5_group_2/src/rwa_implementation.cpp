#include "rwa_implementation.h"
#include <unordered_map>

#include <algorithm>



void RWAImplementation::processOrder()
{
    competition_started_ = true;
    auto order_list = competition_->get_orders_list();
    if (order_list.size() == prev_num_orders_)
        return;

    ROS_INFO_STREAM("Processing Order...");

    // if(prev_num_orders_ > 0) {
    //     gantry_->goToPresetLocation(start_a);
    //     auto agv1_parts = cam_listener_->fetchPartsFromCamera(*node_, agv_to_camera["agv1"]);
    //     for(auto agv1_part : agv1_parts) {
    //         gantry_->goToPresetLocation(agv1_staging_a);            
    //         part temp_part;
    //         temp_part.type = agv1_part.type + "_part_" + agv1_part.color;
    //         temp_part.pose = agv1_part.world_pose;
    //         if (!gantry_->pickPart(temp_part, "left_arm")) return;
    //         gantry_->goToPresetLocation(start_a);
    //         PresetLocation drop_location = getNearestBinPresetLocation();
    //         gantry_->goToPresetLocation(Bump(drop_location, 0.0, -0.8, 0));
    //         simpleDropPart();
    //         gantry_->goToPresetLocation(start_a);

    //     }
    // }

    prev_num_orders_++;
    std::unordered_map<std::string, std::unordered_map<std::string, std::queue<Product>>> total_products;
    std::queue<std::vector<Product>> shipments;
    std::string shipment_type;
    std::string agv_id;
    std::string delimiter = "_";

    auto order = order_list.back();

    int count{0};
    std::queue<std::vector<Product>> order_task_queue;
    for (const auto &shipment : order.shipments)
    {
        shipment_type = shipment.shipment_type;
        agv_id = shipment.agv_id;
        std::vector<Product> products = shipment.products;
        std::queue<std::string> current_shipment;

        ROS_INFO_STREAM("Shipment type = " << shipment_type);
        ROS_INFO_STREAM("AGV ID = " << agv_id);

        ROS_INFO_STREAM("Shipment size = " << order.shipments.size());

        std::queue<Product> conveyor_parts;
        std::vector<Product> shipment_task_queue;
        for (auto &product : products)
        { ////////// FOR PRODUCT IN PRODUCTS
            count++;
            ROS_INFO_STREAM("Product type = " << product.type);
            product.agv_id = agv_id;
            auto color_type = cam_listener_->getColorType(product.type);
            auto color{color_type[0]};
            auto type{color_type[1]};

            product.shipment_type = shipment_type;

            auto full_type = type + "_part_" + color;
            if (!sorted_map[color][type].empty())
            {
                product.designated_model = sorted_map[color][type].top();
                sorted_map[color][type].pop();
                shipment_task_queue.push_back(product);
                ROS_INFO_STREAM(" Product found on map ");
                ROS_INFO_STREAM(" Designated Model ID: " << product.designated_model.id);
            }
            
            else if (full_type == conveyor_type_)
            {
                product.get_from_conveyor = true;
                // product.get_from_conveyor = false; // CRITICAL CHANGE FOR TESTING DELETE/COMMENT THIS LINE
                conveyor_parts.push(product);
                ROS_INFO_STREAM(" Type is conveyor type ");
            }
            else
            {
                product.get_from_conveyor = true;
                // product.get_from_conveyor = false; // CRITICAL CHANGE FOR TESTING DELETE/COMMENT THIS LINE
                conveyor_type_ = full_type;
                conveyor_parts.push(product);
                ROS_INFO_STREAM(" No objects found on map! Assuming that it will show up on conveyor... ");
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

    // check if there is any last order on the same AGV
    if (!task_queue_.empty()){
        if (agv_id == task_queue_.top().front()[0].agv_id) {
            ROS_INFO("New High Priority Order is on the same AGV... Keep the parts somewhere else");
            pickPartsFromAGV(agv_id);
        }
    } 

    task_queue_.push(order_task_queue);
    ROS_INFO_STREAM("Completed order processing. " << count << " parts on task queue.");

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

void RWAImplementation::pickPartsFromAGV(std::string agv_id) {
    int camera = (agv_id == "agv1" ? 0 : 1);
    int camera_idx = agv_to_camera[agv_id];
    // old_parts_in_tray = parts_in_tray[camera_idx];
    parts_in_tray[camera].clear();
    PresetLocation agv = (agv_id == "agv1" ? gantry_->agv1_ : gantry_->agv2_);
    auto agv_parts = cam_listener_->fetchPartsFromCamera(*node_, camera_idx);
    for(auto agv_part : agv_parts) {
        part temp_part;
        temp_part.type = agv_part.type + "_part_" + agv_part.color;
        temp_part.pose = agv_part.world_pose;  
        gantry_->goToPresetLocation(start_a);
        gantry_->goToPresetLocation(agv);
        ros::Duration(1.0).sleep();
        gantry_->pickPart(temp_part, "left_arm");
        gantry_->deactivateGripper("left_arm");
    }
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
    ROS_INFO_STREAM("===========================================");
    ROS_INFO_STREAM("Checking conveyor...");

    ROS_INFO_STREAM("Waiting for part: " << waiting_for_part_);
    ROS_INFO_STREAM("Load time on conveyor size: " << cam_listener_->load_time_on_conveyor_.size());
    ROS_INFO_STREAM("Parts on conveyor size: " << cam_listener_->parts_on_conveyor_.size());
    ROS_INFO_STREAM("buffer parts collected: " << buffer_parts_collected);


    if (!waiting_for_part_ && !cam_listener_->load_time_on_conveyor_.empty() && !cam_listener_->parts_on_conveyor_.empty())
    {
        waiting_for_part_ = true;
        current_part_load_time_ = cam_listener_->load_time_on_conveyor_.front();
        current_part_on_conveyor_ = cam_listener_->parts_on_conveyor_.front();
        cam_listener_->load_time_on_conveyor_.pop();
        cam_listener_->parts_on_conveyor_.pop_front();
    }
    if ((buffer_parts_collected < buffer_parts_) && waiting_for_part_)
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
        gantry_->goToPresetLocation(start_a);

        buffer_parts_collected++;
        continue_ = true;
    }
    ROS_INFO_STREAM("===========================================");
    return continue_;
}

// Note: this function is called once in the constructor of RWAImplementation
void RWAImplementation::InitRegionDictionaryDependingOnSituation() {

    if (!region_dict_defined_) 
    {
        ROS_INFO_STREAM(" Human Obstacles not initialized, Error, need to add some code here ************************************************");
        // auto clear = lane_handler.clearLanes();                 // [Northernmost Lane Row .......... Southermost Lane Row]
        // if(!clear[4]) return;                                   // "clear[4] == true" means that exactly two lanes are known to have obstacles
        // std::array<bool, 5> clear = {true, true ,false ,true, false};
        std::array<bool, 5> clear = {true, true ,true ,true, true};

        region_dict_defined_ = true;
        if (clear[0] == true) { regionDictionary["shelf5upper"] = {"shelf5_fromNorth_near", "nowait", "fromNorth", "near"}; }
        else if (clear[1] == true) { regionDictionary["shelf5upper"] = {"shelf5_fromSouth_far", "nowait", "fromSouth", "far"}; }
        else if (clear[0] == false && clear[1] == false) { regionDictionary["shelf5upper"] = {"shelf5_fromNorth_near", "wait", "fromNorth", "near"}; }


        if (clear[1] == true) { regionDictionary["shelf5lower"] = {"shelf5_fromSouth_near", "nowait", "fromSouth", "near"}; }
        else if (clear[0] == true) { regionDictionary["shelf5lower"] = {"shelf5_fromNorth_far", "nowait", "fromNorth", "far"}; }
        else if (clear[0] == false && clear[1] == false) { regionDictionary["shelf5lower"] = {"shelf5_fromNorth_far", "wait", "fromNorth", "far"}; }
        

        if (clear[1] == true) { regionDictionary["shelf8upper"] = {"shelf8_fromNorth_near", "nowait", "fromNorth", "near"}; }
        else if (clear[2] == true) { regionDictionary["shelf8upper"] = {"shelf8_fromSouth_far", "nowait", "fromSouth", "far"}; }
        else if (clear[1] == false && clear[2] == false) { // Both innermost lanes not clear case, must account for Shelf Gaps

            ROS_INFO_STREAM(" Shelf Gaps not implemented, Error, need to add some code here ************************************************");

            // if ( true ) { // this code block: for testing only
            //     regionDictionary["shelf8upper"] = {"shelf8_fromSouth_far", "wait", "fromSouth", "far"};
            // }
            // else if ( false ) {
            //     regionDictionary["shelf8upper"] = {"shelf8_fromNorth_near", "wait", "fromNorth", "near"};
            // }

            if ( gaps[2] == "gap_end") { //  Gap is between shelf 11 and shelf 10
                ROS_INFO_STREAM(" gap_end [2] case shelf8upper");
                regionDictionary["shelf8upper"] = {"shelf8_fromSouth_far", "wait", "fromSouth", "far"};
            }
            else if ( gaps[0] == "gap_end" ) { // Gap is between shelf 5 and shelf 4
                ROS_INFO_STREAM(" gap_end [0] case shelf8upper");
                regionDictionary["shelf8upper"] = {"shelf8_fromNorth_near", "wait", "fromNorth", "near"};
            }
            else {
                ROS_INFO_STREAM(" Error gaps not found");
            }
        }


        if (clear[2] == true) { regionDictionary["shelf8lower"] = {"shelf8_fromSouth_near", "nowait", "fromSouth", "near"}; }
        else if (clear[1] == true) { regionDictionary["shelf8lower"] = {"shelf8_fromNorth_far", "nowait", "fromNorth", "far"}; }
        else if (clear[1] == false && clear[2] == false) { // Both innermost lanes not clear case, must account for Shelf Gaps

            ROS_INFO_STREAM(" Shelf Gaps not implemented, Error, need to add some code here ************************************************");

            // if ( true ) { // this code block: for testing only
            //     regionDictionary["shelf8lower"] = {"shelf8_fromSouth_near", "wait", "fromSouth", "near"};
            // }
            // else if ( false ) {
            //     regionDictionary["shelf8lower"] = {"shelf8_fromNorth_far", "wait", "fromNorth", "far"};
            // }

            if ( gaps[2] == "gap_end") { //  Gap is between shelf 11 and shelf 10
                ROS_INFO_STREAM(" gap_end [2] case shelf8lower");
                regionDictionary["shelf8lower"] = {"shelf8_fromSouth_near", "wait", "fromSouth", "near"};
            }
            else if ( gaps[0] == "gap_end" ) { // Gap is between shelf 5 and shelf 4
                ROS_INFO_STREAM(" gap_end [0] case shelf8lower");
                regionDictionary["shelf8lower"] = {"shelf8_fromNorth_far", "wait", "fromNorth", "far"};
            }
            else {
                ROS_INFO_STREAM(" Error gaps not found");
            }
        }


        if (clear[2] == true) { regionDictionary["shelf11upper"] = {"shelf11_fromNorth_near", "nowait", "fromNorth", "near"}; }
        else if (clear[3] == true) { regionDictionary["shelf11upper"] = {"shelf11_fromSouth_far", "nowait", "fromSouth", "far"}; }
        else if (clear[2] == false && clear[3] == false) { regionDictionary["shelf11upper"] = {"shelf11_fromSouth_near", "wait", "fromSouth", "near"}; }


        if (clear[3] == true) { regionDictionary["shelf11lower"] = {"shelf11_fromSouth_near", "nowait", "fromSouth", "near"}; }
        else if (clear[2] == true) { regionDictionary["shelf11lower"] = {"shelf11_fromNorth_far", "nowait", "fromNorth", "far"}; }
        else if (clear[2] == false && clear[3] == false) { regionDictionary["shelf11lower"] = {"shelf11_fromNorth_near", "wait", "fromNorth", "near"}; }


        ////////////////////// Shelves 1 and 2 //////////////////////////////////
        regionDictionary["shelf1upper"] = {"shelf1_fromSouth_far", "nowait", "fromSouth", "far"};
        regionDictionary["shelf1lower"] = {"shelf1_fromSouth_near", "nowait", "fromSouth", "near"};
        regionDictionary["shelf2upper"] = {"shelf2_fromNorth_near", "nowait", "fromNorth", "near"};
        regionDictionary["shelf2lower"] = {"shelf2_fromNorth_far", "nowait", "fromNorth", "far"};
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RWAImplementation::buildKit()
{
    if (task_queue_.top().empty())
    {
        ROS_INFO("Task queue is empty. Return");
        return false;
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

    std::string wait_or_nowait_string = "nowait"; // ie. "nowait", assume nowait unless changed further below
    
    ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

    bool moved_to_bin = false;
    ROS_INFO_STREAM(" moved to bin === " << moved_to_bin);
    std::string near_or_far_string = "near"; // assume near unless shown otherwise

    bool moved_to_shelf_1_or_2 = false;

    // if (false) // if part was moved from conveyor to the bin CRITICAL FOR TESTING ONLY, COMMENT THIS LINE
    if (product.get_from_conveyor == true)
    { // if part was moved from conveyor to the bin
        ROS_INFO_STREAM("Deal with conveyor part _______________________");
        cam_listener_->fetchParts(*node_);
        cam_listener_->sort_camera_parts_list();
        auto color_type = cam_listener_->getColorType(product.type);
        auto cam_parts_repoll = cam_listener_->ordered_color_type[color_type[0]][color_type[1]];

        if(cam_parts_repoll.empty()) return true;
        CameraListener::ModelInfo model_info_conveyor_part = cam_parts_repoll[0]; // todo deal with not enough saved up conveyor parts

        product.designated_model = model_info_conveyor_part;
        my_part.pose = model_info_conveyor_part.world_pose; // seems very redundant to store in the modelInfo and part
        part_in_tray.initial_pose = model_info_conveyor_part.world_pose; // save the initial pose
        part_in_tray.target_pose = gantry_->getTargetWorldPose(part_in_tray.pose, product.agv_id);

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
    else if (discovered_cam_idx == 10 || discovered_cam_idx == 13 || discovered_cam_idx == 14 || discovered_cam_idx == 15 ) // shelves 1 and 2
    {
        //////////////////////////////////////////////////////////// Begin Shelf 1 or 2 Case ////////////////////////////////////////////////////////////

        moved_to_shelf_1_or_2 = true;
        ROS_INFO_STREAM(" moved to shelf 1 or 2 block entered, should be true now -> === " << moved_to_shelf_1_or_2);

        ///////////////////////////////////////////////////////////////////////////////////////
        std:: string shelf_and_upper_or_lower_string = isPartInUpperOrLowerRegionOfWhichShelf(my_part, discovered_cam_idx); // ie. "shelf2upper"
        ROS_INFO_STREAM("shelf_and_upper_or_lower_string: " << shelf_and_upper_or_lower_string);
        ROS_INFO_STREAM("executed isPartInUpperOrLowerRegionOfWhichShelf successfully ");

        std::vector<std::string> information_string_vector = regionDictionary[shelf_and_upper_or_lower_string]; // ie. ["shelf5_fromNorth_near", "nowait", "fromNorth", "near"]
        std::string preset_location_string = information_string_vector[0]; // ie. "shelf1_fromSouth_near"
        wait_or_nowait_string = information_string_vector[1]; // ie. "nowait"
        std::string fromNorth_or_fromSouth_string = information_string_vector[2]; // ie. "fromSouth"
        near_or_far_string = information_string_vector[3]; // ie. "near"


        // Create Bump() offsets, 4 possible scenarios
        ROS_INFO_STREAM("Calculating offsets...");
        double add_to_x_shelf = my_part.pose.position.x - -13.522081; // constant is perfect bin red pulley x
        double add_to_y_shelf = my_part.pose.position.y - 3.446263; // constant is perfect bin red pulley y
        double add_to_torso = 0.0;

        if (near_or_far_string == "near" && fromNorth_or_fromSouth_string == "fromSouth") {
            ROS_INFO_STREAM("Case 1 fromSouth near encountered for offsets...");

            add_to_x_shelf = my_part.pose.position.x - -13.522081      + 1.795838; // red pulley + offset to account for spin
            add_to_y_shelf = my_part.pose.position.y - 3.446263        - 1.707474; // constant is perfect bin red pulley y
            add_to_torso = -PI; // spin torso
            // add_to_torso = 0.0; // spin torso
        }
        else if (near_or_far_string == "near" && fromNorth_or_fromSouth_string == "fromNorth") {
            ROS_INFO_STREAM("Case 2 fromNorth near encountered for offsets...");

            add_to_x_shelf = my_part.pose.position.x - -13.522081; // red pulley
            add_to_y_shelf = my_part.pose.position.y - 3.446263; // red pulley
            add_to_torso = 0.0;
        }
        else if (near_or_far_string == "far" && fromNorth_or_fromSouth_string == "fromNorth") {
            ROS_INFO_STREAM("Case 3 fromNorth far encountered for offsets...");
            add_to_x_shelf = my_part.pose.position.x - -13.522081; // red pulley
            add_to_y_shelf = my_part.pose.position.y - -3.523814; // Blue Pulley
            add_to_torso = 0.0;
        }
        else if (near_or_far_string == "far" && fromNorth_or_fromSouth_string == "fromSouth") {
            ROS_INFO_STREAM("Case 4 fromSouth far encountered for offsets...");
            add_to_x_shelf = my_part.pose.position.x - -13.522081       + 0.34115; // blue pulley + account for spin
            add_to_y_shelf = my_part.pose.position.y - -3.523814        - 3.127628; // blue pulley + account for spin
            add_to_torso = PI; // spin torso
        }
        else {
            // code should not reach here
            ROS_INFO_STREAM("error, Shelf 1 or 2, and unknown condition found for near_or_far_string and fromNorth_or_fromSouth_string"); 
        }

        ROS_INFO_STREAM("Shelf 1 and 2 condtion encountered");
        // Lookup PresetLocation in noWaitDictionary
        std::vector<PresetLocation> path = getPresetLocationVectorUsingString(preset_location_string, wait_or_nowait_string); // initialize with no wait
        ROS_INFO_STREAM("getPresetLocationVector executed!");

        executeVectorOfPresetLocations(path);
        ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        if (near_or_far_string == "near") { gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, add_to_torso)); }
        else if (near_or_far_string == "far") { gantry_->goToPresetLocation(Bump(shelf11_south_far, add_to_x_shelf, add_to_y_shelf, add_to_torso));}


        //////////////////////////////////////////////////////////// End Shelf 1 or 2 Case ////////////////////////////////////////////////////////////


    }

    else if (discovered_cam_idx == 0 || discovered_cam_idx == 7 || discovered_cam_idx == 1 || discovered_cam_idx == 2) // the bins
    {
        moved_to_bin = true;
        ROS_INFO_STREAM(" moved to bin block entered, should be true now -> === " << moved_to_bin);

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
        ROS_INFO_STREAM("goToPresetLocation with bump executed! " << cam_to_presetlocation[discovered_cam_idx].name  << " .");
    }
    
    else if (discovered_cam_idx == 6 || discovered_cam_idx == 8 || discovered_cam_idx == 9 || discovered_cam_idx == 11 || discovered_cam_idx == 12 || discovered_cam_idx == 16) // 6,9,8,12,16,11
    {                                                                 // any other camera
        ROS_INFO_STREAM("discovered_cam_idx: " << discovered_cam_idx);
        ROS_INFO_STREAM("Any Other Camera Case Reached");

        if (!region_dict_defined_) {
            ROS_INFO_STREAM("Wait till lanes are known.");
            return true;
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        std::string shelf_and_upper_or_lower_string = isPartInUpperOrLowerRegionOfWhichShelf(my_part, discovered_cam_idx); // ie. "shelf5upper"
        ROS_INFO_STREAM("shelf_and_upper_or_lower_string: " << shelf_and_upper_or_lower_string);
        ROS_INFO_STREAM("executed isPartInUpperOrLowerRegionOfWhichShelf successfully ");
        std::vector<std::string> information_string_vector = regionDictionary[shelf_and_upper_or_lower_string]; // ie. ["shelf5_fromNorth_near", "nowait", "fromNorth", "near"]
        ROS_INFO_STREAM("423");
        std::string preset_location_string = information_string_vector[0]; // ie. "shelf5_fromNorth_near"
        ROS_INFO_STREAM("425");
        wait_or_nowait_string = information_string_vector[1]; // ie. "nowait"
        ROS_INFO_STREAM("427");
        std::string fromNorth_or_fromSouth_string = information_string_vector[2]; // ie. "fromNorth"
        ROS_INFO_STREAM("429");
        near_or_far_string = information_string_vector[3]; // ie. "near"

        // Create Bump() offsets, 4 possible scenarios
        ROS_INFO_STREAM("Calculating offsets...");
        double add_to_x_shelf = my_part.pose.position.x - -13.522081; // constant is perfect bin red pulley x
        double add_to_y_shelf = my_part.pose.position.y - 3.446263; // constant is perfect bin red pulley y
        double add_to_torso = 0.0;

        if (near_or_far_string == "near" && fromNorth_or_fromSouth_string == "fromSouth") {
            ROS_INFO_STREAM("Case 1 fromSouth near encountered for offsets...");

            add_to_x_shelf = my_part.pose.position.x - -13.522081      + 1.795838; // red pulley + offset to account for spin
            add_to_y_shelf = my_part.pose.position.y - 3.446263        - 1.707474; // constant is perfect bin red pulley y
            add_to_torso = -PI; // spin torso
            // add_to_torso = 0.0; // spin torso
        }
        else if (near_or_far_string == "near" && fromNorth_or_fromSouth_string == "fromNorth") {
            ROS_INFO_STREAM("Case 2 fromNorth near encountered for offsets...");

            add_to_x_shelf = my_part.pose.position.x - -13.522081; // red pulley
            add_to_y_shelf = my_part.pose.position.y - 3.446263; // red pulley
            add_to_torso = 0.0;
        }
        else if (near_or_far_string == "far" && fromNorth_or_fromSouth_string == "fromNorth") {
            ROS_INFO_STREAM("Case 3 fromNorth far encountered for offsets...");
            add_to_x_shelf = my_part.pose.position.x - -13.522081; // red pulley
            add_to_y_shelf = my_part.pose.position.y - -3.523814; // Blue Pulley
            add_to_torso = 0.0;
        }
        else if (near_or_far_string == "far" && fromNorth_or_fromSouth_string == "fromSouth") {
            ROS_INFO_STREAM("Case 4 fromSouth far encountered for offsets...");
            add_to_x_shelf = my_part.pose.position.x - -13.522081       + 0.34115; // blue pulley + account for spin
            add_to_y_shelf = my_part.pose.position.y - -3.523814        - 3.127628; // blue pulley + account for spin
            add_to_torso = PI; // spin torso
        }
        else {
            // code should not reach here
            ROS_INFO_STREAM("error, unknown condition found for near_or_far_string and fromNorth_or_fromSouth_string"); 
        }



        // std::vector<PresetLocation> path = getPresetLocationVectorUsingString(preset_location_string, wait_or_nowait_string); // initialize with no wait
        std::vector<PresetLocation> path; // initialize empty for now
        if (wait_or_nowait_string == "wait") {
            ROS_INFO_STREAM("Wait condtion encountered, need to use wait dictionary situation here ******************");
            // Lookup PresetLocation in waitDictionary
            // std::vector<PresetLocation> path = getPresetLocationVector(cam_to_presetlocation[discovered_cam_idx]); // todo lookup in wait dictionary here!
            path = getPresetLocationVectorUsingString(preset_location_string, wait_or_nowait_string); // call with wait
            ROS_INFO_STREAM("getPresetLocationVector executed!");

            executeVectorOfPresetLocationsWithWait(path); // Note the With Wait version is being called here
            ROS_INFO_STREAM("executeVectorOfPresetLocationsWithWait executed!");

            if (near_or_far_string == "near") { gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, add_to_torso)); }
            else if (near_or_far_string == "far") { gantry_->goToPresetLocation(Bump(shelf11_south_far, add_to_x_shelf, add_to_y_shelf, add_to_torso));}

            ROS_INFO_STREAM("goToPresetLocation with bump executed!");

            // ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away // commented for speed Human Obstacles
        }
        else { // else the "nowait" case
            ROS_INFO_STREAM("No Wait condtion encountered");
            // Lookup PresetLocation in noWaitDictionary
            std::vector<PresetLocation> path = getPresetLocationVectorUsingString(preset_location_string, wait_or_nowait_string); // initialize with no wait
            ROS_INFO_STREAM("getPresetLocationVector executed!");

            executeVectorOfPresetLocations(path);
            ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

            if (near_or_far_string == "near") { gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, add_to_torso)); }
            else if (near_or_far_string == "far") { gantry_->goToPresetLocation(Bump(shelf11_south_far, add_to_x_shelf, add_to_y_shelf, add_to_torso));}

            // gantry_->goToPresetLocation(Bump(shelf11_a, add_to_x_shelf, add_to_y_shelf, add_to_torso));
            // ROS_INFO_STREAM("goToPresetLocation with bump executed!");

            // ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away // commented for speed Human Obstacles

        }
        ///////////////////////////////////////////////////////////////////////////////////////

        // ROS_INFO_STREAM("Any Other Camera Block reached _______________________");
        // double add_to_x_shelf = my_part.pose.position.x - -13.522081; // constant is perfect bin red pulley x
        // double add_to_y_shelf = my_part.pose.position.y - 3.446263;
        // ROS_INFO_STREAM(" x " << add_to_x_shelf << " y " << add_to_y_shelf);

        // std::vector<PresetLocation> path = getPresetLocationVector(cam_to_presetlocation[discovered_cam_idx]);
        // ROS_INFO_STREAM("getPresetLocationVector executed!");

        // executeVectorOfPresetLocations(path);
        // ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

        // gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, 0));
        // ROS_INFO_STREAM("goToPresetLocation with bump executed!");

        // ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
    }
    else
    {
        ROS_INFO_STREAM("Cam Idx: " << discovered_cam_idx);
        ROS_INFO_STREAM("error, the camera idx is not equal to an expected number!!");
    }


    if (wait_or_nowait_string == "wait" && near_or_far_string == "far") { // wait and far pick
        //--Go pick the part
        if (!gantry_->pickPartFast(my_part, "left_arm") && wait_or_nowait_string == "nowait")
        {
            //pass
        }
    }
    else if (wait_or_nowait_string == "wait" && near_or_far_string == "near" ) { // wait and near pick (just lift up normally, avoid scraping on shelf lip)
        //--Go pick the part
        if (!gantry_->pickPartFast_Near(my_part, "left_arm") && wait_or_nowait_string == "nowait")
        {
            //pass
        }
    }
    else { // original, no wait code
        //--Go pick the part
        if (!gantry_->pickPart(my_part, "left_arm") && wait_or_nowait_string == "nowait") // no wait, regular pick
        {
            // gantry.goToPresetLocation(gantry.start_);
            // spinner.stop();
            // ros::shutdown();
            //pass
        }
    }

    // go back to start
    // gantry_->goToPresetLocation(start_a);
    if (wait_or_nowait_string == "wait")
    { // if wait, any camera
        // std::vector<PresetLocation> path = getPresetLocationVectorWithWait(start_a); 
        std::vector<PresetLocation> path = getPresetLocationVectorUsingString_Specific(start_a.name, wait_or_nowait_string, inner_fast_preset_locations_list_); // call with wait

        // executeVectorOfPresetLocations(path);
        executeVectorOfPresetLocations_Fast(path); // no delays in-between moves
        // ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves // Commented for speed Human Obstacles
    }
    else if (moved_to_bin == true) { // deal with minor shelf bumping RWA5
        std::vector<PresetLocation> path = getPresetLocationVector_Simple(start_a); // use simple gantry x and y euclidean distance only, not every joint
        executeVectorOfPresetLocations(path);
        ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves // Commented for speed Human Obstacles
    }
    else if (moved_to_shelf_1_or_2 == true) { // Shelves 1 and 2 (this code is a bit redundant, but just to be safe)
        // std::vector<PresetLocation> path = getPresetLocationVector(start_a);
        std::vector<PresetLocation> path = getPresetLocationVectorUsingString_Specific(start_a.name, "nowait", shelf_1_or_2_preset_locations_list_);
        executeVectorOfPresetLocations(path);
        ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves // Commented for speed Human Obstacles
    }
    else { // if nowait, if any camera
        std::vector<PresetLocation> path = getPresetLocationVector(start_a);
        executeVectorOfPresetLocations(path);
        ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves // Commented for speed Human Obstacles
    }

    // // testing drop parts into bins, uncomment this whole block, comment out entire next block
    // PresetLocation drop_location = getNearestBinPresetLocation();
    // gantry_->goToPresetLocation( Bump( drop_location, 0.0, -0.8, 0) );
    // simpleDropPart();

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
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double RWAImplementation::calcDistanceInXYPlane(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    return std::sqrt(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2)); // euclidean distance
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double RWAImplementation::calcDistanceInXYTorso(PresetLocation pLocation, std::vector<double> joint_positions)
{
    // ROS_INFO_STREAM("calcDistanceInXYTorso entered");
    return std::sqrt(std::pow(pLocation.gantry[0] - joint_positions[0], 2) + std::pow(pLocation.gantry[1] - joint_positions[1], 2) + std::pow(pLocation.gantry[2] - joint_positions[2], 2) ); // euclidean distance
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double RWAImplementation::calcDistanceInXYTorso_Accurate(PresetLocation pLocation, std::vector<double> joint_positions)
{
    // ROS_INFO_STREAM("calcDistanceInXYTorso_Accurate entered");
    // return std::sqrt(std::pow(pLocation.gantry[0] - joint_positions[0], 2) + std::pow(pLocation.gantry[1] - joint_positions[1], 2) + std::pow(pLocation.gantry[2] - joint_positions[2], 2 )
    // + std::pow(pLocation.left_arm[0] - joint_positions[3], 2 )
    // + std::pow(pLocation.left_arm[1] - joint_positions[4], 2 )
    // + std::pow(pLocation.left_arm[2] - joint_positions[5], 2 )
    // + std::pow(pLocation.left_arm[3] - joint_positions[6], 2 )
    // + std::pow(pLocation.left_arm[4] - joint_positions[7], 2 )
    // + std::pow(pLocation.left_arm[5] - joint_positions[8], 2 )
    // + std::pow(pLocation.right_arm[0] - joint_positions[9], 2 )
    // + std::pow(pLocation.right_arm[1] - joint_positions[10], 2 )
    // + std::pow(pLocation.right_arm[2] - joint_positions[11], 2 )
    // + std::pow(pLocation.right_arm[3] - joint_positions[12], 2 )
    // + std::pow(pLocation.right_arm[4] - joint_positions[13], 2 )
    // + std::pow(pLocation.right_arm[5] - joint_positions[14], 2 )
    //  ); // euclidean distance
    double euclidean_score = std::sqrt(std::pow(pLocation.gantry[0] - joint_positions[0], 2) + std::pow(pLocation.gantry[1] - joint_positions[1], 2) + std::pow(pLocation.gantry[2] - joint_positions[2], 2 )
    + std::pow(pLocation.left_arm[0] - joint_positions[3], 2 )
    + std::pow(pLocation.left_arm[1] - joint_positions[4], 2 )
    + std::pow(pLocation.left_arm[2] - joint_positions[5], 2 )
    + std::pow(pLocation.left_arm[3] - joint_positions[6], 2 )
    + std::pow(pLocation.left_arm[4] - joint_positions[7], 2 )
    + std::pow(pLocation.left_arm[5] - joint_positions[8], 2 )
    + std::pow(pLocation.right_arm[0] - joint_positions[9], 2 )
    + std::pow(pLocation.right_arm[1] - joint_positions[10], 2 )
    + std::pow(pLocation.right_arm[2] - joint_positions[11], 2 )
    + std::pow(pLocation.right_arm[3] - joint_positions[12], 2 )
    + std::pow(pLocation.right_arm[4] - joint_positions[13], 2 )
    + std::pow(pLocation.right_arm[5] - joint_positions[14], 2 )
    ); // euclidean distance

    // ROS_INFO_STREAM("euclidean_score is: " << euclidean_score);
    return euclidean_score;
}

//////////////////////////////// Get nearest preset location to current gantry position
PresetLocation RWAImplementation::getNearesetPresetLocation()
{
    ////// Get current gantry position
    // geometry_msgs::Pose current_position = gantry_->getGantryPose();
    std::vector<double> joint_positions = gantry_->getGantryJointPositionsDoubleVector(); // instead of converting to Pose, use joint values directly

    ROS_INFO_STREAM("Length of joint_positions double vector is: " << joint_positions.size());

    ////// Define custom struct (distance, PresetLocation), see https://www.cplusplus.com/articles/NhA0RXSz/

    ////// Build vector of custom structs
    std::vector<distance_and_PresetLocation_struct> vector_of_structs;

    for (PresetLocation pset_location : preset_locations_list_)
    {
        // geometry_msgs::Pose location_converted_to_pose = gantryXY2worldposeXY(pset_location);    // convert preset location to pose
        // double distance_i = calcDistanceInXYPlane(current_position, location_converted_to_pose); // euclidean dist in xy plane
        double distance_i = calcDistanceInXYTorso_Accurate(pset_location, joint_positions);              // "score", aka distance, in gantry's joint 0, 1, 2 space
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


//////////////////////////////// Get nearest preset location to current gantry position
PresetLocation RWAImplementation::getNearesetPresetLocation_Specific(std::vector<PresetLocation> preset_vector_to_lookup_in)
{
    ////// Get current gantry position
    // geometry_msgs::Pose current_position = gantry_->getGantryPose();
    std::vector<double> joint_positions = gantry_->getGantryJointPositionsDoubleVector(); // instead of converting to Pose, use joint values directly

    ROS_INFO_STREAM("Length of joint_positions double vector is: " << joint_positions.size());

    ////// Define custom struct (distance, PresetLocation), see https://www.cplusplus.com/articles/NhA0RXSz/

    ////// Build vector of custom structs
    std::vector<distance_and_PresetLocation_struct> vector_of_structs;

    for (PresetLocation pset_location : preset_vector_to_lookup_in)
    {
        // geometry_msgs::Pose location_converted_to_pose = gantryXY2worldposeXY(pset_location);    // convert preset location to pose
        // double distance_i = calcDistanceInXYPlane(current_position, location_converted_to_pose); // euclidean dist in xy plane
        double distance_i = calcDistanceInXYTorso_Accurate(pset_location, joint_positions);              // "score", aka distance, in gantry's joint 0, 1, 2 space
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

//////////////////////////////// Get nearest preset location to current gantry position
PresetLocation RWAImplementation::getNearesetPresetLocation_Simple()
{
    ROS_INFO_STREAM("Entered getNearesetPresetLocation_Simple++++++++++++++++++++++++++++++++++++++++++++++++++");

    ////// Get current gantry position
    geometry_msgs::Pose current_position = gantry_->getGantryPose();
    // std::vector<double> joint_positions = gantry_->getGantryJointPositionsDoubleVector(); // instead of converting to Pose, use joint values directly

    ////// Define custom struct (distance, PresetLocation), see https://www.cplusplus.com/articles/NhA0RXSz/

    ////// Build vector of custom structs
    std::vector<distance_and_PresetLocation_struct> vector_of_structs;

    for (PresetLocation pset_location : preset_locations_list_)
    {
        geometry_msgs::Pose location_converted_to_pose = gantryXY2worldposeXY(pset_location);    // convert preset location to pose
        double distance_i = calcDistanceInXYPlane(current_position, location_converted_to_pose); // euclidean dist in xy plane
        // double distance_i = calcDistanceInXYTorso(pset_location, joint_positions);              // "score", aka distance, in gantry's joint 0, 1, 2 space
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

std::vector<PresetLocation> RWAImplementation::getPresetLocationVector_Simple(PresetLocation target_preset_location)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation_Simple();
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location.name}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);
    std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key);
    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

std::vector<PresetLocation> RWAImplementation::getPresetLocationVectorWithWait(PresetLocation target_preset_location)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation();
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location.name}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);
    std::vector<PresetLocation> path_to_execute = WaitPathingLookupDictionary.at(key);
    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

std::vector<PresetLocation> RWAImplementation::getPresetLocationVectorUsingString(std::string target_preset_location_string, std::string wait_string)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation();
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location_string}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);

    // std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key); // this line error, cannot find wait preset locations
    std::vector<PresetLocation> path_to_execute; // initialize empty, wait_string MUST be "wait" or "nowait"
    if (wait_string == "wait") {
        path_to_execute = WaitPathingLookupDictionary.at(key);
    }
    else {
        path_to_execute = PathingLookupDictionary.at(key);
    }

    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

std::vector<PresetLocation> RWAImplementation::getPresetLocationVectorUsingString_Specific(std::string target_preset_location_string, std::string wait_string, std::vector<PresetLocation> pset_location_vector_to_use)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation_Specific(pset_location_vector_to_use);
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location_string}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);

    // std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key); // this line error, cannot find wait preset locations
    std::vector<PresetLocation> path_to_execute; // initialize empty, wait_string MUST be "wait" or "nowait"
    if (wait_string == "wait") {
        path_to_execute = WaitPathingLookupDictionary.at(key);
    }
    else {
        path_to_execute = PathingLookupDictionary.at(key);
    }

    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

std::vector<PresetLocation> RWAImplementation::getPresetLocationVectorUsingStringNoWait(std::string target_preset_location_string, std::string wait_string)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation_Simple();
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location_string}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);

    // std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key); // this line error, cannot find wait preset locations
    std::vector<PresetLocation> path_to_execute; // initialize empty, wait_string MUST be "wait" or "nowait"
    if (wait_string == "wait") {
        path_to_execute = WaitPathingLookupDictionary.at(key);
    }
    else {
        path_to_execute = PathingLookupDictionary.at(key);
    }

    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

bool RWAImplementation::executeVectorOfPresetLocations(std::vector<PresetLocation> path_to_execute)
{
    for (PresetLocation psetlocation : path_to_execute)
    {
        ROS_INFO_STREAM("Moving to PresetLocation: >>>>> " << psetlocation.name);
        gantry_->goToPresetLocation(psetlocation);
        ros::Duration(0.25).sleep();
        // ros::Duration(0.05).sleep();
    }
    return true;
}

bool RWAImplementation::executeVectorOfPresetLocations_Fast(std::vector<PresetLocation> path_to_execute)
{
    for (PresetLocation psetlocation : path_to_execute)
    {
        ROS_INFO_STREAM("Moving to PresetLocation: >>>>> " << psetlocation.name);
        gantry_->goToPresetLocation(psetlocation);
        // ros::Duration(0.25).sleep(); // this version (Fast) has no delays
        // ros::Duration(0.05).sleep();
    }
    return true;
}

bool RWAImplementation::executeVectorOfPresetLocationsWithWait(std::vector<PresetLocation> path_to_execute)
{
    for (PresetLocation psetlocation : path_to_execute)
    {
        ROS_INFO_STREAM("Moving to PresetLocation: >>>>> " << psetlocation.name);
        gantry_->goToPresetLocation(psetlocation);

        if(std::find(wait_preset_locations_list_.begin(), wait_preset_locations_list_.end(), psetlocation.name) != wait_preset_locations_list_.end()) {
            /* v contains x */
            auto gant_pos = gantry_->getGantryPose();
            int lane{};
            if (gant_pos.position.y > 0) lane = 1;
            else lane = 2;
            lane_handler.waitForOpening(lane);
        } else {
            /* v does not contain x */
            ;// do nothing, no need to wait, pass
        }
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
void RWAImplementation::rankEmptyBins() {
    // [xlow, xhigh, ylow, yhigh]
    std::vector<std::vector<double>> TwoPointsList = {
        {2.3, 2.957942, 1.842058, 2.5}, // bin 5
        {2.3, 2.957942, 1.0, 1.657942}, // bin 1
        {2.3, 2.957942, -1.657942, -1.0}, // bin 9
        {2.3, 2.957942, -2.5, -1.842058}, // bin 13

        {3.14, 3.797, 1.842058, 2.5}, // bin 6
        {3.14, 3.797, 1.0, 1.657942}, // bin 2
        {3.14, 3.797, -1.657942, -1.0}, // bin 10
        {3.14, 3.797, -2.5, -1.842058}, // bin 14

        {3.98, 4.638, 1.842058, 2.5}, // bin 7
        {3.98, 4.638, 1.0, 1.657942}, // bin 3
        {3.98, 4.638, -1.657942, -1.0}, // bin 11
        {3.98, 4.638, -2.5, -1.842058}, // bin 15

        {4.82, 5.478, 1.842058, 2.5}, // bin 8
        {4.82, 5.478, 1.0, 1.657942}, // bin 4
        {4.82, 5.478, -1.657942, -1.0}, // bin 12
        {4.82, 5.478, -2.5, -1.842058}, // bin 16
    };
    std::vector<int> bin_order{5,1,9,13,6,2,10,14,7,3,11,15,8,4,12,16};

        // poll cameras
    auto cam_parts = cam_listener_->fetchParts(*node_); //  returns std::array<std::vector<CameraListener::ModelInfo>, 17>

    // create a flat parts list vector
    int count{0};
    std::vector<CameraListener::ModelInfo> flat_parts_list;
    for (auto cam_part_array : cam_parts)
    {
        for (auto model_info : cam_part_array)
        {
            flat_parts_list.push_back(model_info);
            count++;
        }
    }
    ROS_INFO_STREAM(count << " parts, array has " << flat_parts_list.size() << " parts");

    int counter{0};
    for (auto vec : TwoPointsList) {
        double box_xlow = vec[0];
        double box_xhigh = vec[1];
        double box_ylow = vec[2];
        double box_yhigh = vec[3];
        ROS_INFO_STREAM("==== " << bin_order[counter] <<"====");
        ROS_INFO_STREAM("x range: (" << box_xlow << ", " << box_xhigh << ")");
        ROS_INFO_STREAM("y range: (" << box_ylow << ", " << box_yhigh << ")");
        ROS_INFO_STREAM("=========");
        bool empty{true};
        for (auto model_info : flat_parts_list) {
            double part_x = model_info.world_pose.position.x;
            double part_y = model_info.world_pose.position.y;
            
            // ROS_INFO_STREAM("---------");
            // ROS_INFO_STREAM("Part x: " << part_x << ", part y: " << part_y);
            // ROS_INFO_STREAM("---------");

            if( (part_x >= box_xlow) && (part_x <= box_xhigh) && (part_y >= box_ylow) && (part_y <= box_yhigh)) {
                ROS_INFO_STREAM("Part in bin " << bin_order[counter]);
                empty = false;
                break;
            }
        }
        if (empty) {
            PresetLocation dropPresetLocation;
            dropPresetLocation.gantry = {vec[0]-0.63/2, (vec[2]*-1.0)-0.63-0.63/2.0 , 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
            dropPresetLocation.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
            dropPresetLocation.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
            dropPresetLocation.name = "dropPresetLocation";
            ROS_INFO_STREAM("target [xlow,xhigh,ylow,yhigh]] == " << vec[0] << " " << vec[1] << " " << vec[2] << " " << vec[3]);
            empty_bins_.push(dropPresetLocation);
        }
        counter++;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PresetLocation RWAImplementation::getNearestBinPresetLocation()
{
    if (empty_bins_.empty()) {
        ROS_INFO_STREAM("No known empty bins. Attempting to find...");
        rankEmptyBins();
    }
    ROS_INFO_STREAM("Selecting empty bin... " << empty_bins_.size() << " to choose from.");
    auto dropPresetLocation{empty_bins_.front()};
    ROS_INFO_STREAM("Selected " << dropPresetLocation.name);
    empty_bins_.pop();

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
    auto product = task_queue_.top().front()[0];
    // make replacement as false if true before
    if (task_queue_.top().front()[0].replacement)
        task_queue_.top().front()[0].replacement = false;

    ros::Duration(2.0).sleep();
    bool isQueried = cam_listener_->checkFaulty(*node_, product.agv_id);
    //if the quality control sensor is not queried due to bleckout, save the Product and come later to check
    if (!isQueried)
    {
        ROS_INFO_STREAM("Quality control sensor could not be queried, save the product for future reference");
        checkLater.push_back(task_queue_.top().front()[0]);
    }
    ros::Duration(0.5).sleep(); // make sure it actually goes back to start, instead of running into shelves

    int camera_index = product.agv_id == "agv1" ? 0 : 1;
    bool override = false; // override erase top element from task queue
    if (cam_listener_->faulty_parts)
    {
        ROS_INFO("Detected Faulty Part(s)");
        int n_faulty = cam_listener_->faulty_parts_list.size();
        ROS_INFO_STREAM("Total faulty parts = " << n_faulty);
        std::vector<part> to_replace;
        int i, j, k;

        //check the type of part to be replaced by matching world pose with parts placed in tray
        for (i = 0; i < n_faulty; i++)
        {
            CameraListener::ModelInfo faulty = cam_listener_->faulty_parts_list[i];
            part faulty_part;
            faulty_part.target_pose = faulty.world_pose;
            for (j = 0; j < parts_in_tray[camera_index].size(); j++)
            {
                double x = std::abs(std::abs(parts_in_tray[camera_index][j].target_pose.position.x) - std::abs(faulty_part.target_pose.position.x));
                double y = std::abs(std::abs(parts_in_tray[camera_index][j].target_pose.position.y) - std::abs(faulty_part.target_pose.position.y));
                ROS_INFO_STREAM("Deltas = " << x <<" and " <<y);
                if (x < 0.03 && y < 0.03)
                {
                    faulty_part = parts_in_tray[camera_index][j];
                    //faulty_part.target_pose = faulty.world_pose;//test-----------------
                    ROS_INFO_STREAM("Faulty part matched - type = " << faulty_part.type << "\n, Pose = " << faulty_part.pose);
                    to_replace.push_back(faulty_part);
                    //remove the part from the placed parts on tray
                    parts_in_tray[camera_index].erase(parts_in_tray[camera_index].begin()+j);
                    j--;
                }
                else
                {
                    // Take care of Faulty gripper scenario
                    std::string isFaultyGripper = "";
                    ros::param::get("faulty_gripper", isFaultyGripper);
                    if (to_replace.empty() && isFaultyGripper == "true")
                    {
                        ros::param::set("faulty_gripper", "false");
                        ROS_INFO("Faulty gripper and Faulty part at the same time, pick the  part");
                        faulty_part = parts_in_tray[camera_index][j];
                        faulty_part.target_pose = faulty.world_pose;
                        ROS_INFO_STREAM("Faulty part matched - type = " << faulty_part.type << "\n, Pose = " << faulty_part.pose);
                        to_replace.push_back(faulty_part);
                        parts_in_tray[camera_index].erase(parts_in_tray[camera_index].begin()+j);
                        j--;
                    }

                }
            }
        }

        bool foundInTaskQueue = false;
        bool success = false;
        //Now go through each part to be replaced and find corresponding entry in task_queue_ to update
        //and then finally discard the faulty part
        for (i = 0; i < to_replace.size(); i++)
        {
            foundInTaskQueue = false;
            gantry_->goToPresetLocation(product.agv_id == "agv1" ? gantry_->agv1_ : gantry_->agv2_);

            for (j = 0; j < task_queue_.top().front().size(); j++)
            {
                if (task_queue_.top().front()[j].type == to_replace[i].type)
                {
                    double x,y;
                    auto temp = task_queue_.top().front()[j];
                    auto check = to_replace[i];
                    x = std::abs(temp.pose.position.x - check.pose.position.x);
                    y = std::abs(temp.pose.position.y - check.pose.position.y);
                    ROS_INFO_STREAM("Target pose tray diff = " << x <<" , " << y << "; if more than 0.1, continue");
                    if (!(x < 0.1 && y < 0.1))
                        continue;
                    foundInTaskQueue = true;

                    // throw away part
                    success = gantry_->replaceFaultyPart(to_replace[i], product.agv_id, "left_arm");

                    auto Product = task_queue_.top().front()[j];
                    if (success)
                    {
                        if (!sorted_map[Product.designated_model.color][Product.designated_model.type].empty())
                        {
                            ROS_INFO("Faulty part detected, proceed to get new one");
                            //pick new model
                            Product.designated_model = sorted_map[Product.designated_model.color][Product.designated_model.type].top();
                            sorted_map[Product.designated_model.color][Product.designated_model.type].pop();

                        }
                        else
                            Product.get_from_conveyor = true;

                        Product.replacement = true;
                        task_queue_.top().front()[j] = Product;
                    }
                    else
                    {
                        ROS_INFO_STREAM("Faulty part could not be picked up from the tray.... just delete from the task queue");
                        override = true;
                        //TODO - you can also delete the product from task queue in this block....
                        //If any bug is caught, directly remove from here.
                    }
                }
            }
            /* if the product is not found in the task queue, that means it is already removed from the
                task queue and not detected as faulty due to sensor blackout. In that case, add the product back */
            if (foundInTaskQueue == false)
            {
                for (k = 0; k < checkLater.size(); k++)
                {
                    if (checkLater[k].type == to_replace[i].type &&
                         checkLater[k].pose.position.x == to_replace[i].pose.position.x &&
                           checkLater[k].pose.position.y == to_replace[i].pose.position.y)
                    {
                        auto Product = checkLater[k];
                        ROS_INFO_STREAM("Sensor blackout prevented " <<to_replace[i].type << " to be checked for faulty and is found faulty.");
                        ROS_INFO("Throw away this part and update the task queue to add replacement part");
                        // throw away part
                        success = gantry_->replaceFaultyPart(to_replace[i], product.agv_id, "left_arm");

                        if (success && !sorted_map[Product.designated_model.color][Product.designated_model.type].empty())
                        {
                            ROS_INFO("Assign new model");
                            Product.designated_model = sorted_map[Product.designated_model.color][Product.designated_model.type].top();
                            sorted_map[Product.designated_model.color][Product.designated_model.type].pop();

                            //mark it as a replacement
                            Product.replacement = true;
                            task_queue_.top().front().push_back(Product);
                        }
                        // if not successful in picking up the part, leave it as is.
                        if (!success)
                            ROS_INFO_STREAM("Faulty part could not be picked up from tray...");
                    }
                }
            }
        }
        checkLater.clear();
    }

    bool poseUpdated = false;
    if (!cam_listener_->faulty_parts || override)
    {   ROS_INFO("Erasing product from task queue");
        /* if no faulty part, check for poses, if correctly placed then pop from task_queue_.top(),
        pop the products from current shipment list  */
        poseUpdated = checkAndCorrectPose(product.agv_id);
        if (task_queue_.top().front()[0].replacement)
            ROS_INFO("Front product in task queue is replacement, do not remove from task queue");
        else
            task_queue_.top().front().erase(task_queue_.top().front().begin());
    }

    if (task_queue_.top().front().empty())
    {   
        ROS_INFO("Task queue is now empty");
        if(!isQueried)
        {
            ROS_INFO("Quality control was not queried due to sensor blackout. But the task queue is empty.");
            ROS_INFO("AGV Can not be sent untill the part is checked for faulty, wait and re-recheck");
            ROS_INFO("Fill in the task queue with the last Product and re-check");
            task_queue_.top().front().push_back(product);
            //check again for agv errors - give sufficient sleep to allow sensors to return back
            ros::Duration(8.0).sleep();
            checkAgvErrors();
        }
        // recheck again, as we just had one recursion and we do not want to send agv before faulty parts have been replaced
        if (task_queue_.top().front().empty())
        {
            ros::Duration(1.0).sleep(); // add delay
            // one shipment completed. Send to AGV
            AGVControl agv_control(*node_);
            std::string kit_id = (product.agv_id == "agv1" ? "kit_tray_1" : "kit_tray_2");
            ROS_INFO_STREAM("Sending AGV for shipment id " << product.shipment_type);
            agv_control.sendAGV(product.shipment_type, kit_id);
            task_queue_.top().pop();

            if(task_queue_.top().empty()) 
                task_queue_.pop();
                prev_num_orders_--;
        }
        else
        {
            ROS_INFO_STREAM("Get the replacement for faulty parts - task queue size = " << task_queue_.top().front().size());
        }
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
    ROS_INFO_STREAM("Size of list from camera = " << list_from_camera.size());

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

    if (list_from_camera.size() == 0) return false;

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
        ros::Duration(15).sleep();
        ROS_INFO_STREAM("Competition ending...");
        return true;
    }
    return false;
}

bool RWAImplementation::checkForFlip(part &part_in_tray, Product product)
{
    bool success = false;
    if (product.type.find("pulley") == 0)
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

        if (std::abs(std::abs(roll) - std::abs(roll_desired)) > 3.0)
        {
            ROS_INFO("Flip Part needed");
            double roll_zero = 0.0;
            final_orientation.setRPY(roll_zero, pitch, yaw);
            final_orientation.normalize();

            if (std::abs(roll) > 3.0)
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

bool RWAImplementation::detectGaps()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::array<geometry_msgs::TransformStamped, 9> shelves;
    int shelf_number = 3;
    ros::Duration(0.5).sleep();
    ros::Duration timeout(0.2);
    std::string shelf_frame= "";
    ROS_INFO("Lookup for shelves");

    for (int i = 0; i < 9; i++, shelf_number++)
    {
        try
        {
            shelf_frame = "shelf" + std::to_string(shelf_number) + "_frame";
            shelves[i] = tfBuffer.lookupTransform(
                "world",
                shelf_frame,
                ros::Time(0),
                timeout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO_STREAM("Shelf  " << std::to_string(shelf_number) << "  position = \n"  << shelves[i].transform.translation);
    }

    float s1, s2, s3, gap, safe_y, safe_x, max_y, d21, d32, shelf_length = 4.12;
    PresetLocation toAgv, toFarReachSafeLocation, toSafeLocation;

    for (int i = 0; i < 3; i++)
    {
        s1 = shelves[(3*i)].transform.translation.x;
        s2 = shelves[(3*i)+1].transform.translation.x;
        s3 = shelves[(3*i)+2].transform.translation.x;
        ROS_INFO_STREAM("Shelf's x coordinate --> "<<s1<<" ,"<<s2<<", "<<s3);
        d21 = std::abs(s2) - std::abs(s1);
        d32 = std::abs(s3) - std::abs(s2);
        if (d21 > d32)
        {
            ROS_INFO_STREAM("Gap is between 1st and 2nd shelf in shelf row " << i+1 << " from AGV1");
            gap = d21/2;
            safe_x = -(shelf_length + gap);
            gaps[i] = "gap_conveyor";
        }
        else
        {
            ROS_INFO_STREAM("Gap is between 2nd and 3rd shelf in shelf row " << i+1 << " from AGV1");
            gap = d32/2;
            safe_x = -(2*shelf_length + gap);
            gaps[i] = "gap_end";
        }
        if (i == 0)
        {
            toAgv = gantry_->agv1_;
            safe_y = -3.08; //gantry frame
            max_y = -6.9;
        }
        else if (i == 2)
        {
            toAgv = gantry_->agv2_;
            safe_y = 3.08; //gantry frame
            max_y = 6.9;
        }
        else
        {
            toAgv = gantry_->start_;
            safe_y = 0;
            max_y = 0;
        }
        //keep left arm and right arm joint positions same as agv, just change gantry joint position
        //To be updated if left and right arm joint have to be different than that of agv's
        toFarReachSafeLocation = toAgv;
        toFarReachSafeLocation.gantry = {safe_x, max_y, PI};
        toSafeLocation = toAgv;
        toSafeLocation.gantry = {safe_x, safe_y, PI};

        /*for testing only
        gantry_->goToPresetLocation(toAgv);
        gantry_->goToPresetLocation(toFarReachSafeLocation);
        gantry_->goToPresetLocation(toSafeLocation); */

        /* array of vectors - each vector will have preset for going to:
        1) agv from start
        2) waypoint b/w agv and safe location - waiting area to check for obstacles
        3) safe location from waypoint in 2) - gap b/w shelves
        As per requirement, individual preset locations can be extracted from the vector
        */
        shelf_preset_locations[i].push_back(toAgv);
        shelf_preset_locations[i].push_back(toFarReachSafeLocation);
        shelf_preset_locations[i].push_back(toSafeLocation);
    }
    return true;
}