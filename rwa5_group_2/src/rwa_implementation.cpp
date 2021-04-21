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


    // joint positions to go to bin11 (and all 8 bins in the entire grouping)
    bin11_a.gantry = {4.0, -1.1, 0.}; // the exact same as bin311_a
    bin11_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin11_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin11_a.name = GET_VARIABLE_NAME(bin11_a);



    // rename this to general far pick
    shelf11_south_far.gantry = {-13.52-0.172656, 1.96, -PI/2}; // WORKS FOR LEFT SHELF PULLEY NOT RIGHT
    shelf11_south_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf11_south_far.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf11_south_far.name = GET_VARIABLE_NAME(shelf11_south_far);



    ///////////// Shelf Preset Locations: arranged in rows from Northernmost (world +y direction) row to Southernmost (world -y direction) row,
    ////////////                          each row is arranged from left (-x) to right (+x). This section is for the shelves only.

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 1
    bottom_left_staging_a.gantry = {-14.22, -6.75, 0.00};
    bottom_left_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    bottom_left_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same except for joint 0
    bottom_left_staging_a.name = GET_VARIABLE_NAME(bottom_left_staging_a);

    waitpoint_best_north_fromNorth.gantry = {-11.0, -6.9, 0.88}; // waitpoint_best_north_fromNorth
    waitpoint_best_north_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_north_fromNorth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    waitpoint_best_north_fromNorth.name = GET_VARIABLE_NAME(waitpoint_best_north_fromNorth);

    waitpoint_best_north_fromSouth.gantry = {-11.0, -6.9, 0.88-PI}; // waitpoint_best_north_fromSouth
    waitpoint_best_north_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_north_fromSouth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    waitpoint_best_north_fromSouth.name = GET_VARIABLE_NAME(waitpoint_best_north_fromSouth);

    // joint positions to go to agv1
    agv1_staging_a.gantry = {0.6, -6.9, 0.00};
    agv1_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    agv1_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same except for joint 0
    agv1_staging_a.name = GET_VARIABLE_NAME(agv1_staging_a);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 2
    shelf5_a.gantry = {-14.42, -4.30, 0.00}; // todo why is this -14.42 instead of -14.22
    shelf5_a.left_arm = {-1.76, -1.00, 1.86, -.85, -.20, 0.0};
    shelf5_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf5_a.name = GET_VARIABLE_NAME(shelf5_a);

    shelf5_fromNorth_near.gantry = {-14.22, -4.30, 0.88}; // shelf5_fromNorth_near
    shelf5_fromNorth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf5_fromNorth_near.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf5_fromNorth_near.name = GET_VARIABLE_NAME(shelf5_fromNorth_near);


    shelf5_fromNorth_far.gantry = {-14.22, -4.30, 0.88}; // shelf5_fromNorth_far
    shelf5_fromNorth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf5_fromNorth_far.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf5_fromNorth_far.name = GET_VARIABLE_NAME(shelf5_fromNorth_far);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 3
    shelf8_a.gantry = {-14.22, -1.5, 0.00};
    shelf8_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf8_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf8_a.name = GET_VARIABLE_NAME(shelf8_a);


    shelf5_fromSouth_near.gantry = {-14.22, -1.5, 0.88-PI}; // shelf5_fromSouth_near
    shelf5_fromSouth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf5_fromSouth_near.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf5_fromSouth_near.name = GET_VARIABLE_NAME(shelf5_fromSouth_near);


    shelf5_fromSouth_far.gantry = {-14.22, -1.5, 0.88-PI}; // shelf5_fromSouth_far
    shelf5_fromSouth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf5_fromSouth_far.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf5_fromSouth_far.name = GET_VARIABLE_NAME(shelf5_fromSouth_far);


    shelf8_fromNorth_near.gantry = {-14.22, -1.5, 0.88}; // shelf8_fromNorth_near
    shelf8_fromNorth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf8_fromNorth_near.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf8_fromNorth_near.name = GET_VARIABLE_NAME(shelf8_fromNorth_near);


    shelf8_fromNorth_far.gantry = {-14.22, -1.5, 0.88}; // shelf8_fromNorth_far
    shelf8_fromNorth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf8_fromNorth_far.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf8_fromNorth_far.name = GET_VARIABLE_NAME(shelf8_fromNorth_far);



    mid_5_8_intersection_fromNorth.gantry = {-11.0, -1.5, 0.88}; // Intersection!
    mid_5_8_intersection_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_intersection_fromNorth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_5_8_intersection_fromNorth.name = GET_VARIABLE_NAME(mid_5_8_intersection_fromNorth);

    mid_5_8_intersection_fromSouth.gantry = {-11.0, -1.5, 0.88-PI}; // mid_5_8_intersection_fromSouth
    mid_5_8_intersection_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_intersection_fromSouth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_5_8_intersection_fromSouth.name = GET_VARIABLE_NAME(mid_5_8_intersection_fromSouth);

    mid_5_8_staging_a.gantry = {0.0, -1.5, 0.00};
    mid_5_8_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.name = GET_VARIABLE_NAME(mid_5_8_staging_a);


    mid_5_8_staging_fromNorth.gantry = {0.0, -1.5, 0.88}; // mid_5_8_staging_fromNorth
    mid_5_8_staging_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_staging_fromNorth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_5_8_staging_fromNorth.name = GET_VARIABLE_NAME(mid_5_8_staging_fromNorth);

    mid_5_8_staging_fromSouth.gantry = {0.0, -1.5, 0.88-PI}; // mid_5_8_staging_fromSouth
    mid_5_8_staging_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_staging_fromSouth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_5_8_staging_fromSouth.name = GET_VARIABLE_NAME(mid_5_8_staging_fromSouth);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 4
    shelf11_a.gantry = {-14.22, 1.5, 0.00};
    shelf11_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf11_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00}; // same as left except for joint 0
    shelf11_a.name = GET_VARIABLE_NAME(shelf11_a);

    shelf8_fromSouth_near.gantry = {-14.22, 1.5, 0.88-PI}; // shelf8_fromSouth_near
    shelf8_fromSouth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf8_fromSouth_near.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf8_fromSouth_near.name = GET_VARIABLE_NAME(shelf8_fromSouth_near);

    shelf8_fromSouth_far.gantry = {-14.22, 1.5, 0.88-PI}; // shelf8_fromSouth_far
    shelf8_fromSouth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf8_fromSouth_far.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf8_fromSouth_far.name = GET_VARIABLE_NAME(shelf8_fromSouth_far);

    shelf11_fromNorth_near.gantry = {-14.22, 1.5, 0.88}; // shelf11_fromNorth_near
    shelf11_fromNorth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf11_fromNorth_near.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf11_fromNorth_near.name = GET_VARIABLE_NAME(shelf11_fromNorth_near);

    shelf11_fromNorth_far.gantry = {-14.22, 1.5, 0.88}; // shelf11_fromNorth_far
    shelf11_fromNorth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf11_fromNorth_far.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf11_fromNorth_far.name = GET_VARIABLE_NAME(shelf11_fromNorth_far);


    mid_8_11_intersection_fromNorth.gantry = {-11.0, 1.5, 0.88}; // mid_8_11_intersection_fromNorth
    mid_8_11_intersection_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_intersection_fromNorth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_8_11_intersection_fromNorth.name = GET_VARIABLE_NAME(mid_8_11_intersection_fromNorth);

    mid_8_11_intersection_fromSouth.gantry = {-11.0, 1.5, 0.88-PI}; // mid_8_11_intersection_fromSouth
    mid_8_11_intersection_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_intersection_fromSouth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_8_11_intersection_fromSouth.name = GET_VARIABLE_NAME(mid_8_11_intersection_fromSouth);

    mid_8_11_staging_a.gantry = {0.0, 1.5, 0.00};
    mid_8_11_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.name = GET_VARIABLE_NAME(mid_8_11_staging_a);


    mid_8_11_staging_fromNorth.gantry = {0.0, 1.5, 0.88}; // mid_8_11_staging_fromNorth
    mid_8_11_staging_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_staging_fromNorth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_8_11_staging_fromNorth.name = GET_VARIABLE_NAME(mid_8_11_staging_fromNorth);

    mid_8_11_staging_fromSouth.gantry = {0.0, 1.5, 0.88-PI}; // mid_8_11_staging_fromSouth
    mid_8_11_staging_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_staging_fromSouth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    mid_8_11_staging_fromSouth.name = GET_VARIABLE_NAME(mid_8_11_staging_fromSouth);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 5

    shelf11_fromSouth_near.gantry = {-14.22, 4.30, 0.88-PI}; // shelf11_fromSouth_near
    shelf11_fromSouth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf11_fromSouth_near.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf11_fromSouth_near.name = GET_VARIABLE_NAME(shelf11_fromSouth_near);

    shelf11_fromSouth_far.gantry = {-14.22, 4.30, 0.88-PI}; // shelf11_fromSouth_far
    shelf11_fromSouth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf11_fromSouth_far.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    shelf11_fromSouth_far.name = GET_VARIABLE_NAME(shelf11_fromSouth_far);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 6

    southwest_corner_staging.gantry = {-14.22, 6.9, PI}; // southwest_corner_staging
    southwest_corner_staging.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    southwest_corner_staging.right_arm = {PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    southwest_corner_staging.name = GET_VARIABLE_NAME(southwest_corner_staging);


    waitpoint_best_south_fromNorth.gantry = {-11.0, 6.9, 0.88}; // waitpoint_best_south_fromNorth
    waitpoint_best_south_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_south_fromNorth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    waitpoint_best_south_fromNorth.name = GET_VARIABLE_NAME(waitpoint_best_south_fromNorth);

    waitpoint_best_south_fromSouth.gantry = {-11.0, 6.9, 0.88-PI}; // waitpoint_best_south_fromSouth
    waitpoint_best_south_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_south_fromSouth.right_arm = {PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // right elbow bent more
    waitpoint_best_south_fromSouth.name = GET_VARIABLE_NAME(waitpoint_best_south_fromSouth);

    // joint positions to go to agv2
    agv2_a.gantry = {0.6, 6.9, PI};
    agv2_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.name = GET_VARIABLE_NAME(agv2_a);

    //////////// End Shelf Preset Locations
    ////////////
    

    cam_to_y_coordinate = {
        {0, 1.750927},
        {7, 1.698910},
        {9, 3.006604},
        {12, 3.006604},

        {6, 0.000000},
        {16, 0.000000},

        {8, -3.024268},
        {11, -3.024268},

        {1, -1.794935},
        {2, -1.806997},

        {3, 6.582773},
        {4, -7.175601},
        {5, 4.2},
        {10, -3.024268},
        {13, 3.78},
        {14, -3.024268},
        {15, 3.78},
        {16, 0.000000},
    };



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

    cam_to_shelf_string = {
        // {0, "bin3_a"},
        // {7, "bin3_a"},
        {9, "shelf5"},
        {12, "shelf5"},
        {6, "shelf8"},
        {16, "shelf8"},
        {8, "shelf11"},
        {11, "shelf11"},
        // {1, "bin11_a"},
        // {2, "bin11_a"},
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

        ////////////////////////////////////////////////////////////////////////////////////////////////// Row by row, left to right
        {{"start_a", "waitpoint_best_north_fromNorth"}, std::vector<PresetLocation>{start_a, agv1_staging_a, waitpoint_best_north_fromNorth}},
        {{"start_a", "waitpoint_best_north_fromSouth"}, std::vector<PresetLocation>{start_a, agv1_staging_a, waitpoint_best_north_fromSouth}},

        {{"start_a", "shelf5_fromNorth_near"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_near}},
        {{"start_a", "shelf5_fromNorth_far"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_far}},

        {{"start_a", "shelf5_fromSouth_near"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near}},
        {{"start_a", "shelf5_fromSouth_far"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near, shelf5_fromSouth_far}},
        {{"start_a", "shelf8_fromNorth_near"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near}},
        {{"start_a", "shelf8_fromNorth_far"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near, shelf8_fromNorth_far}},
        // {{"start_a", "mid_5_8_intersection_fromNorth"}, std::vector<PresetLocation>{start_a, 222, mid_5_8_intersection_fromNorth}},
        // {{"start_a", "mid_5_8_intersection_fromSouth"}, std::vector<PresetLocation>{start_a, 222, mid_5_8_intersection_fromSouth}},

        {{"start_a", "mid_5_8_staging_fromNorth"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth}},
        {{"start_a", "mid_5_8_staging_fromSouth"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth}},

        {{"start_a", "shelf8_fromSouth_near"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near}},
        {{"start_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"start_a", "shelf11_fromNorth_near"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near}},
        {{"start_a", "shelf11_fromNorth_far"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near, shelf11_fromNorth_far}},
        // {{"start_a", "mid_8_11_intersection_fromNorth"}, std::vector<PresetLocation>{start_a, 222, mid_8_11_intersection_fromNorth}},
        // {{"start_a", "mid_8_11_intersection_fromSouth"}, std::vector<PresetLocation>{start_a, 222, mid_8_11_intersection_fromSouth}},
        {{"start_a", "mid_8_11_staging_fromNorth"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth}},
        {{"start_a", "mid_8_11_staging_fromSouth"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth}},

        {{"start_a", "shelf11_fromSouth_near"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_near}},
        {{"start_a", "shelf11_fromSouth_far"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_far}},
        
        {{"start_a", "southwest_corner_staging"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging}},
        {{"start_a", "waitpoint_best_south_fromNorth"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromNorth}},
        {{"start_a", "waitpoint_best_south_fromSouth"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromSouth}},


        ////////////////////////////////////////////////////////////////////////////////////////////////// Row by row, left to right REVERSED
        // {{"start_a", "waitpoint_best_north_fromNorth"}, std::vector<PresetLocation>{start_a, agv1_staging_a, waitpoint_best_north_fromNorth}},
        // {{"start_a", "waitpoint_best_north_fromSouth"}, std::vector<PresetLocation>{start_a, agv1_staging_a, waitpoint_best_north_fromSouth}},

        // {{"start_a", "shelf5_fromNorth_near"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_near}},
        // {{"start_a", "shelf5_fromNorth_far"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_far}},

        // {{"start_a", "shelf5_fromSouth_near"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near}},
        // {{"start_a", "shelf5_fromSouth_far"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near, shelf5_fromSouth_far}},
        // {{"start_a", "shelf8_fromNorth_near"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near}},
        // {{"start_a", "shelf8_fromNorth_far"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near, shelf8_fromNorth_far}},
        // // {{"start_a", "mid_5_8_intersection_fromNorth"}, std::vector<PresetLocation>{start_a, 222, mid_5_8_intersection_fromNorth}},
        // // {{"start_a", "mid_5_8_intersection_fromSouth"}, std::vector<PresetLocation>{start_a, 222, mid_5_8_intersection_fromSouth}},

        // {{"start_a", "mid_5_8_staging_fromNorth"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth}},
        // {{"start_a", "mid_5_8_staging_fromSouth"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth}},

        // {{"start_a", "shelf8_fromSouth_near"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near}},
        // {{"start_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        // {{"start_a", "shelf11_fromNorth_near"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near}},
        // {{"start_a", "shelf11_fromNorth_far"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near, shelf11_fromNorth_far}},
        // // {{"start_a", "mid_8_11_intersection_fromNorth"}, std::vector<PresetLocation>{start_a, 222, mid_8_11_intersection_fromNorth}},
        // // {{"start_a", "mid_8_11_intersection_fromSouth"}, std::vector<PresetLocation>{start_a, 222, mid_8_11_intersection_fromSouth}},
        // {{"start_a", "mid_8_11_staging_fromNorth"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth}},
        // {{"start_a", "mid_8_11_staging_fromSouth"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth}},

        // {{"start_a", "shelf11_fromSouth_near"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_near}},
        // {{"start_a", "shelf11_fromSouth_far"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_far}},
        
        // {{"start_a", "southwest_corner_staging"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging}},
        // {{"start_a", "waitpoint_best_south_fromNorth"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromNorth}},
        // {{"start_a", "waitpoint_best_south_fromSouth"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromSouth}},



    };
}

// Note: this function is called once in the constructor of RWAImplementation
void RWAImplementation::InitRegionDictionaryDependingOnSituation() {

    ROS_INFO_STREAM(" Human Obstacles not initialized, Error, need to add some code here ************************************************");
    std::vector<bool> clear = {false, true, true, false}; // [Northernmost Lane Row .......... Southermost Lane Row]

    // First deal with case of both inner lanes taken
    if (clear[1] == false && clear[2] == false) {
        // do stuff
    }
    else {

        if (clear[0] == true) { regionDictionary["shelf5upper"] = {"shelf5_fromNorth_near", "nowait", "fromNorth", "near"}; }
        else if (clear[1] == true) { regionDictionary["shelf5upper"] = {"shelf5_fromNorth_near", "nowait", "fromNorth"}; }
        else if (clear[0] == false && clear[1] == false) { regionDictionary["shelf5upper"] = {"shelf5_fromNorth_near", "wait", "fromNorth", "near"}; }


        if (clear[1] == true) { regionDictionary["shelf5lower"] = {"shelf5_fromSouth_near", "nowait", "fromSouth", "near"}; }
        else if (clear[0] == true) { regionDictionary["shelf5lower"] = {"shelf5_fromNorth_far", "nowait", "fromNorth", "far"}; }
        else if (clear[0] == false && clear[1] == false) { regionDictionary["shelf5lower"] = {"shelf5_fromNorth_far", "wait", "fromNorth", "far"}; }
        

        if (clear[1] == true) { regionDictionary["shelf8upper"] = {"shelf8_fromNorth_near", "nowait", "fromNorth", "near"}; }
        else if (clear[2] == true) { regionDictionary["shelf8upper"] = {"shelf8_fromSouth_far", "nowait", "fromSouth", "far"}; }
        else if (clear[1] == false && clear[2] == false) { // Both innermost lanes not clear case, must account for Shelf Gaps

            ROS_INFO_STREAM(" Shelf Gaps not implemented, Error, need to add some code here ************************************************");

            if ( true ) { // this code block: for testing only
                regionDictionary["shelf8upper"] = {"shelf8_fromSouth_far", "wait", "fromSouth", "far"};
            }
            else if ( false ) {
                regionDictionary["shelf8upper"] = {"shelf8_fromNorth_near", "wait", "fromNorth", "near"};
            }

            // if ( Gap is between shelf 11 and shelf 10) {
            //     regionDictionary["shelf8upper"] = {"shelf8_fromSouth_far", "wait", "fromSouth", "far"};
            // }
            // else if ( Gap is between shelf 5 and shelf 4) {
            //     regionDictionary["shelf8upper"] = {"shelf8_fromNorth_near", "wait", "fromNorth", "near"};
            // }
        }


        if (clear[2] == true) { regionDictionary["shelf8lower"] = {"shelf8_fromSouth_near", "nowait", "fromSouth", "near"}; }
        else if (clear[1] == true) { regionDictionary["shelf8lower"] = {"shelf8_fromNorth_far", "nowait", "fromNorth", "far"}; }
        else if (clear[1] == false && clear[2] == false) { // Both innermost lanes not clear case, must account for Shelf Gaps

            ROS_INFO_STREAM(" Shelf Gaps not implemented, Error, need to add some code here ************************************************");

            if ( true ) { // this code block: for testing only
                regionDictionary["shelf8lower"] = {"shelf8_fromSouth_near", "wait", "fromSouth", "near"};
            }
            else if ( false ) {
                regionDictionary["shelf8lower"] = {"shelf8_fromNorth_far", "wait", "fromNorth", "far"};
            }

            // if ( Gap is between shelf 11 and shelf 10) {
            //     regionDictionary["shelf8lower"] = {"shelf8_fromSouth_near", "wait", "fromSouth", "near"};
            // }
            // else if ( Gap is between shelf 5 and shelf 4) {
            //     regionDictionary["shelf8lower"] = {"shelf8_fromNorth_far", "wait", "fromNorth", "far"};
            // }
        }


        if (clear[2] == true) { regionDictionary["shelf11upper"] = {"shelf11_fromNorth_near", "nowait", "fromNorth", "near"}; }
        else if (clear[3] == true) { regionDictionary["shelf11upper"] = {"shelf5_fromSouth_far", "nowait", "fromSouth", "far"}; }
        else if (clear[2] == false && clear[3] == false) { regionDictionary["shelf11upper"] = {"shelf11_fromSouth_near", "wait", "fromSouth", "near"}; }


        if (clear[3] == true) { regionDictionary["shelf11lower"] = {"shelf11_fromSouth_near", "nowait", "fromSouth", "near"}; }
        else if (clear[2] == true) { regionDictionary["shelf11lower"] = {"shelf11_fromNorth_far", "nowait", "fromNorth", "far"}; }
        else if (clear[2] == false && clear[3] == false) { regionDictionary["shelf11lower"] = {"shelf11_fromNorth_near", "wait", "fromNorth", "near"}; }

    }


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
    else if ((discovered_cam_idx != 0 || discovered_cam_idx != 7 || discovered_cam_idx != 1 || discovered_cam_idx != 2) && discovered_cam_idx >= 0 && discovered_cam_idx <= 16) // Any other camera
    {                                                                 // any other camera
        ROS_INFO_STREAM("Any Other Camera Case Reached");

        ///////////////////////////////////////////////////////////////////////////////////////
        std:: string shelf_and_upper_or_lower_string = isPartInUpperOrLowerRegionOfWhichShelf(my_part, discovered_cam_idx); // ie. "shelf5upper"
        std::vector<std::string> information_string_vector = regionDictionary[shelf_and_upper_or_lower_string]; // ie. ["shelf5_fromNorth_near", "nowait", "fromNorth", "near"]
        
        std::string preset_location_string = information_string_vector[0]; // ie. "shelf5_fromNorth_near"
        std::string wait_or_nowait_string = information_string_vector[1]; // ie. "nowait"
        std::string fromNorth_or_fromSouth_string = information_string_vector[2]; // ie. "fromNorth"
        std::string near_or_far_string = information_string_vector[3]; // ie. "near"

        // Create Bump() offsets, 4 possible scenarios
        ROS_INFO_STREAM("Calculating offsets...");
        double add_to_x_shelf = my_part.pose.position.x - -13.522081; // constant is perfect bin red pulley x
        double add_to_y_shelf = my_part.pose.position.y - 3.446263; // constant is perfect bin red pulley y
        double add_to_torso = 0.0;

        if (near_or_far_string == "near" && fromNorth_or_fromSouth_string == "fromSouth") {
            ROS_INFO_STREAM("Case 1 fromSouth near encountered for offsets...");

            add_to_x_shelf = my_part.pose.position.x - -13.522081      + 1.795838; // red pulley + offset to account for spin
            add_to_y_shelf = my_part.pose.position.y - 3.446263        - 1.707474; // constant is perfect bin red pulley y
            add_to_torso = PI; // spin torso
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



        std::vector<PresetLocation> path = getPresetLocationVectorUsingString(preset_location_string); // initialize with no wait
        if (wait_or_nowait_string == "wait") {
            ROS_INFO_STREAM("Wait condtion encountered, need to use wait dictionary situation here ******************");
            // Lookup PresetLocation in waitDictionary
            // std::vector<PresetLocation> path = getPresetLocationVector(cam_to_presetlocation[discovered_cam_idx]); // todo lookup in wait dictionary here!
            ROS_INFO_STREAM("getPresetLocationVector executed!");

            executeVectorOfPresetLocations(path);
            ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

            if (near_or_far_string == "near") { gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, add_to_torso)); }
            else if (near_or_far_string == "far") { gantry_->goToPresetLocation(Bump(shelf11_south_far, add_to_x_shelf, add_to_y_shelf, add_to_torso));}

            ROS_INFO_STREAM("goToPresetLocation with bump executed!");

            ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away
        }
        else { // else the "nowait" case
            ROS_INFO_STREAM("No Wait condtion encountered");
            // Lookup PresetLocation in noWaitDictionary
            std::vector<PresetLocation> path = getPresetLocationVectorUsingString(preset_location_string); // initialize with no wait
            ROS_INFO_STREAM("getPresetLocationVector executed!");

            executeVectorOfPresetLocations(path);
            ROS_INFO_STREAM("executeVectorOfPresetLocations executed!");

            if (near_or_far_string == "near") { gantry_->goToPresetLocation(Bump(shelf5_a, add_to_x_shelf, add_to_y_shelf, add_to_torso)); }
            else if (near_or_far_string == "far") { gantry_->goToPresetLocation(Bump(shelf11_south_far, add_to_x_shelf, add_to_y_shelf, add_to_torso));}

            gantry_->goToPresetLocation(Bump(shelf11_a, add_to_x_shelf, add_to_y_shelf, add_to_torso));
            ROS_INFO_STREAM("goToPresetLocation with bump executed!");

            ros::Duration(1.0).sleep(); // upped to 1.0 from 0.5 to keep red errors away

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

std::vector<PresetLocation> RWAImplementation::getPresetLocationVectorUsingString(std::string target_preset_location_string)
{
    PresetLocation approximate_current_position = getNearesetPresetLocation();
    std::vector<std::string> key = {{approximate_current_position.name, target_preset_location_string}};
    ROS_INFO_STREAM("key executed! =====" << key[0] << " " << key[1]);
    std::vector<PresetLocation> path_to_execute = PathingLookupDictionary.at(key);
    ROS_INFO_STREAM("path_to_execute lookup executed!");
    return path_to_execute;
}

bool RWAImplementation::executeVectorOfPresetLocations(std::vector<PresetLocation> path_to_execute)
{
    for (PresetLocation psetlocation : path_to_execute)
    {
        ROS_INFO_STREAM("Moving to PresetLocation: >>>>> " << psetlocation.name);
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
                ROS_INFO_STREAM("\nCHECK FOR PART " << part_to_check.type);
                //correct world pose in tray as per order
                auto correct_world_pose = part_to_check.target_pose;
                //check delta between the above and the actual position determined by camera
                auto delta_x = std::abs(model.world_pose.position.x) - std::abs(correct_world_pose.position.x);
                auto delta_y = std::abs(model.world_pose.position.y) - std::abs(correct_world_pose.position.y);
                auto delta_z = std::abs(model.world_pose.position.z) - std::abs(correct_world_pose.position.z);
                auto delta_deg = std::abs(model.world_pose.orientation.z) - std::abs(correct_world_pose.orientation.z); //---

                ROS_INFO_STREAM("CURRENT WORLD POSE IN TRAY = " << model.world_pose);
                ROS_INFO_STREAM("DESIRED WORLD POSE = " << correct_world_pose);
                ROS_INFO_STREAM("Deltas = " << delta_x << " ; " << delta_y << " ; " << delta_z << " ; " << delta_deg);

                if (delta_x <= 0.15 && delta_y <= 0.15 && delta_z <= 0.20 && delta_deg <= 0.02) //---
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