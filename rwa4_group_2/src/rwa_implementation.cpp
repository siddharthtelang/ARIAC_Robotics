#include "rwa_implementation.h"
#include <unordered_map>

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
            } else product.get_from_conveyor = true;
            total_products[color][type].push(product);
        }
    }
         
    std::queue<Product> needs_pair;
    std::queue<std::vector<Product>> order_task_queue;
    for(auto color : cam_listener_->colors_) {
        for(auto type : cam_listener_->types_) {
            auto similar_parts = total_products[color][type];
            while(similar_parts.size() > 1){
                std::vector<Product> task_pair{};
                task_pair.push_back(similar_parts.front());
                similar_parts.pop();
                task_pair.push_back(similar_parts.front());
                similar_parts.pop();
                order_task_queue.push(task_pair);
            }
            if(similar_parts.size() == 1) {
                if(needs_pair.size() > 0) {
                    std::vector<Product> task_pair{};
                    task_pair.push_back(needs_pair.front());
                    needs_pair.pop();
                    task_pair.push_back(similar_parts.front());
                    similar_parts.pop();
                    order_task_queue.push(task_pair);
                } else needs_pair.push(similar_parts.front());
            }
        }
    }

    while(needs_pair.size() > 0) {
        order_task_queue.push(std::vector<Product>{needs_pair.front()});
        needs_pair.pop();
    }
    task_queue_.push(order_task_queue);
    ROS_INFO_STREAM("Completed order processing. All parts on task queue.");

    // // CANNOT ITERATE OVER QUEUE. THIS PART IS JUST FOR TESTING. WILL NOT WORK IF UNCOMMENTED DURING EXECUTION
    ROS_INFO_STREAM("");
    auto task_queue = task_queue_.top();
    while(!task_queue.empty()) {
        auto vec = task_queue.front();
        ROS_INFO_STREAM("Vector on queue.");
        for(auto product : vec) {
            ROS_INFO_STREAM("  Product on queue.");
            ROS_INFO_STREAM("  Product type: " << product.type);
            ROS_INFO_STREAM("  Product target AGV: " << product.agv_id);
            ROS_INFO_STREAM("    Designated model id: " << product.designated_model.id);
            ROS_INFO_STREAM("    Designated model type: " << product.designated_model.color << " " << product.designated_model.type << "\n");
        }
        task_queue.pop();
    }
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

    cam_to_presetlocation = {
        {0, bin3_a},
        {7, bin3_a},
        {9, shelf5_a},
        {12, shelf5_a}
    };
}


void RWAImplementation::buildKit() {
    Product product = task_queue_.top().front()[0];
    part my_part;
    my_part.type = product.type;
    my_part.pose = product.designated_model.world_pose;

    part part_in_tray;
    part_in_tray.type = task_queue_.top().front()[0].type;
    part_in_tray.pose = task_queue_.top().front()[0].pose;

    double add_to_x = my_part.pose.position.x - 4.365789 - 0.1; // constant is perfect bin red pulley x
    double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y
    // double add_to_x = my_part.pose.position.x - 4.665789; // constant is perfect bin red pulley x
    // double add_to_y = my_part.pose.position.y - 1.173381; // constant is perfect bin red pulley y

    ROS_INFO_STREAM(" x " << add_to_x << " y " << add_to_y);

    /********************   Hard Coded     ****************/
    gantry_->goToPresetLocation(bin3_a);
//    gantry_->goToPresetLocation(Bump(bin3_a, add_to_x, add_to_y, 0));
//    gantry_->goToPresetLocation(Bump(cam_to_presetlocation[discovered_cam_idx], add_to_x, add_to_y, 0));
    /********************   Hard Coded     ****************/

    //--Go pick the part
    if (!gantry_->pickPart(my_part, "left_arm")){
        // gantry.goToPresetLocation(gantry.start_);
        // spinner.stop();
        // ros::shutdown();
        ;//pass
    }

    // go back to start
    gantry_->goToPresetLocation(start_a);
    ros::Duration(1.0).sleep(); // make sure it actually goes back to start, instead of running into shelves

    //place the part
    gantry_->placePart(part_in_tray, product.agv_id, "left_arm");
    task_queue_.pop();
//    gantry_->goToPresetLocation(start_a);
}


void RWAImplementation::checkAgvErrors() {
    /************** Check for Faulty Parts *****************/

}