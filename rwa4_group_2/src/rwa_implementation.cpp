#include "include/rwa_implementation.h"

void RWAImplementation::processOrder(Order order) {
    std::vector<Product> total_products;
    std::vector<Product> products;
    std::vector<std::string> total_products_agv_id;
    product product;
    CameraListener::ModelInfo pickPart;
    std::vector<Order> orders;
    std::vector<Shipment> shipments;
    std::string shipment_type;
    std::string agv_id;
    std::string kit_id;
    bool partFound = false;
    bool isPartFaulty = true; // assume part is faulty (repeat while loop) until we see otherwise

    for(const auto& shipment : order.shipments) {
        shipment_type = shipment.shipment_type;
        agv_id = shipment.agv_id;
        products = shipment.products;

        ROS_INFO_STREAM("Shipment type = " << shipment_type);
        ROS_INFO_STREAM("AGV ID = " << agv_id);

        ROS_INFO_STREAM("Shipment size = " << order.shipments.size());

        for (auto product : products) { ////////// FOR PRODUCT IN PRODUCTS
            partFound = false;
            isPartFaulty = true;
            ROS_INFO_STREAM("Product type = " << product.type);
            total_products.push_back(product);
            total_products_agv_id.push_back(agv_id);
        }
    }
    cam_listener_.fetchParts(node_);
    cam_listener_.sort_camera_parts_list();
    auto sorted_map = cam_listener_.ordered_color_type;

    // Choose parts closest to AGV

    // Find part regions

    // Match two parts in a given region
    std::vector<CameraListener::ModelInfo> task_pair{};

    // Add parts to FIFO queue
    task_queue_.push(task_pair);

}

void RWAImplementation::checkConveyor(bool part_wanted) {

    if (!waiting_for_part_ && !cam_listener_.load_time_on_conveyor_.empty()) {
        auto cam_parts = cam_listener_.fetchParts(node_)[5];
        for (const auto& found_part : cam_parts) {
            bool part_in_queue{false};
            for (const auto& part: parts_on_conveyor_) {
                if (part.id == found_part.id) {
                    part_in_queue = true;
                    break;
                }
            } 
            if (!part_in_queue) parts_on_conveyor_.push_back(found_part);
        }
        waiting_for_part_ = true;
        current_part_load_time_ = cam_listener_.load_time_on_conveyor_.front();
        current_part_on_conveyor_ = parts_on_conveyor_.front();
        cam_listener_.load_time_on_conveyor_.pop();
        parts_on_conveyor_.pop_front();
        gantry_.goToPresetLocation(conveyor_belt);
    } else if (part_wanted && waiting_for_part_ && (ros::Time::now()-current_part_load_time_).toSec() > dx_/cam_listener_.conveyor_spd_) {
        ROS_INFO_STREAM("Pick up part now!");
        part temp_part;
        temp_part.type = current_part_on_conveyor_.type;
        temp_part.pose = current_part_on_conveyor_.world_pose;
        temp_part.pose.position.y = -2.5;
        ROS_INFO_STREAM("x: " << temp_part.pose.position.x << ", y: " << temp_part.pose.position.y << ", z: " << temp_part.pose.position.z);
        // current_part_on_conveyor.world_pose
        waiting_for_part_ = false;
        gantry_.pickPart(temp_part, "left_arm");
    }
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
}