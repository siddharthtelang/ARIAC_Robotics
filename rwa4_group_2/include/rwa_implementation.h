#pragma once

/* ========= Std Libs ========= */
#include <queue>
#include <vector>

/* ========= ROS Libs ========= */
#include <ros/ros.h>

/* ========= User Libs ========= */
#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "agv_control.h"
#include "camera_listener.h"



class RWAImplementation
{
private:
    ros::NodeHandle& node_;

    /* ===================== Gazebo Control ===================== */
    CameraListener& cam_listener_;
    GantryControl& gantry_;
    std::queue<std::vector<CameraListener::ModelInfo>> task_queue_;

    /* ===================== Subscribers ===================== */
    ros::Subscriber breakbeam_sub_;

    /* ===================== Preset Locations ===================== */
    PresetLocation conveyor_belt;
    PresetLocation start_a;
    PresetLocation bin3_a;
    PresetLocation bingreen_a;
    PresetLocation binblue_a;
    PresetLocation agv2_a;
    PresetLocation agv1_staging_a;
    PresetLocation bottom_left_staging_a;
    PresetLocation shelf5_a;
    PresetLocation shelf5_spun_a;

    /* ===================== Conveyor Variables ===================== */
    const float dx_ = 3+4.492549-1;
    bool waiting_for_part_{false};
    ros::Time current_part_load_time_;
    CameraListener::ModelInfo current_part_on_conveyor_;
    std::deque<CameraListener::ModelInfo> parts_on_conveyor_; 

public:
    RWAImplementation(ros::NodeHandle& node, CameraListener& camera_listener, GantryControl& gantry) : 
        node_{node}, cam_listener_{camera_listener}, gantry_{gantry}
    {
        initPresetLocs();
        breakbeam_sub_ = node.subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, &cam_listener_);
    };

    void checkConveyor(bool part_wanted);
    void initPresetLocs();
    void processOrder(Order order);



};


