#pragma once

/* ========= Std Libs ========= */
#include <queue>
#include <vector>
#include <stack>

/* ========= ARIAC Libs ========= */
#include <nist_gear/Proximity.h>

/* ========= ROS Libs ========= */
#include <ros/ros.h>

/* ========= User Libs ========= */
#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "agv_control.h"
#include "camera_listener.h"

#define pi 3.14159265895866

class RWAImplementation
{
private:
    ros::NodeHandle* node_;

    /* ===================== Gazebo Control ===================== */
    CameraListener* cam_listener_;
    GantryControl* gantry_;
    Competition* competition_;
    AGVControl *agv_control_;

    // ORDERS < SHIPMENTS < PRODUCTS  
    std::stack<std::queue<std::vector<Product>>> task_queue_;
    std::stack<std::queue<std::string>> current_shipments;

    std::unordered_map<std::string, std::unordered_map<std::string, std::priority_queue<CameraListener::ModelInfo,
    std::vector<CameraListener::ModelInfo>, CameraListener::CompareDists>>> sorted_map;
    // track of parts in tray
	std::array<std::vector <part>,2> parts_in_tray;

    /* ===================== Subscribers ===================== */
    ros::Subscriber breakbeam_sub_;

    /* ===================== Preset Locations ===================== */
    PresetLocation conveyor_belt;
    PresetLocation start_a;
    PresetLocation bin3_a; // for bin3 (and all 8 bins in that entire grouping)
    PresetLocation bingreen_a;
    PresetLocation binblue_a;
    PresetLocation agv2_a;
    PresetLocation agv1_staging_a;
    PresetLocation bottom_left_staging_a;
    PresetLocation shelf5_a;
    PresetLocation shelf5_spun_a;

    PresetLocation mid_5_8_staging_a;
    PresetLocation mid_8_11_staging_a;

    PresetLocation shelf8_a;
    PresetLocation shelf11_a;

    PresetLocation bin11_a; // for bin11 (and all 8 bins in that entire grouping)

    std::vector<PresetLocation> preset_locations_list_; // lookup list to compare distances with

    /* ===================== Conveyor Variables ===================== */
    const float dx_ = 6.6;
    const int buffer_parts_{1};
    int buffer_parts_collected{0};
    bool waiting_for_part_{false};
    std::string conveyor_type_{""};
    ros::Time current_part_load_time_;
    CameraListener::ModelInfo current_part_on_conveyor_;
    std::deque<CameraListener::ModelInfo> parts_on_conveyor_; 

    /* ===================== Misc Variables ===================== */
    int prev_num_orders_{0};

    std::map<int, PresetLocation> cam_to_presetlocation;
    std::map<std::string, int> agv_to_camera;
    bool competition_started_{false};


public:
    RWAImplementation(ros::NodeHandle& node, CameraListener& camera_listener, GantryControl& gantry, Competition& competition, AGVControl& agv_control) :
        node_{&node}, cam_listener_{&camera_listener}, gantry_{&gantry}, competition_{&competition}, agv_control_{&agv_control}
    {
        initPresetLocs();
        breakbeam_sub_ = node_->subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, cam_listener_);
        cam_listener_->fetchParts(*node_);
        cam_listener_->sort_camera_parts_list();
        sorted_map = cam_listener_->sortPartsByDist();
    };

    bool checkConveyor();
    void initPresetLocs();
    void processOrder();
    void buildKit();
    void checkAgvErrors();
    bool checkAndCorrectPose(std::string agv_id);
    bool competition_over();

    struct distance_and_PresetLocation_struct
    {
        double distance;
        PresetLocation candidate_location;
    };

    template <typename Container> // we can make this generic for any container [1]
    struct container_hash {
        std::size_t operator()(Container const& c) const {
            return boost::hash_range(c.begin(), c.end());
        }
    };


    std::unordered_map<std::vector<std::string>, std::vector<PresetLocation>, container_hash<std::vector<std::string>> > PathingLookupDictionary;

    double calcDistanceInXYPlane(geometry_msgs::Pose a, geometry_msgs::Pose b);
    PresetLocation getNearesetPresetLocation();
    std::vector<PresetLocation> getPresetLocationVector(PresetLocation target_preset_location);
    bool executeVectorOfPresetLocations( std::vector<PresetLocation> path_to_execute );
    geometry_msgs::Pose gantryXY2worldposeXY(PresetLocation preset_location_2_convert);

    PresetLocation getNearestBinPresetLocation();



    bool simpleDropPart() {
        gantry_->deactivateGripper("left_arm");
        return true;
    }
};
