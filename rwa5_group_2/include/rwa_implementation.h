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

/**
 * \brief :A separate RWAImplentation class to have all the functionalities
 * \param :references of node, camera listener, gantry controller, and agv control classes
 */
public:
    RWAImplementation(ros::NodeHandle& node, CameraListener& camera_listener, GantryControl& gantry, Competition& competition, AGVControl& agv_control) :
        node_{&node}, cam_listener_{&camera_listener}, gantry_{&gantry}, competition_{&competition}, agv_control_{&agv_control},
        right_lane(node,1,2), mid_right_lane(node,3,4), mid_left_lane(node,5,6), left_lane(node, 7,8)
    {
        initPresetLocs();
        breakbeam_sub_ = node_->subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, cam_listener_);
        cam_listener_->fetchParts(*node_);
        cam_listener_->sort_camera_parts_list();
        sorted_map = cam_listener_->sortPartsByDist();
    };

    /**
     * \brief: check for conveyor items
     * \result: void
     */
    bool checkConveyor();

    /**
     * \brief: do an init of all preset locations
     * \result: void
     */
    void initPresetLocs();

    /**
     * \brief: process new incoming order
     * \result: void
     */
    void processOrder();

    /**
     * \brief: build kit functionality
     * \result: void
     */
    void buildKit();

    /**
     * \brief: check for agv errors: faulty part, call check pose
     * \result: void
     */
    void checkAgvErrors();

    /**
     * \brief: check pose of part and correct if incorrect
     * \param: agv id
     * \result: void
     */
    bool checkAndCorrectPose(std::string agv_id);

    /**
     * \brief: competition end trigger function
     * \result: true/false
     */
    bool competition_over();

    bool detectGaps();


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
    // preset locations from start to safe location for three shelf rows starting from agv1 side
    std::array<std::vector<PresetLocation>,3> shelf_preset_locations;
    std::array<std::string, 3> gaps; //either "gap_conveyor" or "gap_end"

    PresetLocation getNearestBinPresetLocation();

    /**
     * \brief: simple drop functionlity
     * \result: true
     */
    bool simpleDropPart() {
        gantry_->deactivateGripper("left_arm");
        return true;
    }

    /* ===================== Human Obstacle Variables ===================== */
    LaneBreakbeamPair left_lane;
    LaneBreakbeamPair mid_left_lane;
    LaneBreakbeamPair mid_right_lane;
    LaneBreakbeamPair right_lane;

};
