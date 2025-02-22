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
    std::queue<PresetLocation> empty_bins_;
    PresetLocation conveyor_belt;
    PresetLocation start_a;

    PresetLocation bin3_a; // for bin3 (and all 8 bins in that entire grouping)
    PresetLocation bin11_a; // for bin11 (and all 8 bins in that entire grouping)

    // PresetLocation bingreen_a;
    // PresetLocation binblue_a;
    // PresetLocation agv2_a;
    // PresetLocation agv1_staging_a;
    // PresetLocation bottom_left_staging_a;
    // PresetLocation shelf5_a;
    // PresetLocation shelf5_spun_a;

    // PresetLocation mid_5_8_staging_a;
    // PresetLocation mid_8_11_staging_a;

    // PresetLocation shelf8_a;
    // PresetLocation shelf11_a;

    PresetLocation shelf11_south_far; // rename this to general far pick

    ///////////// Shelf Preset Locations: arranged in rows from Northernmost (world +y direction) row to Southernmost (world -y direction) row,
    ////////////                          each row is arranged from left (-x) to right (+x). This section is for the shelves only.

    PresetLocation bottom_left_staging_a;
    PresetLocation waitpoint_best_north_fromNorth;
    PresetLocation waitpoint_best_north_fromSouth;
    PresetLocation agv1_staging_a;

    PresetLocation shelf5_a; // rename this to general near pick
    PresetLocation shelf5_fromNorth_near;
    PresetLocation shelf5_fromNorth_far;

    //////////// Row 3
    PresetLocation shelf8_a;
    PresetLocation shelf5_fromSouth_near;
    PresetLocation shelf5_fromSouth_far;
    PresetLocation shelf8_fromNorth_near;
    PresetLocation shelf8_fromNorth_far;
    PresetLocation mid_5_8_intersection_fromNorth;
    PresetLocation mid_5_8_intersection_fromSouth;
    PresetLocation mid_5_8_staging_a;
    PresetLocation mid_5_8_staging_fromNorth;
    PresetLocation mid_5_8_staging_fromSouth;

    //////////// Row 4
    PresetLocation shelf11_a;

    PresetLocation shelf8_fromSouth_near;
    PresetLocation shelf8_fromSouth_far;
    PresetLocation shelf11_fromNorth_near;
    PresetLocation shelf11_fromNorth_far;

    PresetLocation mid_8_11_intersection_fromNorth;
    PresetLocation mid_8_11_intersection_fromSouth;

    PresetLocation mid_8_11_staging_a;

    PresetLocation mid_8_11_staging_fromNorth;
    PresetLocation mid_8_11_staging_fromSouth;

    //////////// Row 5
    PresetLocation shelf11_fromSouth_near;
    PresetLocation shelf11_fromSouth_far;

    //////////// Row 6
    PresetLocation southwest_corner_staging;

    PresetLocation waitpoint_best_south_fromNorth;
    PresetLocation waitpoint_best_south_fromSouth;

    PresetLocation agv2_a;

    ///////////// Miscellaneous (all rows)
    PresetLocation waitpoint_best_south_fromSouth_near;
    PresetLocation mid_8_11_intersection_fromSouth_near;

    PresetLocation waitpoint_best_north_fromNorth_near;
    PresetLocation mid_5_8_intersection_fromNorth_near;

    //////////// Shelves 1 and 2
    // shelf1_fromSouth_far, shelf1_fromSouth_near, shelf2_fromNorth_near, shelf2_fromNorth_far
    PresetLocation shelf1_fromSouth_far;
    PresetLocation shelf1_fromSouth_near;
    PresetLocation shelf2_fromNorth_near;
    PresetLocation shelf2_fromNorth_far;


    //////////// End Shelf Preset Locations

    std::vector<PresetLocation> preset_locations_list_; // lookup list to compare distances with
    std::vector<PresetLocation> preset_locations_list_simple_; // lookup list to compare distances with
    std::vector<std::string> wait_preset_locations_list_;
    std::vector<PresetLocation> inner_fast_preset_locations_list_;
    std::vector<PresetLocation> shelf_1_or_2_preset_locations_list_;

    /* ===================== Conveyor Variables ===================== */
    const float dx_ = 6.6-0.01;
    const int buffer_parts_{2};
    int buffer_parts_collected{0};
    bool waiting_for_part_{false};
    std::string conveyor_type_{""};
    ros::Time current_part_load_time_;
    CameraListener::ModelInfo current_part_on_conveyor_;
    std::deque<CameraListener::ModelInfo> parts_on_conveyor_; 

    /* ===================== Misc Variables ===================== */
    int prev_num_orders_{0};

    std::map<int, PresetLocation> cam_to_presetlocation;
    std::map<int, std::string> cam_to_shelf_string;
    std::map<int, double> cam_to_y_coordinate;
    std::map<std::string, int> agv_to_camera;
    bool competition_started_{false};
    bool region_dict_defined_{false};
    

public:
    /**
     * \brief :A Constructor for the RWAImplentation class
     * \param :references of node, camera listener, gantry controller, and agv control classes
     */
    RWAImplementation(ros::NodeHandle& node, CameraListener& camera_listener, GantryControl& gantry, Competition& competition, AGVControl& agv_control) :
        node_{&node}, cam_listener_{&camera_listener}, gantry_{&gantry}, competition_{&competition}, agv_control_{&agv_control}
    {
        initPresetLocs();
        lane_handler.setNode(node);

        breakbeam_sub_ = node_->subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, cam_listener_);
        
        ros::Duration(5).sleep();
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
    bool buildKit();

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

    /**
     * \brief: check if the part needs to be flipped and flip if required
     * \param: part in the tray to be check for flipping
     * \param: product in the tray to be checked for flipping
     * \result: true/false
     */
    bool checkForFlip(part &part_in_tray, Product product);

    /**
     * \brief: Method to detect gaps in shelf locations
     * \result: true if detected, false otherwise
     */
    bool detectGaps();

    /**
     * \brief: Method to build regionDictionary, which maps regions which indicate shelf and upper/lower, to a preset location string and wait/nowait string.
     * \result: regionDictionary is defined
     */
    void InitRegionDictionaryDependingOnSituation();

   // regionDictionary, key is string indicating shelf and upper/lower, value is vector of strings, ie. ["shelf5_fromsouth_near", "nowait"]
    std::unordered_map<std::string, std::vector<std::string> > regionDictionary;

    void pickPartsFromAGV(std::string agv_id);

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

    std::vector<Product> checkLater;

    std::unordered_map<std::vector<std::string>, std::vector<PresetLocation>, container_hash<std::vector<std::string>> > PathingLookupDictionary;
    std::unordered_map<std::vector<std::string>, std::vector<PresetLocation>, container_hash<std::vector<std::string>> > WaitPathingLookupDictionary;

    double calcDistanceInXYPlane(geometry_msgs::Pose a, geometry_msgs::Pose b);
    double calcDistanceInXYTorso(PresetLocation pLocation, std::vector<double> joint_positions);
    double calcDistanceInXYTorso_Accurate(PresetLocation pLocation, std::vector<double> joint_positions);
    PresetLocation getNearesetPresetLocation();
    PresetLocation getNearesetPresetLocation_Simple();
    PresetLocation getNearesetPresetLocation_Specific(std::vector<PresetLocation> preset_vector_to_lookup_in);
    std::vector<PresetLocation> getPresetLocationVector(PresetLocation target_preset_location);
    std::vector<PresetLocation> getPresetLocationVector_Simple(PresetLocation target_preset_location);
    std::vector<PresetLocation> getPresetLocationVectorWithWait(PresetLocation target_preset_location);
    std::vector<PresetLocation> getPresetLocationVectorUsingString(std::string target_preset_location_string, std::string wait_string);
    std::vector<PresetLocation> getPresetLocationVectorUsingStringNoWait(std::string target_preset_location_string, std::string wait_string);
    std::vector<PresetLocation> getPresetLocationVectorUsingString_Specific(std::string target_preset_location_string, std::string wait_string, std::vector<PresetLocation> pset_location_vector_to_use);
    bool executeVectorOfPresetLocations( std::vector<PresetLocation> path_to_execute );
    bool executeVectorOfPresetLocations_Fast( std::vector<PresetLocation> path_to_execute );
    bool executeVectorOfPresetLocationsWithWait( std::vector<PresetLocation> path_to_execute );
    geometry_msgs::Pose gantryXY2worldposeXY(PresetLocation preset_location_2_convert);
    // preset locations from start to safe location for three shelf rows starting from agv1 side
    std::array<std::vector<PresetLocation>,3> shelf_preset_locations;
    std::array<std::string, 3> gaps; //either "gap_conveyor" or "gap_end"

    PresetLocation getNearestBinPresetLocation();

    void rankEmptyBins();

    /**
     * \brief: simple drop functionlity
     * \result: true
     */
    bool simpleDropPart() {
        gantry_->deactivateGripper("left_arm");
        return true;
    }

    /**
     * \brief: Method to detect the current part belongs to which shelf
     * \param: part to be located
     * \param: camera index that identified the part
     * \result: string with information about part location
     */
    std::string isPartInUpperOrLowerRegionOfWhichShelf(part my_part, int discovered_cam_idx) {
        std::string shelf_string = cam_to_shelf_string[discovered_cam_idx]; // ie. "shelf5"

        ROS_INFO_STREAM("shelf_string is: " << shelf_string);

        double product_y_coord = my_part.pose.position.y;
        double camera_y_coord = cam_to_y_coordinate[discovered_cam_idx];

        ROS_INFO_STREAM("product_y_coord === " << product_y_coord << " and camera_y_coord === " << camera_y_coord);

        std::string upper_or_lower_string = "upper"; // initialize as upper
        if (product_y_coord >= camera_y_coord) {
            ROS_INFO_STREAM("Part is Above (upper) to camera");

            upper_or_lower_string = "upper";
        }
        else {
            upper_or_lower_string = "lower";
            ROS_INFO_STREAM("Part is Below (lower) to camera");
        }
        ROS_INFO_STREAM("Output isPartInUpperOrLowerRegionOfWhichShelf String === " << shelf_string + upper_or_lower_string);
        return shelf_string + upper_or_lower_string; // ie. "shelf5" + "upper" = "shelf5upper"
    }

    /* ===================== Human Obstacle Variables ===================== */
    AllLanesHandler lane_handler;
};
