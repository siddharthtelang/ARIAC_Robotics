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

#include <unordered_map>


class RWAImplementation
{
private:
    ros::NodeHandle* node_;

    /* ===================== Gazebo Control ===================== */
    CameraListener* cam_listener_;
    GantryControl* gantry_;
    Competition* competition_;
    std::stack<std::queue<std::vector<Product>>> task_queue_;

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
    const float dx_ = 3+4.492549-1;
    bool waiting_for_part_{false};
    ros::Time current_part_load_time_;
    CameraListener::ModelInfo current_part_on_conveyor_;
    std::deque<CameraListener::ModelInfo> parts_on_conveyor_; 

    /* ===================== Misc Variables ===================== */
    int prev_num_orders_{0};

    std::map<int, PresetLocation> cam_to_presetlocation;

public:
    RWAImplementation(ros::NodeHandle& node, CameraListener& camera_listener, GantryControl& gantry, Competition& competition) : 
        node_{&node}, cam_listener_{&camera_listener}, gantry_{&gantry}, competition_{&competition}
    {
        initPresetLocs();
        breakbeam_sub_ = node_->subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, cam_listener_);
    };

    bool checkConveyor(bool part_wanted);
    void initPresetLocs();
    void processOrder();
    void buildKit();
    void checkAgvErrors();







    double calcDistanceInXYPlane(geometry_msgs::Pose a, geometry_msgs::Pose b) {
        return std::sqrt( std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2) ); // euclidean distance
    }




    struct distance_and_PresetLocation_struct {
        double distance;
        PresetLocation candidate_location;
    };

    //////////////////////////////// Get nearest preset location to current gantry position
    PresetLocation getNearesetPresetLocation() {
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

    // std::unordered_map<std::string, double> asdfasdf;


    template <typename Container> // we can make this generic for any container [1]
    struct container_hash {
        std::size_t operator()(Container const& c) const {
            return boost::hash_range(c.begin(), c.end());
        }
    };

    // // Dictionary, Key is (current_location, target_location), Value is path to get there.
    // std::unordered_map<double, double> PathingLookupDictionary;
    // std::unordered_map<std::vector<double>, double,container_hash<std::vector<double>> > PathingLookupDictionary;
    // std::unordered_map<std::vector<double>, std::vector<double>> PathingLookupDictionary;
    std::unordered_map<std::vector<std::string>, std::vector<PresetLocation>, container_hash<std::vector<std::string>> > PathingLookupDictionary;
    // std::unordered_map<std::vector<std::string>, std::vector<PresetLocation>, container_hash<std::vector<std::string>> > PathingLookupDictionary = {
    //     { {"start_a", "bin3_a"} , std::vector<PresetLocation>{start_a, bin3_a} },
    //     { {"start_a", "agv2_a"} , std::vector<PresetLocation>{start_a, agv2_a} },
    //     { {"start_a", "agv1_staging_a"} , std::vector<PresetLocation>{start_a, agv1_staging_a} },
    //     { {"start_a", "bottom_left_staging_a"} , std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a} },
    //     { {"start_a", "shelf8_a"} , std::vector<PresetLocation>{start_a, mid_5_8_staging_a, shelf8_a} },
    //     { {"start_a", "shelf11_a"} , std::vector<PresetLocation>{start_a, mid_8_11_staging_a, shelf11_a} },
    //     { {"start_a", "bin11_a"} , std::vector<PresetLocation>{start_a, bin11_a} },

    //     { {"bin3_a", "start_a"} , std::vector<PresetLocation>{bin3_a, start_a} },
    //     { {"agv2_a", "start_a"} , std::vector<PresetLocation>{agv2_a, start_a} },
    //     { {"agv1_staging_a", "start_a"} , std::vector<PresetLocation>{agv1_staging_a, start_a} },
    //     { {"bottom_left_staging_a", "start_a"} , std::vector<PresetLocation>{bottom_left_staging_a, agv1_staging_a, start_a} },
    //     { {"shelf8_a", "start_a"} , std::vector<PresetLocation>{shelf8_a, mid_5_8_staging_a, start_a} },
    //     { {"shelf11_a", "start_a"} , std::vector<PresetLocation>{shelf11_a, mid_8_11_staging_a, start_a} },
    //     { {"bin11_a", "start_a"} , std::vector<PresetLocation>{bin11_a, start_a} },

    // };

    // std::unordered_map<std::vector<double>, std::vector<double>> PathingLookupDictionary = {
    //     { {10.0, 11.3} , {11.2, 11.44, 11.556} }

    // };



    std::vector<PresetLocation> getPresetLocationVector(PresetLocation target_preset_location) {
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


    bool executeVectorOfPresetLocations( std::vector<PresetLocation> path_to_execute ) {
        for (PresetLocation psetlocation : path_to_execute) {
            gantry_->goToPresetLocation(psetlocation);
        }
        return true;
    }



    // converts PresetLocation to a Pose, but only look at Pose's x and y, other numbers are meaningless
    geometry_msgs::Pose gantryXY2worldposeXY(PresetLocation preset_location_2_convert) {
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

};


