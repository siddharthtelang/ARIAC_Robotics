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
    AGVControl* agv_control_;
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
    RWAImplementation(ros::NodeHandle& node, CameraListener& camera_listener, GantryControl& gantry, Competition& competition, AGVControl& agv_control) : 
        node_{&node}, cam_listener_{&camera_listener}, gantry_{&gantry}, competition_{&competition}, agv_control_{&agv_control}
    {
        initPresetLocs();
        breakbeam_sub_ = node_->subscribe<nist_gear::Proximity>("/ariac/breakbeam_0_change", 10, &CameraListener::breakbeam_callback, cam_listener_);
    };

    bool checkConveyor(bool part_wanted);
    void initPresetLocs();
    void processOrder();
    void buildKit();
    void checkAgvErrors();


    struct distance_and_PresetLocation_struct {
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

    PresetLocation getNearestBinPresetLocation()
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
        for (auto cam_part_array : cam_parts) {
            for (auto model_info : cam_part_array) {
                flat_parts_list.push_back(model_info);

            }
        }

        std::vector<std::vector<double>> ValidPointsList = TwoPointsList;

        // //https://stackoverflow.com/questions/7631996/remove-an-element-from-a-vector-by-value-c
        // for( std::vector<std::vector<double>>::iterator iter = ValidPointsList.begin(); iter != ValidPointsList.end(); ++iter ) {
        //     for (auto model_info : flat_parts_list) {
        //             // if (model_info.world_pose.position.x < *iter[0][0] || model_info.world_pose.position.x > iter[0]
        //             //     || model_info.world_pose.position.y < iter[2] && model_info.world_pose.position.y > iter[3]) {
        //             //     // ValidPointsList.push_back(bin_points);
        //             // }
        //             if( *iter == VALUE ) {
        //                 ValidPointsList.erase( iter );
        //                 break;
        //             }
        //     }
        // }


        // curFiles is: vector < string > curFiles;

        // std::vector<std::vector<double>>::iterator it = ValidPointsList.begin();

        // while(it != ValidPointsList.end()) {
        //     for (auto model_info : flat_parts_list) {

        //         if (model_info.world_pose.position.x < it[0][0] || model_info.world_pose.position.x > it[0][1]|| model_info.world_pose.position.y < it[0][2] || model_info.world_pose.position.y > it[0][3]) {

        //             it = ValidPointsList.erase(it);
        //         }
        //         else ++it;
        //     }
        // }


        // bool need_to_remove = false;
        // for (auto bin_points : ValidPointsList) {
        //     need_to_remove = false;

        //     for (auto model_info : flat_parts_list) {
        //         if (model_info.world_pose.position.x < bin_points[0] || model_info.world_pose.position.x > bin_points[1]|| model_info.world_pose.position.y < bin_points[2] || model_info.world_pose.position.y > bin_points[3]) {
        //             need_to_remove = true;
        //         }
        //     }

        //     if (need_to_remove == true) {
        //         // Uses the find() function to return a iterator pointing to the game we want to delete.
        //         std::vector<std::vector<double>>::iterator result = find(ValidPointsList.begin(), ValidPointsList.end(), bin_points);

        //         // If result points to game_list.end() it couldn't find the game so we can't delete it. If it isn't it delete the game.
        //         if (result == ValidPointsList.end())
        //             std::cout << "That vector is not in there!" << std::endl;
        //         else
        //             ValidPointsList.erase(result);
        //     }
        // }

        PresetLocation dropPresetLocation;
        // dropPresetLocation.gantry = {ValidPointsList[0][0] + 0.3, (ValidPointsList[0][2])*-1.0 , 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
        // dropPresetLocation.gantry = {ValidPointsList[0][0] + 0.0, (ValidPointsList[0][2])*-1.0 , 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
        dropPresetLocation.gantry = {2.3 - 0.3, (1.842058)*-1.0 , 0.0}; // xlow+.3, (ylow+.3)*-1, no spin
        dropPresetLocation.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
        dropPresetLocation.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};



        // ROS_INFO_STREAM( "target Xcoord == " << ValidPointsList[0][0] + 0.0 << " target Ycoord == " << (ValidPointsList[0][2])*-1.0);
        ROS_INFO_STREAM( "target [xlow,xhigh,ylow,yhigh]] == " << ValidPointsList[0][0] << " " << ValidPointsList[0][1] << " " << ValidPointsList[0][2] << " " << ValidPointsList[0][3] );


        return dropPresetLocation;
    };

    bool simpleDropPart() {
        gantry_->deactivateGripper("left_arm");
        return true;
    }
};


