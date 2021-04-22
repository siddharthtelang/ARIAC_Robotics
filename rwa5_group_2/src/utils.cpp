#include "utils.h"

std::unordered_map<std::string, double> model_height = {
    {"piston_rod_part_red", 0.0065}, // modified because it sinks into the surface a bit
    {"piston_rod_part_green", 0.0065},
    {"piston_rod_part_blue", 0.0065},
    {"pulley_part_red", 0.07},
    {"pulley_part_green", 0.07},
    {"pulley_part_blue", 0.07},
    {"gear_part_red", 0.012},
    {"gear_part_green", 0.012},
    {"gear_part_blue", 0.012},
    {"gasket_part_red", 0.02},
    {"gasket_part_green", 0.02},
    {"gasket_part_blue", 0.02},
    {"disk_part_red", 0.023},
    {"disk_part_green", 0.023},
    {"disk_part_blue", 0.023}
};

void LaneBreakbeamPair::breakbeam_callback(const nist_gear::Proximity::ConstPtr &msg, const int breakbeam)
{
        if ((msg->object_detected == true) && (ros::Time::now().toSec() - time_hit_[breakbeam] > breakbeam_rate_)) 
        {
                if (breakbeam == 0)
                {
                        if (prev_hit_ == 1) towards_conveyor_ = 1;
                        else towards_conveyor_ = 0;
                } else 
                {
                        if (prev_hit_ == 0) towards_conveyor_ = 0;
                        else towards_conveyor_ = 1;
                }
                prev_hit_ = breakbeam;
                time_hit_[breakbeam] = ros::Time::now().toSec();
        }
}

std::array<int, 4> AllLanesHandler::queryLanes() {
        std::array<int, 4> lane_dirs{};
        for (int i{0}; i < lanes_.size(); i++) {
                lane_dirs[i] = lanes_[i].queryPair();
        }
        return lane_dirs;
}

std::array<bool, 5> AllLanesHandler::clearLanes() {
        std::array<bool, 5> clear_lanes{false};
        auto lane_dirs = queryLanes();
        int count{0};
        for (int i{0}; i < 4; i++) {
                dir = lane_dirs[i];
                if (dir == -1) {
                        clear_lanes[i] = true;
                        count++;
                }
        }
        if (count == 2) clear_lanes[4] = true;
        return clear_lanes;
}

bool AllLanesHandler::waitForLane(int lane) {
        auto dir = lanes_[lane].queryPair();
        while(dir == 0 || lanes_[lane].prev_hit_ == 1) {
                ros::Duration(0.1).sleep();
                dir = lanes_[lane].queryPair();
        }
        return true;
}