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
                ROS_INFO_STREAM("========");
                ROS_INFO_STREAM("Passed sensor " << breakbeam);
                ROS_INFO_STREAM("========");
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
        std::array<int, 4> lane_w_obstacle_dirs{};
        for (int i{0}; i < lanes_.size(); i++) {
                lane_w_obstacle_dirs[i] = lanes_[i].queryPair();
        }
        return lane_w_obstacle_dirs;
}