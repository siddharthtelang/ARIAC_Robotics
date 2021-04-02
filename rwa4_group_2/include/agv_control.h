#ifndef AGVCONTROL_H
#define AGVCONTROL_H

#include <string>
#include <std_msgs/String.h>
#include <nist_gear/AGVControl.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "utils.h"


/**
 * \brief: Defines class for AGV control
 * \param: reference to ROS Node Handle
 * \result: object
 */
class AGVControl {

public:
    /**
    * \brief: Prepares AGV controls
    * \param: reference to ROS Node Handle
    * \result: Ensured existence of AGV and ability to communicate
    */
    explicit AGVControl(ros::NodeHandle &);

    /**
    * \brief: Checks if specific AGV is ready
    * \param: Tray name
    * \result: boolean
    */
    bool isAGVReady(std::string tray_name);
    
    /**
     * @brief Submit an AGV shipment
     * 
     * @param shipment_type Shipment type which should match the order
     * @param kit_tray  Kit tray used to build the kit
     * @return true AGV successfully submitted 
     * @return false AGV not successfully submitted 
     */
    bool sendAGV(std::string shipment_type, std::string kit_tray);
    /**
     * @brief State for AGV1
     * 
     * @param msg 
     */
    void agv1_state_callback(const std_msgs::String & msg);
    /**
     * @brief State for AGV2
     * 
     * @param msg 
     */
    void agv2_state_callback(const std_msgs::String & msg);
private:
    ros::ServiceClient agv1_client; 
    ros::ServiceClient agv2_client;
    ros::Subscriber agv1_state_subscriber_;
    ros::Subscriber agv2_state_subscriber_;
    bool agv1_ready;
    bool agv2_ready;
};


#endif //AGV_CONTROL_H