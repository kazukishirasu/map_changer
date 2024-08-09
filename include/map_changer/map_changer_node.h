#ifndef MAP_CHANGER_NODE_H__
#define MAP_CHANGER_NODE_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/LoadMap.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

namespace map_changer {

class map_changer_node
{
public:
    map_changer_node();
    ~map_changer_node();
    void cb_wp(const waypoint_manager_msgs::Waypoint::ConstPtr &msg);
    void cb_result(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);
    void call_next_wp();
    void read_yaml();
    void send_map(int);
    void send_emclpose(geometry_msgs::Pose);
    void send_initialpose(geometry_msgs::Pose);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber wp_sub_;
    ros::Subscriber result_sub_;
    ros::ServiceClient next_wp_srv_;
    ros::ServiceClient map_srv_;
    ros::ServiceClient costmap_srv_;
    ros::Publisher pose_pub_;
    std::string file_path_;
    std::vector<std::array<std::string, 2>> config_list_;
    bool reach_flag_ = false;
    double wait_time_;
};

}
#endif