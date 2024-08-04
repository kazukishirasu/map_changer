#ifndef MAP_CHANGER_NODE_H__
#define MAP_CHANGER_NODE_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <nav_msgs/LoadMap.h>

namespace map_changer {

class map_changer_node
{
public:
    map_changer_node();
    ~map_changer_node();
    void cb_wp(const waypoint_manager_msgs::Waypoint::ConstPtr &msg);
    void read_yaml();
    void send_map(int);
    void send_pose(geometry_msgs::Pose);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber wp_sub_;
    ros::ServiceClient map_srv_;
    ros::ServiceClient costmap_srv_;
    ros::Publisher pose_pub_;
    std::string file_path_;
    std::vector<std::array<std::string, 2>> config_list;
};

}
#endif