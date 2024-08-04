#include "map_changer/map_changer_node.h"

namespace map_changer {

map_changer_node::map_changer_node() : pnh_("~")
{
    ROS_INFO("Start map_changer_node");
    read_yaml();
    wp_sub_ = nh_.subscribe("/waypoint_manager/waypoint", 1, &map_changer_node::cb_wp, this);
    map_srv_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map");
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, this);
}

map_changer_node::~map_changer_node()
{
}

void map_changer_node::cb_wp(const waypoint_manager_msgs::Waypoint::ConstPtr &msg)
{
    static int index = 0;
    static std::string old_id;
    if (config_list.size() - 1 >= index)
    {
        std::string id = config_list[index][0];
        if (msg->identity == id && msg->identity != old_id)
        {
            send_map(index);
            send_pose(msg->pose);
            index++;
        }
        // if (msg->identity != id && id == old_id)
        // {
        //     send_map(index);
        //     send_pose(msg->pose);
        //     index++;
        // }
    }
    old_id = msg->identity;
}

void map_changer_node::read_yaml()
{
    pnh_.param("file_path", file_path_, std::string(ros::package::getPath("map_changer") += "/config/test1.yaml"));
    ROS_INFO("Load %s", file_path_.c_str());
    YAML::Node config = YAML::LoadFile(file_path_);
    for (const auto& node:config["config"])
    {
        std::array<std::string, 2> tmp;
        tmp[0] = node["waypoint_id"].as<std::string>();
        tmp[1] = node["map_file"].as<std::string>();
        config_list.push_back(tmp);
    }
}

void map_changer_node::send_map(int index)
{
    nav_msgs::LoadMap srv;
    srv.request.map_url = config_list[index][1];
    if (map_srv_.call(srv))
    {
        ROS_INFO("Map change to %s", srv.request.map_url.c_str());
    } else {
        ROS_ERROR("Failed to call service /change_map");
    }
}

void map_changer_node::send_pose(geometry_msgs::Pose pose)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose = pose;
    pose_pub_.publish(msg);
    ROS_INFO("Teleport to x:%lf, y:%lf", msg.pose.pose.position.x, msg.pose.pose.position.y);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_changer_node");
    map_changer::map_changer_node mc;
    ros::spin();
    return 0;
}