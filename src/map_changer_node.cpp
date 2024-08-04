#include "map_changer/map_changer_node.h"

namespace map_changer {

map_changer_node::map_changer_node() : pnh_("~")
{
    ROS_INFO("Start map_changer_node");
    read_yaml();
    wp_sub_ = nh_.subscribe("/waypoint_manager/waypoint", 1, &map_changer_node::cb_wp, this);
    map_srv_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map");
    costmap_srv_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map_for_costmap");
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
        // if (msg->identity == id && msg->identity != old_id)
        // {
        //     send_map(index);
        //     send_pose(msg->pose);
        //     index++;
        // }
        if (msg->identity != id && id == old_id)
        {
            send_map(index);
            send_pose(msg->pose);
            index++;
        }
    } else if (config_list.size() - 1 < index)
    {
        index = 0;
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
    nav_msgs::LoadMap map, costmap;
    map.request.map_url = config_list[index][1] + ".yaml";
    costmap.request.map_url = config_list[index][1] + "_for_costmap.yaml";
    if (map_srv_.call(map))
    {
        ROS_INFO("Map change to %s", map.request.map_url.c_str());
    } else {
        ROS_ERROR("Failed to call service /change_map");
    }
    if (costmap_srv_.call(costmap))
    {
        ROS_INFO("Costmap change to %s", costmap.request.map_url.c_str());
    } else {
        ROS_ERROR("Failed to call service /change_map_for_costmap");
    }
}

void map_changer_node::send_pose(geometry_msgs::Pose pose)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose = pose;
    pose_pub_.publish(msg);

    double roll = 0, pitch = 0, yaw = 0;
    tf::Quaternion quat;
	quaternionMsgToTF(pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    nh_.setParam("/emcl2_node/initial_pose_x", pose.position.x);
    nh_.setParam("/emcl2_node/initial_pose_y", pose.position.y);
    nh_.setParam("/emcl2_node/initial_pose_y", yaw);

    ROS_INFO("Teleport to x:%lf, y:%lf", pose.position.x, pose.position.y);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_changer_node");
    map_changer::map_changer_node mc;
    ros::spin();
    return 0;
}