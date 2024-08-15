#include "map_changer/map_changer_node.h"

namespace map_changer {

map_changer_node::map_changer_node() : pnh_("~")
{
    ROS_INFO("Start map_changer_node");
    pnh_.param("wait_time", wait_time_, 1.0);
    read_yaml();
    wp_sub_ = nh_.subscribe("/waypoint_manager/waypoint", 1, &map_changer_node::cb_wp, this);
    result_sub_ = nh_.subscribe("/move_base/result", 1, &map_changer_node::cb_result, this);
    next_wp_srv_ = nh_.serviceClient<std_srvs::Trigger>("/waypoint_manager/waypoint_server/next_waypoint");
    map_srv_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map");
    costmap_srv_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map_for_costmap");
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, this);
}

map_changer_node::~map_changer_node()
{
}

void map_changer_node::cb_wp(const waypoint_manager_msgs::Waypoint::ConstPtr &msg)
{
    wp_ = msg;
    static waypoint_manager_msgs::Waypoint old_wp;
    if (*msg != old_wp)
    {
        new_goal_ = true;
    }else
    {
        new_goal_ = false;
    }
    old_wp = *msg;
}

void map_changer_node::cb_result(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
    if (!wp_)
    {
        return;
    }

    static std::string old_id;
    if (msg->status.status == 3)
    {
        for (auto itr = config_list_.begin(); itr != config_list_.end(); ++itr)
        {
            std::array<std::string, 2>& config = *itr;
            if (config[0] == wp_->identity && config[0] == old_id)
            {
                // ros::Duration(wait_time_).sleep();
                call_next_wp();
                reach_goal_ = true;
                index_ = std::distance(config_list_.begin(), itr);
            }
        }
    }
    old_id = wp_->identity;
}

void map_changer_node::loop()
{
    if (!wp_)
    {
        return;
    }

    if (reach_goal_ && new_goal_)
    {
        send_emclpose(wp_->pose);
        send_map(index_);
        send_initialpose(wp_->pose);
        reach_goal_ = false;
        new_goal_ = false;
    }
}

void map_changer_node::read_yaml()
{
    try
    {
        pnh_.param("file_path", file_path_, std::string(ros::package::getPath("map_changer") += "/config/test1.yaml"));
        ROS_INFO("Load %s", file_path_.c_str());
        YAML::Node config = YAML::LoadFile(file_path_);
        for (const auto& node:config["config"])
        {
            std::array<std::string, 2> tmp;
            tmp[0] = node["waypoint_id"].as<std::string>();
            tmp[1] = node["map_file"].as<std::string>();
            config_list_.push_back(tmp);
        }
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
}

void map_changer_node::call_next_wp()
{
    std_srvs::Trigger srv;
    try
    {
        next_wp_srv_.call(srv);
        ROS_INFO("Call service : /next_waypoint");
    }
    catch(const ros::Exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
}

void map_changer_node::send_map(int index)
{
    nav_msgs::LoadMap map, costmap;
    map.request.map_url = config_list_[index][1] + ".yaml";
    costmap.request.map_url = config_list_[index][1] + "_for_costmap.yaml";
    try
    {
        map_srv_.call(map);
        ROS_INFO("Map change to %s", map.request.map_url.c_str());
    }
    catch(const ros::Exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }

    try
    {
        costmap_srv_.call(costmap);
        ROS_INFO("Costmap change to %s", costmap.request.map_url.c_str());
    }
    catch(const ros::Exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
}

void map_changer_node::send_emclpose(geometry_msgs::Pose pose)
{
    double roll = 0, pitch = 0, yaw = 0;
    tf::Quaternion quat;
    quaternionMsgToTF(pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    nh_.setParam("/emcl2_node/initial_pose_x", pose.position.x);
    nh_.setParam("/emcl2_node/initial_pose_y", pose.position.y);
    nh_.setParam("/emcl2_node/initial_pose_y", yaw);
}

void map_changer_node::send_initialpose(geometry_msgs::Pose pose)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose = pose;
    pose_pub_.publish(msg);
    ROS_INFO("Teleport to x:%lf, y:%lf", pose.position.x, pose.position.y);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_changer_node");
    map_changer::map_changer_node mc;
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        mc.loop();
        rate.sleep();
    }
    return 0;
}