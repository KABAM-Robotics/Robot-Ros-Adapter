#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <string>

// Abstract base class for ROS1 robot nodes
class RobotBase {
public:
    explicit RobotBase(std::string node_name = "robot_node")
    : node_name_(node_name)
    {
        ROS_INFO_STREAM(node_name_ << " initialized.");

        // Initialize all default interfaces
        setupGoalInterfaces();
        setupTeleopInterfaces();
        setupBatteryInterfaces();
        setupMapInterfaces();
        setupAppStateInterfaces();
    }

    virtual ~RobotBase() {
        ROS_INFO_STREAM(node_name_ << " destroyed.");
    }

protected:
    ros::NodeHandle nh_;
    std::string node_name_;

    // ========================
    // Goal interfaces
    // ========================
    ros::Subscriber goal_sub_;
    ros::Subscriber cancel_goal_sub_;
    ros::Publisher navigation_status_pub_;
    std::string goal_sub_topic_ = "nav/goal";
    std::string cancel_goal_sub_topic_ = "nav/cancel";
    std::string navigation_status_pub_topic_ = "nav/status";

    virtual void setupGoalInterfaces() {
        goal_sub_ = nh_.subscribe<geometry_msgs::Pose>(
            "/" + node_name_ + "/" + goal_sub_topic_, 10,
            &RobotBase::onGoalReceived, this);

        cancel_goal_sub_ = nh_.subscribe<std_msgs::String>(
            "/" + node_name_ + "/" + cancel_goal_sub_topic_, 10,
            &RobotBase::onCancelGoalReceived, this);

        navigation_status_pub_ = nh_.advertise<std_msgs::String>(
            "/" + node_name_ + "/" + navigation_status_pub_topic_, 10);
    }

    // Default callbacks — can be overridden in derived classes
    virtual void onGoalReceived(const geometry_msgs::Pose::ConstPtr& msg) {
        ROS_INFO_STREAM("Received goal pose (default callback)");
    }

    virtual void onCancelGoalReceived(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO_STREAM("Received cancel goal (default callback)");
    }

    // ========================
    // Teleoperation interfaces
    // ========================
    ros::Subscriber teleop_cmd_sub_;
    std::string teleop_cmd_sub_topic_ = "teleop/cmd";

    virtual void setupTeleopInterfaces() {
        teleop_cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>(
            "/" + node_name_ + "/" + teleop_cmd_sub_topic_, 10,
            &RobotBase::onTeleopCmdReceived, this);
    }

    virtual void onTeleopCmdReceived(const geometry_msgs::Twist::ConstPtr& msg) {
        ROS_INFO_STREAM("Received teleop command (default callback)");
    }

    // ========================
    // Battery interfaces
    // ========================
    ros::Publisher battery_pub_;
    std::string battery_pub_topic_ = "battery/percentage";

    virtual void setupBatteryInterfaces() {
        battery_pub_ = nh_.advertise<std_msgs::Int8>("/" + node_name_ + "/" + battery_pub_topic_, 10);
    }

    // ========================
    // Map interfaces
    // ========================
    ros::Publisher map_list_pub_;
    ros::Publisher active_map_pub_;
    ros::Publisher waypoint_list_pub_;
    std::string map_list_pub_topic_ = "map/list";
    std::string active_map_pub_topic_ = "map/current";
    std::string waypoint_list_pub_topic_ = "map/waypoints";

    virtual void setupMapInterfaces() {
        map_list_pub_ = nh_.advertise<std_msgs::String>("/" + node_name_ + "/" +map_list_pub_topic_, 10);
        active_map_pub_ = nh_.advertise<std_msgs::String>("/" + node_name_ + "/" +active_map_pub_topic_, 10);
        waypoint_list_pub_ = nh_.advertise<std_msgs::String>("/" + node_name_ + "/" +waypoint_list_pub_topic_, 10);
    }

    // ========================
    // User confirmation interfaces
    // ========================
    ros::Publisher app_state_pub_;
    ros::Subscriber app_state_sub_;
    std::string app_state_pub_topic_ = "app_state";
    std::string app_state_sub_topic_ = "app_state_cmd";

    virtual void setupAppStateInterfaces() {
        app_state_pub_ = nh_.advertise<std_msgs::String>("/" + node_name_ + "/" +app_state_pub_topic_, 10);
        app_state_sub_ = nh_.subscribe<std_msgs::String>(
            "/" + node_name_ + "/" + app_state_sub_topic_, 10,
            &RobotBase::onAppStateReceived, this);
    }

    virtual void onAppStateReceived(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO_STREAM("Received app state command (default callback)");
    }
};
