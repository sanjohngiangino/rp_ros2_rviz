#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"
#include "rp_ros2_rviz/robot.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

class PathPlanner : public rclcpp::Node
{

public:
    explicit PathPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:


    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void robotCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void makePath(const std::shared_ptr<DMap>& dmap, float resolution, float expansion_range, const Eigen::Vector2f& goal_position);
    Vector2iVector getObstaclesFromGrid(const nav_msgs::msg::OccupancyGrid& grid);
    void publishBool();


    

    std_msgs::msg::Bool bool_msg;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub_path;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string image_path;
    cv::Mat custom_image;
    DMapPlanner planner;

    Eigen::Vector2f latest_robot_position = Eigen::Vector2f::Zero();
    Eigen::Vector2f latest_goal_position = Eigen::Vector2f(0, 0);
    bool has_goal_ = false;

    bool has_robot_position_ = false;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position_sub_;

    std::list<Vector2f> path;
    bool use_gradient = false;
    bool dmap_ready_ = false;
    bool path_generated_ = false;
    bool is_moving_ = false;
    std::shared_ptr<DMap>  dmap;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
};