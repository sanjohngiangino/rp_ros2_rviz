#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

class MapServer : public rclcpp::Node {
public:
    explicit MapServer(const rclcpp::NodeOptions& options);

private:
    std::string map_path_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_msg_;

    bool loadMap(const std::string& image_path);
    void publishMap();
};
