#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"
#include "rp_ros2_rviz/robot.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"  
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std;
using namespace cv;

class WorldNode : public rclcpp::Node {

private:

    const float ROBOT_RADIUS = 5.0f;
    const float LASER_CONE_DEG = 270.0f;
    const float LASER_MAX_RANGE = 150.0f;
    const float LASER_ANGLE_STEP_DEG = 1.0f;
    const float ROBOT_MOVE_STEP = 2.0f;
    const float ROBOT_ROTATION_STEP = 0.15f;
    const int TIMER_PERIOD_MS = 100;
    const int OCCUPIED_THRESHOLD = 50;
    const int BLACK_THRESHOLD = 127;
    const bool USE_GRADIENT = false;
    const int BACKGROUND_MODE = 0;

    DMapPlanner planner;
    std::list<Vector2f> path;
    cv::Mat background_image;
    cv::Mat shown_image;

    std::unique_ptr<Robot> my_robot;   

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_publisher_;  
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controller_stop_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr particle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr step_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_; 
    geometry_msgs::msg::PoseArray::SharedPtr particles_;

    bool continue_publishing_= true;
    bool is_following_path_=false;
    bool update_goal_=false;
    bool first_position_sent_=false;    
    void publishSimulatedOdomAndScan();
    void stepPoseCallback(const geometry_msgs::msg::Pose::SharedPtr pose);
    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void followPath(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void simulateLaserConeAndPublishScan(const cv::Mat& background, const Eigen::Vector2f& robot_pos,float theta_rad,
        float cone_deg, float max_range, float angle_step_deg);
    void drawSimulatedLaserOnImage(const cv::Mat& background, cv::Mat& shown_image, const Eigen::Vector2f& robot_pos,
        float theta_rad, float cone_deg, float max_range, float angle_step_deg);
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publishRobot();


public:
    explicit WorldNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void redisplay();
    static void onMouse(int event, int x, int y, int, void* arg);
    void handleKeyboard(int key);

};
