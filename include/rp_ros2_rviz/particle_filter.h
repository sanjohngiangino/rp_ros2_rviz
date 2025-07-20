#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>     
#include <cmath>  
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tuple>
#include <algorithm>
#include <chrono>   
#include <iterator> 
#include <functional> 

struct Particle {
    double x, y, theta, weight;
};

class ParticleFilterNode : public rclcpp::Node {

public:
    explicit ParticleFilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
        cv::Mat map_img_;
        cv::Mat dmap_;
        double  map_resolution_;
        double  map_origin_x_, map_origin_y_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
        std::vector<Particle> particles;
        bool dmap_computed_ = false;
        bool particles_initialized_ = false;
        bool map_received_           = false;
        bool has_prev_odom_ = false;
        rclcpp::TimerBase::SharedPtr timer_;
        double prev_x_ = 0.0, prev_y_ = 0.0, prev_theta_ = 0.0;
        sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
        int    K_candidates_      = 5;     
        int    N_per_candidate_   = 20;   
        double sigma_init_        = 0.2; 
        std::mt19937 gen_{std::random_device{}()};
        cv::Mat rasterizeScan(const std::vector<std::pair<double,double>>& scan, double theta);
        std::vector<float> simulateLaserFromPose(double x, double y, double theta, const sensor_msgs::msg::LaserScan& scan, float step_size, float dmap_hit_thresh);
        void initializeParticles(int num_particles);
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void resampleParticles();
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void publishParticlesAsPoseArray();
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};