#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"
#include "rp_ros2_rviz/robot.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        RCLCPP_INFO(this->get_logger(), "PathPlanner started");
        bool_msg.data = false;

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "PathImage", 10, std::bind(&PathPlanner::string_callback, this, std::placeholders::_1));

        bool_pub_path = this->create_publisher<std_msgs::msg::Bool>("stop_publish", 10);
        
        robot_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "Robot", 10, std::bind(&PathPlanner::robotCallback, this, std::placeholders::_1));
        
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path_poses", 10);
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "goal_point", 10, std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPlanner::publishBool, this));
        
        

    }

private:


    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_goal_position = Eigen::Vector2f(msg->x, msg->y);
        
        //Eigen::Vector2f goal_world = planner.mapping.w2g(latest_goal_position);
        
        has_goal_ = true;

        if (!is_moving_ && has_robot_position_ && dmap_ready_ ) {
            is_moving_= true;
            RCLCPP_INFO(this->get_logger(), "Click received: (%.2f, %.2f)", latest_goal_position.x(), latest_goal_position.y());
            makePath(this->dmap, 0.05, 1.0,latest_goal_position);
            is_moving_= false;

        }
    }


    void robotCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_robot_position = Eigen::Vector2f(msg->x, msg->y);
        has_robot_position_ = true;
        /*RCLCPP_INFO(this->get_logger(), "📡 Posizione robot aggiornata: (%.2f, %.2f)", 
                    latest_robot_position.x(), latest_robot_position.y());*/
        if (!dmap_ready_) return;
        
        //makePath(this->dmap, 0.05, 1.0);

    }

    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {   
        image_path =  msg->data;
        RCLCPP_INFO(this->get_logger(), "Received PNG: '%s'", image_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Start Dmap");

        bool_msg.data = true;
        bool_pub_path->publish(bool_msg);
        custom_image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (!custom_image.empty()) {
            cv::resize(custom_image, custom_image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        }
        /*if (timer_) {
            timer_->cancel();  // Ferma il timer
        }*/
        int cols=custom_image.rows, rows=custom_image.cols;

        //RCLCPP_INFO(this->get_logger(), "Image size: rows = %d, cols = %d", rows, cols);

        dmap =std::make_shared<DMap>(rows, cols);
        dmap->clear();
        RCLCPP_INFO(this->get_logger(), "Getting the obstacles..");

        Vector2iVector obs = getObstaclesFromImage(custom_image);
        RCLCPP_INFO(this->get_logger(), "OK obstacles,updating dmap..");

        dmap->update(obs);
        RCLCPP_INFO(this->get_logger(), "OK updating dmap..");
        dmap_ready_ = true;
        path_generated_ = false;

        
        planner.init(0.05, 0.3, 1.0, *dmap);

        RCLCPP_INFO(this->get_logger(), "Mapping size: %d rows × %d cols", 
    planner.mapping.rows, planner.mapping.cols);
    
    RCLCPP_INFO(this->get_logger(), "Mapping center: (%.2f, %.2f)", 
        planner.mapping.center.x(), planner.mapping.center.y());
        
        RCLCPP_INFO(this->get_logger(), "DMap ready planner initalized. Waiting Goal Click...");


    }

    void makePath(const std::shared_ptr<DMap>& dmap, float resolution, float expansion_range, const Eigen::Vector2f& goal_position){
        int retries = 200;
        while (!has_robot_position_ && rclcpp::ok() && retries-- > 0) {
            RCLCPP_WARN(this->get_logger(), "Waiting Robot position...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!has_robot_position_) {
            RCLCPP_ERROR(this->get_logger(), "No robot position.");
            return;
        }
        auto inBounds = [&](const Eigen::Vector2f& p) {
            return (p.x() >= 0 && p.x() < planner.mapping.rows &&
                    p.y() >= 0 && p.y() < planner.mapping.cols);
        };
    
        if (!inBounds(goal_position) || !inBounds(latest_robot_position)) {
            RCLCPP_ERROR(this->get_logger(), "Gol out of bound");
            return;
        }


        Eigen::Vector2f click = planner.mapping.g2w(goal_position);

        planner.computePolicy(click);
    
        Eigen::Vector2f world_goal = planner.mapping.g2w(latest_robot_position);

        RCLCPP_INFO(this->get_logger(), "Start Position: (%.2f, %.2f)", 
        latest_robot_position.x(), latest_robot_position.y());
        
        planner.computePath(path, world_goal, planner.mapping.resolution * 2, 3000, use_gradient);

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        pose_array.header.stamp = this->now();
        
        Eigen::Vector2f previous;
        bool has_prev = false;

        for (const auto& p : path) {
            Eigen::Vector2f world_point = planner.mapping.w2g(p);

            geometry_msgs::msg::Pose pose;
            pose.position.x = world_point.x();
            pose.position.y = world_point.y();
            pose.position.z = 0.0;

            if (has_prev) {
                Eigen::Vector2f dir = (world_point - previous).normalized();
                float yaw = std::atan2(dir.y(), dir.x());

                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = std::sin(yaw * 0.5);
                pose.orientation.w = std::cos(yaw * 0.5);
            } else {
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;
            }

            pose_array.poses.push_back(pose);
            previous = world_point;
            has_prev = true;
        }

        
        
        RCLCPP_INFO(this->get_logger(), "Publishing %ld poses to world node", pose_array.poses.size());
        pose_array_pub_->publish(pose_array);
        
    }
    
    
    
    void publishBool()
    {
        bool_pub_path->publish(bool_msg);
    }

    Vector2iVector getObstaclesFromImage(const cv::Mat& image) {
        Vector2iVector obstacles;
    
        if (image.empty()) {
            std::cerr << "No image!" << std::endl;
            return obstacles;
        }
    
        cv::Mat gray;
        if (image.channels() == 3)
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        else
            gray = image.clone();
    
        cv::Mat binary;
        cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY_INV);
    
        std::vector<cv::Point> nonZeroPoints;
        cv::findNonZero(binary, nonZeroPoints);
    
        for (const auto& pt : nonZeroPoints)
            obstacles.push_back(Vector2i(pt.x, pt.y));
        /*
        cv::Mat vis = cv::Mat::zeros(binary.size(), CV_8UC3);
        for (const auto& obs : obstacles) {
            vis.at<cv::Vec3b>(obs.y(), obs.x()) = cv::Vec3b(0, 0, 255);
        }
        
        cv::imshow("Ostacoli rilevati", vis);
        cv::imwrite("/tmp/ostacoli.png", vis);
        cv::waitKey(1); 
        */
        return obstacles;
    }
    


    std_msgs::msg::Bool bool_msg;
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto path_planner_node = std::make_shared<PathPlanner>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(path_planner_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
