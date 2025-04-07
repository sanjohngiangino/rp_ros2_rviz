#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"
#include "rp_ros2_rviz/robot.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"  
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std;
using namespace cv;

class WorldNode : public rclcpp::Node {

private:
    DMapPlanner planner;
    std::list<Vector2f> path;
    cv::Mat background_image;
    cv::Mat shown_image;
    cv::Mat custom_image;
    std::unique_ptr<Robot> my_robot;    
    int background_mode = 0;
    bool use_gradient = false;
    float resolution = 0.05;
    float expansion_range = 1.0;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_publisher_;  

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;

    bool continue_publishing_= true;
    bool is_following_path_=false;
    bool update_goal_=false;
    bool first_position_sent_=false;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;



public:
    WorldNode() : Node("World_Node") {
    RCLCPP_INFO(this->get_logger(), "WorldNode started");

    bool_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "stop_publish", 10, std::bind(&WorldNode::bool_callback, this, std::placeholders::_1));
     
    publisher_ = this->create_publisher<std_msgs::msg::String>("PathImage", 10);
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("goal_point", 10);

    robot_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("Robot", 10);
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "path_poses", 10,
        std::bind(&WorldNode::pathCallback, this, std::placeholders::_1));
    

    custom_image = cv::imread("/home/john/Desktop/path_ros/rp_ros2_rviz/map/labirinto.png", cv::IMREAD_COLOR);
    if (!custom_image.empty()) {
        cv::resize(custom_image, custom_image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    }
    // y , x
    my_robot = std::make_unique<Robot>(10,10, 0, 1);
    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&WorldNode::publishRobot, this));
    

    if (custom_image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Error loading image");
    } else {
        background_image = custom_image.clone();
        shown_image = background_image.clone();
    }
}   
    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        path.clear();
        for (const auto& pose : msg->poses) {
            path.emplace_back(pose.position.x, pose.position.y);
        }
        RCLCPP_INFO(this->get_logger(), "path with %lu pose received", msg->poses.size());
        followPath(msg);
    }

    void followPath(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        float dt_ms = 10.0f;

        is_following_path_ = true;

        for (const auto& pose : msg->poses) {
           
            //Eigen::Vector2f world = planner.mapping.g2w(Eigen::Vector2f(pose.position.x, pose.position.y));
            my_robot->position = Eigen::Vector2f(pose.position.x, pose.position.y);
            my_robot->orientation = 2.0f * std::atan2(pose.orientation.z, pose.orientation.w);

            geometry_msgs::msg::Point robot_msg;
            robot_msg.x = my_robot->position.x();
            robot_msg.y = my_robot->position.y();
            robot_msg.z = 0.0;
            robot_publisher_->publish(robot_msg);

            RCLCPP_INFO(this->get_logger(), "Position Robot: (%.3f, %.3f)", pose.position.x, pose.position.y);
            redisplay();
            cv::waitKey(static_cast<int>(dt_ms));

            if (update_goal_) {
                RCLCPP_WARN(this->get_logger(), "Path interruptby new gol");
                break;
            }
        }
        update_goal_ = false;
        is_following_path_ = false;
        RCLCPP_INFO(this->get_logger(), "Path Completed.");
    }
    


    void simulateLaserCone(const cv::Mat& background, cv::Mat& shown_image,
        const Eigen::Vector2f& robot_pos, float theta_rad,
        float cone_deg = 60.0f, float max_range = 100.0f) {

    float half_cone = (cone_deg * M_PI / 180.0f) / 2.0f;
    float angle_min = theta_rad - half_cone;
    float angle_max = theta_rad + half_cone;
    float angle_step = M_PI / 180.0f; // 1 degree
    float step = 1.0f;

    for (float angle = angle_min; angle <= angle_max; angle += angle_step) {
    Eigen::Vector2f dir(std::cos(angle), std::sin(angle));

    for (float d = 0; d <= max_range; d += step) {
    Eigen::Vector2f pt = robot_pos + dir * d;
    int px = static_cast<int>(pt.x());
    int py = static_cast<int>(pt.y());

    if (px < 0 || px >= background.cols || py < 0 || py >= background.rows)
    break;

    uchar pixel = background.at<cv::Vec3b>(py, px)[0]; // Canale blu

    if (pixel < 127) {
    cv::circle(shown_image, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
    break;
    }

    cv::circle(shown_image, cv::Point(px, py), 1, cv::Scalar(230, 230, 230), -1);
    }
    }
    }

    void redisplay() {
        if (background_image.empty()) return;
        background_image.copyTo(shown_image);
        
        if (my_robot) {
            Eigen::Vector2f robot_pos = my_robot->position; //130,90

            float robot_theta = my_robot->orientation;

            float radius = 5.0f;
            cv::Point center(robot_pos.x(), robot_pos.y());
        
            cv::Point end_point(
                center.x + static_cast<int>(radius * std::cos(robot_theta)),
                center.y + static_cast<int>(radius * std::sin(robot_theta))
            );
            
            simulateLaserCone(background_image, shown_image, my_robot->position, robot_theta);
            cv::circle(shown_image, cv::Point(robot_pos.x(), robot_pos.y()), 5, cv::Scalar(0, 0, 255), -1); //1300,90

            cv::line(shown_image, center, end_point, cv::Scalar(255, 0, 0), 2);


        cv::imshow("planner", shown_image);
    }
    }
    
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    
        if (msg->data && !first_position_sent_) {
            continue_publishing_ = false;
        
            if (timer_) {
                timer_->cancel();
                timer_.reset();
            }
        
            if (my_robot) {
                auto robot_pos = my_robot->position;
                geometry_msgs::msg::Point pos_msg;
                pos_msg.x = robot_pos.x();
                pos_msg.y = robot_pos.y();
                pos_msg.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Inital position robot: (%.2f, %.2f)", pos_msg.x, pos_msg.y);
                robot_publisher_->publish(pos_msg);
                first_position_sent_ = true;
            }
        }
        
    }
    

    void publishMessage() {
    
        if (continue_publishing_) {
            auto string_message = std_msgs::msg::String();
            string_message.data = "/home/john/Desktop/path_ros/rp_ros2_rviz/map/labirinto.png";
            RCLCPP_INFO(this->get_logger(), "Publishing PNG path: '%s'", string_message.data.c_str());
            publisher_->publish(string_message);
        } else {
            RCLCPP_INFO(this->get_logger(), "Stopped publishing path.");
            if (timer_) {
                timer_->cancel();
            }
        }
        
    }
    void publishRobot() {
    
        if (continue_publishing_) {
            auto string_message = std_msgs::msg::String();
            string_message.data = "/home/john/Desktop/path_ros/rp_ros2_rviz/map/labirinto.png";
            RCLCPP_INFO(this->get_logger(), "Publishing PNG path: '%s'", string_message.data.c_str());
            publisher_->publish(string_message);
        } else {
            RCLCPP_INFO(this->get_logger(), "Stopped Publishing path");
            if (timer_) {
                timer_->cancel(); 
            }
        }
        
    }

    static void onMouse(int event, int x, int y, int, void* arg) {
    WorldNode* node = static_cast<WorldNode*>(arg);  
    

    if (event == EVENT_LBUTTONDOWN) {
        if (node->is_following_path_) {
            node->update_goal_ =true;
            RCLCPP_WARN(node->get_logger(), "Goal updated. Stopping the robot.");
          //  return;
        }

        auto point_message = geometry_msgs::msg::Point();
        point_message.x = static_cast<float>(x);
        point_message.y = static_cast<float>(y);
        point_message.z = 0.0;

        RCLCPP_INFO(node->get_logger(), "Click setted to: (%f, %f)", point_message.x, point_message.y);
        node->goal_publisher_->publish(point_message);
    }
}


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WorldNode>();
    cv::namedWindow("planner", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("planner", WorldNode::onMouse, node.get());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        node->redisplay();        
        cv::waitKey(30);           
        executor.spin_some();      
    }
    rclcpp::shutdown();
    return 0;
}