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

    //const float RESOLUTION = 0.05;
    //const float EXPANSION_RANGE = 1.0;
    const float ROBOT_RADIUS = 5.0f;
    //const float PARTICLE_RADIUS = 3.0f;
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

    //rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;

    //rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr step_pose_sub_;


public:
    WorldNode() : Node("World_Node") {

    RCLCPP_INFO(this->get_logger(), "WorldNode started");
    
    // Declaration of subscriber and publishers

    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    publisher_ = this->create_publisher<std_msgs::msg::String>("PathImage", 10);
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("goal_point", 10);
    controller_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("stop_controller", 10);
    robot_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("Robot", 10);
    
    bool_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "stop_publish", 10, std::bind(&WorldNode::bool_callback, this, std::placeholders::_1));
    
    step_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "step_pose", 10,
        std::bind(&WorldNode::stepPoseCallback, this, std::placeholders::_1));
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&WorldNode::mapCallback, this, std::placeholders::_1));

    particle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "particles", 10,
        [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
            particles_ = msg;
        });

    //Robot to change in order to initialPose
    my_robot = std::make_unique<Robot>(351,251, 0, 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&WorldNode::publishSimulatedOdomAndScan, this)
    );

}   

    void publishSimulatedOdomAndScan() {
        if (!my_robot || background_image.empty()) return;

        background_image.copyTo(shown_image);
        
        simulateLaserConeAndPublishScan(background_image, my_robot->position, my_robot->orientation);

        redisplay();
    }



    void stepPoseCallback(const geometry_msgs::msg::Pose::SharedPtr pose) {
        my_robot->position = Eigen::Vector2f(pose->position.x, pose->position.y);
        my_robot->orientation = 2.0f * std::atan2(pose->orientation.z, pose->orientation.w);

        geometry_msgs::msg::Point robot_msg;
        robot_msg.x = my_robot->position.x();
        robot_msg.y = my_robot->position.y();
        robot_msg.z = 0.0;
        robot_publisher_->publish(robot_msg);

        redisplay();
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
    


    void simulateLaserConeAndPublishScan(
        const cv::Mat& background,
        const Eigen::Vector2f& robot_pos,
        float theta_rad,
        float cone_deg = 270.0f,
        float max_range = 150.0f,
        float angle_step_deg = 1.0f)
    {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    
        scan_msg->header.stamp = this->get_clock()->now();
        scan_msg->header.frame_id = "base_laser";
        scan_msg->angle_min = -cone_deg * M_PI / 360.0f;
        scan_msg->angle_max = cone_deg * M_PI / 360.0f;
        scan_msg->angle_increment = angle_step_deg * M_PI / 180.0f;
        scan_msg->range_min = 0.0f;
        scan_msg->range_max = max_range;
    
        int num_beams = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.resize(num_beams, scan_msg->range_max);
    
        float angle = theta_rad + scan_msg->angle_min;
    
        for (int i = 0; i < num_beams; ++i, angle += scan_msg->angle_increment) {
            Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
    
            for (float d = scan_msg->range_min; d <= scan_msg->range_max; d += 1.0f) {
                Eigen::Vector2f pt = robot_pos + dir * d;
                int px = static_cast<int>(pt.x());
                int py = static_cast<int>(pt.y());
    
                if (px < 0 || px >= background.cols || py < 0 || py >= background.rows) break;
    
                uchar pixel = background.at<cv::Vec3b>(py, px)[0];
                if (pixel < 127) {
                    scan_msg->ranges[i] = d;
                    break;
                }
            }
        }
    
        // Publish LaserScan and Odometry

        nav_msgs::msg::Odometry odom_msg;
        rclcpp::Time now = scan_msg->header.stamp;
    
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = robot_pos.x();
        odom_msg.pose.pose.position.y = robot_pos.y();
        odom_msg.pose.pose.position.z = 0.0;
    
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_rad);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        laser_pub_->publish(*scan_msg);
        odom_publisher_->publish(odom_msg);
    }

    void drawSimulatedLaserOnImage(
        const cv::Mat& background,
        cv::Mat& shown_image,
        const Eigen::Vector2f& robot_pos,
        float theta_rad,
        float cone_deg = 270.0f,
        float max_range = 150.0f,
        float angle_step_deg = 1.0f)
    {
        float angle_min = -cone_deg * M_PI / 360.0f;
        float angle_max = cone_deg * M_PI / 360.0f;
        float angle_increment = angle_step_deg * M_PI / 180.0f;
    
        int num_beams = std::ceil((angle_max - angle_min) / angle_increment);
        float angle = theta_rad + angle_min;
    
        for (int i = 0; i < num_beams; ++i, angle += angle_increment) {
            Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
    
            for (float d = 0.0f; d <= max_range; d += 1.0f) {
                Eigen::Vector2f pt = robot_pos + dir * d;
                int px = static_cast<int>(pt.x());
                int py = static_cast<int>(pt.y());
    
                if (px < 0 || px >= background.cols || py < 0 || py >= background.rows) break;
    
                uchar pixel = background.at<cv::Vec3b>(py, px)[0];
                if (pixel < 127) {
                    cv::circle(shown_image, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);  // rosso: colpito
                    break;
                }
    
                cv::circle(shown_image, cv::Point(px, py), 1, cv::Scalar(200, 200, 200), -1);  // grigio: libero
            }
        }
    }
    
    
    

    void redisplay() {
        if (background_image.empty()) return;
        background_image.copyTo(shown_image);
        
        
        if (my_robot) {
            Eigen::Vector2f robot_pos = my_robot->position; //130,90

            float robot_theta = my_robot->orientation;

            cv::Point center(robot_pos.x(), robot_pos.y());
        
            cv::Point end_point(
                center.x + static_cast<int>(ROBOT_RADIUS * std::cos(robot_theta)),
                center.y + static_cast<int>(ROBOT_RADIUS * std::sin(robot_theta))
            );
            
            drawSimulatedLaserOnImage(background_image, shown_image, my_robot->position, my_robot->orientation);
           
            if (particles_) {
                for (const auto& pose : particles_->poses) {
                    int x = static_cast<int>(pose.position.x);
                    int y = static_cast<int>(pose.position.y);
                    int theha = 2.0f * std::atan2(pose.orientation.z, pose.orientation.w);

                    cv::Point centerp(pose.position.x, pose.position.y);
                    cv::Point end_pointp(
                        centerp.x + static_cast<int>(ROBOT_RADIUS * std::cos(theha)),
                        centerp.y + static_cast<int>(ROBOT_RADIUS * std::sin(theha))
                    );

                    cv::arrowedLine(shown_image, centerp, end_pointp, cv::Scalar(0, 255, 0), 1, 8, 0, 0.4);

                }
            }
            cv::circle(shown_image, cv::Point(robot_pos.x(), robot_pos.y()), 5, cv::Scalar(0, 0, 255), -1);
            cv::line(shown_image, center, end_point, cv::Scalar(255, 0, 0), 2);
            
        }    
     
        cv::imshow("planner", shown_image);
    }
    
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    
        if (msg->data && !first_position_sent_) {
            //continue_publishing_ = false;
        
            /*if (timer_) {
                timer_->cancel();
                timer_.reset();
            }
            */
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

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        int width = msg->info.width;
        int height = msg->info.height;
    
        // Convert OccupancyGrid to CV grayscale image
        background_image = cv::Mat::zeros(height, width, CV_8UC3);
    
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int i = x + (height - 1 - y) * width;
                int val = msg->data[i];
    
                uchar pixel;
                if (val == -1) pixel = 128;        // unknown = gray
                else if (val >= 50) pixel = 0;     // occupied = black
                else pixel = 255;                 // free = white
    
                background_image.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel, pixel, pixel);
            }
        }
    
        shown_image = background_image.clone();
        RCLCPP_INFO(this->get_logger(), "✅ Map received and converted to OpenCV image.");
        
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
    

    void publishRobot() {
    
        if (continue_publishing_) {
            auto string_message = std_msgs::msg::String();
            string_message.data = "/home/john/rp_ros2_rviz/map/labirinto.png";
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

        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        node->controller_stop_publisher_->publish(stop_msg);
        RCLCPP_INFO(node->get_logger(), "🛑 Stop inviato al Controller.");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        geometry_msgs::msg::Point pos_msg;
        pos_msg.x = node->my_robot->position.x();
        pos_msg.y = node->my_robot->position.y();
        pos_msg.z = 0.0;
        node->robot_publisher_->publish(pos_msg);
        RCLCPP_INFO(node->get_logger(), "📍 Posizione attuale pubblicata: (%.2f, %.2f)", pos_msg.x, pos_msg.y);
        
        auto point_message = geometry_msgs::msg::Point();
        point_message.x = static_cast<float>(x);
        point_message.y = static_cast<float>(y);
        point_message.z = 0.0;

        RCLCPP_INFO(node->get_logger(), "Click setted to: (%f, %f)", point_message.x, point_message.y);
        node->goal_publisher_->publish(point_message);
    }
}

void handleKeyboard(int key) {
    float step = 2.0f;
    float rotation_step = 0.15f;
    Eigen::Vector2f delta(0, 0);
    bool should_stop_controller = false;

    switch (key) {
        case 'w': delta.y() -= step; should_stop_controller = true; break;
        case 's': delta.y() += step; should_stop_controller = true; break;
        case 'a': delta.x() -= step; should_stop_controller = true; break;
        case 'd': delta.x() += step; should_stop_controller = true; break;
        case 'q':
            my_robot->orientation -= rotation_step;
            should_stop_controller = true;
            redisplay();
            return;
        case 'e':
            my_robot->orientation += rotation_step;
            should_stop_controller = true;
            redisplay();
            return;
        default:
            return;
    }

    if (should_stop_controller) {
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        controller_stop_publisher_->publish(stop_msg);
        update_goal_ = true;
        is_following_path_ = false;
        RCLCPP_WARN(this->get_logger(), "🛑 Stop pubblicato: controllo manuale attivo.");
    }

    Eigen::Vector2f new_pos = my_robot->position + delta;

    if (new_pos.x() < 0 || new_pos.x() >= background_image.cols ||
        new_pos.y() < 0 || new_pos.y() >= background_image.rows) {
        RCLCPP_WARN(this->get_logger(), "📦 Movimento fuori mappa.");
        return;
    }

    uchar pixel_value = background_image.at<cv::Vec3b>(
        static_cast<int>(new_pos.y()), static_cast<int>(new_pos.x()))[0];

    if (pixel_value < 127) {
        RCLCPP_WARN(this->get_logger(), "⛔ Ostacolo rilevato. Movimento annullato.");
        return;
    }

    my_robot->position = new_pos;

    if (delta.norm() > 0.01f) {
        my_robot->orientation = std::atan2(delta.y(), delta.x());
    }

    geometry_msgs::msg::Point robot_msg;
    robot_msg.x = new_pos.x();
    robot_msg.y = new_pos.y();
    robot_msg.z = 0.0;
    robot_publisher_->publish(robot_msg);

    redisplay();
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
        int key = cv::waitKey(1);
        if (key != -1) {
            node->handleKeyboard(key);
        }

        executor.spin_some();      
    }
    rclcpp::shutdown();
    return 0;
}