#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"
#include "rp_ros2_rviz/robot.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"  // Per il messaggio Point
#include "std_msgs/msg/bool.hpp"  // Questo è l'header corretto per il messaggio booleano
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
    
    my_robot = std::make_unique<Robot>(10.0f, 10.0f, 0, 1);
    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&WorldNode::publishRobot, this));
    

    if (custom_image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento dell'immagine.");
    } else {
        background_image = custom_image.clone();
        shown_image = background_image.clone();
    }

    /*

    if (custom_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento dell'immagine.");
        } else {
            cv::imshow("planner", custom_image);
            cv::setMouseCallback("planner", onMouse, this);

            while (true) {
                int key = cv::waitKey(1); 
                if (key == 27) { 
                    break;
                }
            }
        }

    */
    
    //cv::imshow("planner", shown_image);

    /*
    int rows=custom_image.rows, cols=custom_image.cols;
    float resolution=0.05;
    
    my_robot = std::make_unique<Robot>(10.0f, 0.0f, 0, 1);
    cerr << "robot_pose in definition: " << my_robot->position << endl;

    float expansion_range=1;
    
    cv::setMouseCallback("planner", onMouse, this);

    cerr << "una chiamata : " << endl;
    */
}   
    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        path.clear();
        for (const auto& pose : msg->poses) {
            path.emplace_back(pose.position.x, pose.position.y);
        }
        RCLCPP_INFO(this->get_logger(), "📥 Ricevuto nuovo path con %lu pose", msg->poses.size());
        followPath(msg);
    }

    void followPath(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        float dt_ms = 50.0f;
    
        for (const auto& pose : msg->poses) {
            Eigen::Vector2f world = planner.mapping.g2w(Eigen::Vector2f(pose.position.x, pose.position.y));
            my_robot->position = world;

            RCLCPP_INFO(this->get_logger(), "➡️ Robot posizione: (%.3f, %.3f)", pose.position.x, pose.position.y);
            redisplay();
            cv::waitKey(static_cast<int>(dt_ms));
        }
    
        RCLCPP_INFO(this->get_logger(), "✅ Path completato.");
    }
    


        
    void redisplay() {
        if (background_image.empty()) return;
        background_image.copyTo(shown_image);
        drawPoints(shown_image, planner.mapping, Eigen::Isometry2f::Identity(), path, 255);
        if (my_robot) {
            Eigen::Vector2f robot_pos = my_robot->position;
            auto robot_grid = planner.mapping.w2g(robot_pos);
            cv::circle(shown_image, cv::Point(robot_grid.y(), robot_grid.x()), 5, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("planner", shown_image);
    }
    
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Ricevuto: '%d'", msg->data);
    
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Ricevuto stop, fermo la pubblicazione.");
            continue_publishing_ = false;
    
            if (timer_) {
                timer_->cancel();  // Ferma il timer
                timer_.reset();    // Libera la memoria del timer
            }

            if (my_robot) {
                auto robot_pos = my_robot->position;
                geometry_msgs::msg::Point pos_msg;
                pos_msg.x = robot_pos.x();
                pos_msg.y = robot_pos.y();
                pos_msg.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "📤 Posizione robot pubblicata: (%.2f, %.2f)", pos_msg.x, pos_msg.y);
                robot_publisher_->publish(pos_msg);
            }
            
        } else {
            //RCLCPP_INFO(this->get_logger(), "Ricevuto start, continuo la pubblicazione.");
            continue_publishing_ = true;
    
            if (!timer_) {  // Se il timer non esiste, lo ricreo
                timer_ = this->create_wall_timer(
                    std::chrono::seconds(1),
                    std::bind(&WorldNode::publishMessage, this));
            }
        }
    }
    

    void publishMessage() {
    
        if (continue_publishing_) {
            // Crea e pubblica il messaggio con il percorso del PNG
            auto string_message = std_msgs::msg::String();
            string_message.data = "/home/john/Desktop/path_ros/rp_ros2_rviz/map/labirinto.png";
            RCLCPP_INFO(this->get_logger(), "Publishing PNG path: '%s'", string_message.data.c_str());
            publisher_->publish(string_message);
        } else {
            RCLCPP_INFO(this->get_logger(), "La pubblicazione è stata fermata.");
            if (timer_) {
                timer_->cancel();  // Ferma il timer
            }
        }
        
    }
    void publishRobot() {
    
        if (continue_publishing_) {
            // Crea e pubblica il messaggio con il percorso del PNG
            auto string_message = std_msgs::msg::String();
            string_message.data = "/home/john/Desktop/path_ros/rp_ros2_rviz/map/labirinto.png";
            RCLCPP_INFO(this->get_logger(), "Publishing PNG path: '%s'", string_message.data.c_str());
            publisher_->publish(string_message);
        } else {
            RCLCPP_INFO(this->get_logger(), "La pubblicazione è stata fermata.");
            if (timer_) {
                timer_->cancel();  // Ferma il timer
            }
        }
        
    }

    static void onMouse(int event, int x, int y, int, void* arg) {
    WorldNode* node = static_cast<WorldNode*>(arg);  

    if (event == EVENT_LBUTTONDOWN) {
        auto point_message = geometry_msgs::msg::Point();
        point_message.x = static_cast<float>(x);
        point_message.y = static_cast<float>(y);
        point_message.z = 0.0;

        RCLCPP_INFO(node->get_logger(), "🎯 Nuovo goal impostato: (%f, %f)", point_message.x, point_message.y);
        node->goal_publisher_->publish(point_message);
    }
}

    
    Vector2iVector getObstaclesFromImage(const cv::Mat& image) {
        Vector2iVector obstacles;  
    
        if (image.empty()) {
            std::cerr << "Immagine non trovata o vuota!" << std::endl;
            return obstacles;
        }
    
        cv::Mat gray;
        if (image.channels() == 3) {  
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();  
        }
        cv::Mat rotated;
        cv::rotate(gray, rotated, cv::ROTATE_90_CLOCKWISE);

        for (int r = 0; r < rotated.rows; ++r) {
            for (int c = 0; c < rotated.cols; ++c) {
                uchar pixel = rotated.at<uchar>(r, c);            
                if (pixel < 127) {  
                    obstacles.push_back(Vector2i(rotated.cols - c - 1, r)); 
                }
            }
        }
    
        return obstacles;
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