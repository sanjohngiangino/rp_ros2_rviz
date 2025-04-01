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

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        RCLCPP_INFO(this->get_logger(), "PathPlanner started");
        bool_msg.data = false;

        // Creazione della subscription e pubblicazione
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "PathImage", 10, std::bind(&PathPlanner::string_callback, this, std::placeholders::_1));

        bool_pub_path = this->create_publisher<std_msgs::msg::Bool>("stop_publish", 10);
        
        robot_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "Robot", 10, std::bind(&PathPlanner::robotCallback, this, std::placeholders::_1));
        
        // Creazione del timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPlanner::publishBool, this));
        
        

    }

private:
    void robotCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_robot_position = Eigen::Vector2f(msg->x, msg->y);
        has_robot_position_ = true;
        RCLCPP_INFO(this->get_logger(), "📡 Posizione robot aggiornata: (%.2f, %.2f)", 
                    latest_robot_position.x(), latest_robot_position.y());
        if (!dmap_ready_) return;

        
        makePath(this->dmap, 0.05, 1.0);

    }

    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {   
        image_path =  msg->data;
        RCLCPP_INFO(this->get_logger(), "Ricevuto PNG: '%s'", image_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Inizializzazione Dmap");

        bool_msg.data = true;
        bool_pub_path->publish(bool_msg);
        custom_image = cv::imread(image_path, cv::IMREAD_COLOR);
        /*if (timer_) {
            timer_->cancel();  // Ferma il timer
        }*/
        int rows=custom_image.rows, cols=custom_image.cols;

        dmap =std::make_shared<DMap>(rows, cols);
        dmap->clear();

        Vector2iVector obs = getObstaclesFromImage(custom_image);
        dmap->update(obs);
        dmap_ready_ = true;
        path_generated_ = false;

        RCLCPP_INFO(this->get_logger(), "✅ DMap pronto. Aspetto posizione robot...");
    }

    void makePath(const std::shared_ptr<DMap>& dmap, float resolution, float expansion_range){
        int retries = 200;
        while (!has_robot_position_ && rclcpp::ok() && retries-- > 0) {
            RCLCPP_WARN(this->get_logger(), "⏳ In attesa della posizione del robot...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!has_robot_position_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Posizione del robot non ricevuta. Aborto pianificazione.");
            return;
        }
            
        RCLCPP_INFO(this->get_logger(), "🤖 Posizione robot da topic: (%.2f, %.2f)", 
                latest_robot_position.x(), latest_robot_position.y());

        planner.init(resolution, 0.3, expansion_range, *dmap);
        //planner.computePolicy(my_robot->position);
    
        Eigen::Vector2f world_goal = planner.mapping.g2w(latest_robot_position);
    
        planner.computePath(path, world_goal, planner.mapping.resolution * 2, 10000, use_gradient);
    
        //my_robot->followPath(path);
        /*
        RCLCPP_INFO(this->get_logger(), "🏁 Path completato. Posizione finale: (%.2f, %.2f)",
                    my_robot->position.x(), my_robot->position.y());*/
    }
    

    void publishBool()
    {
        bool_pub_path->publish(bool_msg);
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

    std_msgs::msg::Bool bool_msg;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub_path;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string image_path;
    cv::Mat custom_image;
    DMapPlanner planner;
    Eigen::Vector2f latest_robot_position = Eigen::Vector2f::Zero();
    bool has_robot_position_ = false;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position_sub_;
    //std::unique_ptr<Robot> my_robot;  // 🔹 Puntatore unico a Robot (inizialmente nullo)    
    std::list<Vector2f> path;
    bool use_gradient = false;
    bool dmap_ready_ = false;
    bool path_generated_ = false;
    std::shared_ptr<DMap>  dmap;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto path_planner_node = std::make_shared<PathPlanner>();

    // Creazione dell'esecutore
    rclcpp::executors::MultiThreadedExecutor executor;

    // Aggiungi il nodo all'esecutore
    executor.add_node(path_planner_node);

    // Esegui l'esecutore, che gestisce lo spin in modo automatico
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
