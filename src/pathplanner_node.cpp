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

        // Creazione del timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPlanner::publishBool, this));
        
        

    }

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {   
        image_path =  msg->data;
        RCLCPP_INFO(this->get_logger(), "Ricevuto PNG: '%s'", image_path.c_str());
        bool_msg.data = true;
        bool_pub_path->publish(bool_msg);
        custom_image = cv::imread(image_path, cv::IMREAD_COLOR);
        /*if (timer_) {
            timer_->cancel();  // Ferma il timer
        }*/
        int rows=custom_image.rows, cols=custom_image.cols;
        float resolution=0.05;

        DMap dmap(rows, cols);
        dmap.clear();

        Vector2iVector obs = getObstaclesFromImage(custom_image);
        dmap.update(obs);
    
        my_robot = std::make_unique<Robot>(10.0f, 0.0f, 0, 1);
        cerr << "robot_pose in definition: " << my_robot->position << endl;

        float expansion_range=1;
        planner.init(resolution, 0.3, expansion_range, dmap);
        planner.computePolicy(my_robot->position);
        
        Eigen::Vector2f world_pose=planner.mapping.g2w(Eigen::Vector2f(0,0));

        planner.computePath(path,
            world_pose,
            planner.mapping.resolution*2,
            10000,
            use_gradient);
        
            my_robot->followPath(path);
        cerr << "robot_pose in ending: " << my_robot->position << endl;

        RCLCPP_INFO(this->get_logger(), "Path completato");



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
    std::unique_ptr<Robot> my_robot;  // 🔹 Puntatore unico a Robot (inizialmente nullo)    
    std::list<Vector2f> path;
    bool use_gradient = false;

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
