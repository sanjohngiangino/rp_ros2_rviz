#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "world.msg" 
#include "rp_ros2_rviz/world.h"
#include "std_msgs/msg/string.hpp"
#include "rp_ros2_rviz/grid_map.h"
#include "rp_ros2_rviz/differential_drive_robot.h"

class WorldNode : public rclcpp::Node {
public:
    WorldNode() : Node("world_node") {

        map_path_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "map_path", 10, std::bind(&WorldNode::loadGridMapCallback, this, std::placeholders::_1));

        world_ = std::make_shared<World>();
        RCLCPP_INFO(this->get_logger(), "WorldNode avviato. In attesa del percorso della mappa...");

        world_publisher_ = this->create_publisher<rp_ros2_rviz::msg::World>("World", 10);
    }

private:
    rclcpp::Publisher<rp_ros2_rviz::msg::World>::SharedPtr world_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_path_subscriber_;
    std::shared_ptr<World> world_; 
    void loadGridMapCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string image_path = msg->data;  
        RCLCPP_INFO(this->get_logger(), "Pronto a caricare!");

        try {

            GridMap* grid_map = new GridMap(image_path.c_str(), 0.1, world_.get(), Isometry2f(0, 10, 0.3));
            
            DifferentialDriveRobot* ddr = new DifferentialDriveRobot(grid_map);
            ddr->pose_in_parent=Isometry2f(0,0,0);
            ddr->radius=1.5;
            
            Canvas canvas;
            canvas.init(grid_map->rows / 2, grid_map->cols / 2, 0.2);

            RCLCPP_INFO(this->get_logger(), "GridMap creata con mappa caricata!");

            while (rclcpp::ok()) {
                world_->draw(canvas);  
                canvas.show();          
                int key = cv::waitKey(10);
                
                if (key == 27) {  
                    break;
                }
            }

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricare la GridMap: %s", e.what());
        }
    }

   
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
