#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rp_ros2_rviz/world.h"
#include "std_msgs/msg/string.hpp"
#include "rp_ros2_rviz/grid_map.h"
#include "rp_ros2_rviz/differential_drive_robot.h"

class WorldNode : public rclcpp::Node {
public:
    WorldNode() : Node("world_node") {
        world_ = std::make_shared<World>();

        RCLCPP_INFO(this->get_logger(), "World Created. Please insert path of the map: (only png files)");
        std::string map_path;
        std::getline(std::cin, map_path);
        try {

            GridMap* grid_map = new GridMap(map_path.c_str(), 0.1, world_.get(), Isometry2f(0, 10, 0.3));
            RCLCPP_INFO(this->get_logger(), "GridMap successful created");

            DifferentialDriveRobot* ddr = new DifferentialDriveRobot(grid_map);
            ddr->pose_in_parent=Isometry2f(0,0,0);
            ddr->radius=1.5;
            
            Canvas canvas;
            canvas.init(grid_map->rows / 2, grid_map->cols / 2, 0.2);


            while (rclcpp::ok()) {
                world_->draw(canvas);  
                canvas.show();          
                int key = cv::waitKey(10);
                
                if (key == 27) {  
                    break;
                }
            }

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error wrong path.png : %s", e.what());
        }

    }

private:
    std::shared_ptr<World> world_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
