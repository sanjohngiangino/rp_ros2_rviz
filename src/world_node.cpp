#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rp_ros2_rviz/world.h"
#include "std_msgs/msg/string.hpp"
#include "rp_ros2_rviz/grid_map.h"
#include "rp_ros2_rviz/differential_drive_robot.h"
#include "rp_ros2_rviz/lidar.h"


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
            /*
            DifferentialDriveRobot* ddr = new DifferentialDriveRobot(grid_map);
            ddr->pose_in_parent=Isometry2f(0,0,0);
            ddr->radius=1.5;
            */
            ddr_ = std::make_shared<DifferentialDriveRobot>(grid_map);
            ddr_->pose_in_parent = Isometry2f(0, 0, 0);
            ddr_->radius = 1.5;
            
            LaserScan scan(90);
            //Lidar* lid = new Lidar(scan, ddr);
            
            Canvas canvas;
            canvas.init(grid_map->rows / 2, grid_map->cols / 2, 0.2);

            control_sub_ = this->create_subscription<std_msgs::msg::String>(
                "robot_control", 10, std::bind(&WorldNode::controlCallback, this, std::placeholders::_1));
            
            
            int key=0;

            while (rclcpp::ok()) {
                world_->draw(canvas);  
                canvas.show();         
                key=cv::waitKey(1);
 
                rclcpp::spin_some(this->get_node_base_interface());  
                world_->timerTick(0.1); 
            }
                /*
            int key=0;
            float rv, tv;
            while (rclcpp::ok()) {
                tv=0; rv=0;
                world_->draw(canvas);
                canvas.show();
                key=cv::waitKey(0);
                switch(key) {
                case 81: rv=0.5; break;
                case 82: tv=1; break;
                case 84: tv=-1; break;
                case 83: rv=-0.5; break;
                default:;
                }
                ddr->rot_vel=rv;
                ddr->trans_vel=tv;
                world_->timerTick(0.1);
                if (key == 27) {  
                    break;
                }
            }*/

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error wrong path.png : %s", e.what());
        }

    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;  // Subscription for control messages
    std::shared_ptr<World> world_;
    std::shared_ptr<DifferentialDriveRobot> ddr_;

    void controlCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        float tv = 0, rv = 0;

        if (command == "tv=1") {
            tv = 1;
            msg->data = "";
        } else if (command == "tv=-1") {
            tv = -1;
            msg->data = "";

        } else if (command == "rv=0.5") {
            rv = -0.5; 
            msg->data = "";

        } else if (command == "rv=-0.5") {
            rv = 0.5;
            msg->data = "";

        } else if (command == "stop"){
            tv =0,rv=0;
        }
        RCLCPP_INFO(this->get_logger(), "Prima di cambiare i valori");
        ddr_->trans_vel = tv;
        ddr_->rot_vel = rv;
        

        RCLCPP_INFO(this->get_logger(), "Robot control updated: tv=%f, rv=%f", tv, rv);
    }
   

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
