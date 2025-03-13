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
        rclcpp::TimerBase::SharedPtr timer_;

        try {

            GridMap* grid_map = new GridMap(map_path.c_str(), 0.1, world_.get(), Isometry2f(0, 0, 0));
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

            //position_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pose", 10);
            goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("goal_pose", 10);
            
            control_sub_ = this->create_subscription<std_msgs::msg::String>(
                "robot_control", 10, std::bind(&WorldNode::controlCallback, this, std::placeholders::_1));
            

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500), 
                std::bind(&WorldNode::executeNextCommand, this)
            );
            
            int key=0;
            //cv::namedWindow("World");
            //cv::setMouseCallback("World", onMouse, this);

            while (rclcpp::ok()) {
                world_->draw(canvas);  
                canvas.show();

                cv::setMouseCallback("canvas", onMouse, this);

                rclcpp::spin_some(this->get_node_base_interface());  
                world_->timerTick(0.1); 
                cv::waitKey(1);
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
void controlCallback(const std_msgs::msg::String::SharedPtr msg) {
    // Memorizza la sequenza di comandi
    command_sequence = msg->data;
    command_queue = std::queue<char>(std::deque<char>(command_sequence.begin(), command_sequence.end()));
    
    // Log
    RCLCPP_INFO(this->get_logger(), "Sequenza di comandi ricevuta: %s", msg->data.c_str());
}
void executeNextCommand() {
    if (!command_queue.empty()) {
        char command_char = command_queue.front();
        command_queue.pop(); // Estrai il comando dalla coda

        float tv = 0, rv = 0;
        if (command_char == 'w') {
            tv = 1;
       
        } else if (command_char == 'a') {
            rv = 0.5;
        } else if (command_char == 's') {
            tv = -1;
        } else if (command_char == 'd') {
            rv = -0.5;
        } else if (command_char == 'q'){
            tv = 0;
            rv = 0;
            RCLCPP_WARN(this->get_logger(), "Comando non riconosciuto: %c", command_char);
        }

        // Esegui il comando
        ddr_->trans_vel = tv;
        ddr_->rot_vel = rv;

        // Log per monitorare lo stato del robot
        RCLCPP_INFO(this->get_logger(), "Comando eseguito: tv=%f, rv=%f", tv, rv);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sequenza completata.");
    }
}
/*
void controlCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command_sequence = msg->data;  // La stringa ricevuta con la sequenza di comandi
    float tv = 0, rv = 0;

    // Itera su ogni carattere della sequenza
    for (char command_char : command_sequence) {
        if (command_char == 'w') {
            tv = 1;
            rv = 0;
        } else if (command_char == 'a') {
            rv = 0.5;
            tv = 0;
        } else if (command_char == 's') {
            tv = -1;
            rv = 0;
        } else if (command_char == 'd') {
            rv = -0.5;
            tv = 0;
        } else if (command_char == 'q'){
            tv = 0;
            rv = 0;
            RCLCPP_WARN(this->get_logger(), "Comando non riconosciuto: %c", command_char);
        }

        // Applica il comando al robot
        ddr_->trans_vel = tv;
        ddr_->rot_vel = rv;

        // Log per monitorare lo stato del robot
        RCLCPP_INFO(this->get_logger(), "Comando eseguito: tv=%f, rv=%f", tv, rv);
    }

    // Stato finale dopo aver eseguito tutta la sequenza
    RCLCPP_INFO(this->get_logger(), "Sequenza completata: tv=%f, rv=%f", tv, rv);
}*/

    /*
    void publishRobotPose() {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = ddr_->pose_in_parent.x();
        pose_msg.position.y = ddr_->pose_in_parent.y();
        pose_msg.orientation.z = sin(ddr_->pose_in_parent.theta() / 2.0);
        pose_msg.orientation.w = cos(ddr_->pose_in_parent.theta() / 2.0);

        position_pub_->publish(pose_msg);
    }
    */
   static void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN && userdata) {
        auto* self = reinterpret_cast<WorldNode*>(userdata);

        if (!self) {
            std::cerr << "Errore: userdata non valido in onMouse" << std::endl;
            return;
        }

        geometry_msgs::msg::Pose goal_msg;
        goal_msg.position.x = x * 0.1;
        goal_msg.position.y = y * 0.1;

        self->goal_pub_->publish(goal_msg);
        RCLCPP_INFO(self->get_logger(), "Goal impostato a: (%f, %f)", goal_msg.position.x, goal_msg.position.y);
    }
}


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub_;
    std::shared_ptr<World> world_;
    std::shared_ptr<DifferentialDriveRobot> ddr_;
    std::string command_sequence;  // Sequenza di comandi ricevuta
    std::queue<char> command_queue;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
