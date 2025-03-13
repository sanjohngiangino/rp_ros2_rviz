#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rp_ros2_rviz/world.h"
#include "std_msgs/msg/string.hpp"
#include "rp_ros2_rviz/grid_map.h"
#include "rp_ros2_rviz/differential_drive_robot.h"
#include "rp_ros2_rviz/lidar.h"
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"

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
            
            DMap* dmap = new DMap(grid_map->rows, grid_map->cols);
            dmap->clear();

            Vector2iVector obstacles = grid_map->getObstacles();
            dmap->update(obstacles);

            RCLCPP_INFO(this->get_logger(), "Ostacoli aggiornati nella Dmap");

            DMapPlanner planner;
            float expansion_range=1;

            planner.init(0.05, 0.3, expansion_range, *dmap);
            
            planner.computePolicy(Vector2f::Zero());
            RCLCPP_INFO(this->get_logger(), "Planner Ready");


            //printObstacles(obstacles);
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

            while (rclcpp::ok()) {
                world_->draw(canvas);  

                grid_map->drawObstacles(canvas);

                canvas.show();

                //cv::setMouseCallback("canvas", onMouse, this);

                rclcpp::spin_some(this->get_node_base_interface());  
                world_->timerTick(0.1); 
                cv::waitKey(1);
            }

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error wrong path.png : %s", e.what());
        }

    }
    /*

    static void onMouse(int event, int x, int y, int val, void* arg) {

        DMapPlanner* planner=static_cast<DMapPlanner*>(arg);

        Eigen::Vector2f world_pose=planner->mapping.g2w(Eigen::Vector2f(y,x));
        
        if( event == cv::EVENT_RBUTTONDOWN ) {
          cerr << "grid_pose: " << x << " " << y << " world_pose: " << world_pose.transpose() << endl;
          cerr << "compute policy " << endl;
          planner->computePolicy(world_pose);
          redisplay(planner->mapping);
        } else {
          printPath(path);
          planner->computePath(path,
                               world_pose,
                               planner->mapping.resolution*2,
                               10000,
                               use_gradient);
          //redisplay(planner->mapping);
          printPath(path);
         
        }
      
        cerr << "background_mode= " << background_mode << endl;
        switch(background_mode) {
        case 0:
          cerr << "\r distance: " << planner->distances.at(y,x)*planner->mapping.resolution << "        ";
          break;
        case 1:
          cerr << "\r cost: " << planner->obstacle_costs.at(y,x) << "        ";
          break;
        case 2:
          Eigen::Vector2i parent_pos(-10000, -10000);
          auto &d_cell=planner->d_grid.at(y,x);
          if (d_cell.parent)
            parent_pos=planner->d_grid.ptr2rc(d_cell.parent);
          cerr << "\r policy: " << y << " " << x << " val: " << planner->policy.at(y,x) << " parent: " << parent_pos.transpose() << "             ";
          break;
        }
      } */

private:
bool use_gradient=false;
int background_mode=0; // 0: DMap, 1:Obstacles, 2: Policy

cv::Mat background_image;
cv::Mat shown_image;
std::list<Vector2f> path;

void controlCallback(const std_msgs::msg::String::SharedPtr msg) {
    command_sequence = msg->data;
    command_queue = std::queue<char>(std::deque<char>(command_sequence.begin(), command_sequence.end()));
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

//Planner functions
void redisplay(const GridMap& mapping) {
    if (background_image.rows==0 || background_image.cols==0)
      return;
    background_image.copyTo(shown_image);
    drawPoints(shown_image, mapping, Eigen::Isometry2f::Identity(), path, 255);
    cv::imshow("canvas", shown_image);
  }

void printObstacles(const Vector2iVector& obstacles) {
    for (const Eigen::Vector2i& obstacle : obstacles) {
        std::cout << "Ostacolo trovato alla cella: ("
                  << obstacle.x() << ", "
                  << obstacle.y() << ")" << std::endl;
    }
}
    /*
    void publishRobotPose() {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = ddr_->pose_in_parent.x();
        pose_msg.position.y = ddr_->pose_in_parent.y();
        pose_msg.orientation.z = sin(ddr_->pose_in_parent.theta() / 2.0);
        pose_msg.orientation.w = cos(ddr_->pose_in_parent.theta() / 2.0);

        position_pub_->publish(pose_msg);
    }
   
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

    } */
   

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
