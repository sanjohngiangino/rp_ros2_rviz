#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rp_ros2_rviz/dmap_planner.h"
#include "rp_ros2_rviz/display_utils.h"
#include "rp_ros2_rviz/robot.h"

using namespace std;
using namespace cv;

class DmapNode : public rclcpp::Node {

private:
    DMapPlanner planner;
    std::list<Vector2f> path;
    cv::Mat background_image;
    cv::Mat shown_image;
    cv::Mat custom_image;
    std::unique_ptr<Robot> my_robot;  // 🔹 Puntatore unico a Robot (inizialmente nullo)    
    int background_mode = 0;
    bool use_gradient = false;
    float resolution = 0.05;
    float expansion_range = 1.0;

public:
    DmapNode() : Node("dmap_planner_node") {
    RCLCPP_INFO(this->get_logger(), "DmapNode started");

    custom_image = cv::imread("/home/john/Desktop/path_ros/rp_ros2_rviz/map/labirinto.png", cv::IMREAD_COLOR); // Aggiungi il percorso dell'immagine

    int rows=custom_image.rows, cols=custom_image.cols;
    float resolution=0.05;
  
    //1 compute dmap
    DMap dmap(rows, cols);
    dmap.clear();

    Vector2iVector obs = getObstaclesFromImage(custom_image);
    dmap.update(obs);
    
    my_robot = std::make_unique<Robot>(10.0f, 0.0f, 0, 1);
    cerr << "robot_pose in definition: " << my_robot->position << endl;

    float expansion_range=1;
    planner.init(resolution, 0.3, expansion_range, dmap);
    planner.computePolicy(my_robot->position);
    
    cerr << "planner ready" << endl;
    cv::namedWindow("planner", cv::WINDOW_GUI_NORMAL | cv::WINDOW_AUTOSIZE);
    
    background_image=grid2cv(planner.policy, true);

    
    
    redisplay(planner.mapping);
    cv::setMouseCallback("planner", onMouse, this);

    cerr << "una chiamata : " << endl;
}

        void recomputeBackground(DMapPlanner& planner, int new_mode) {
            if (new_mode==background_mode)
            return;
            if (new_mode==-1)
            new_mode=background_mode;
            background_mode=new_mode;
            switch(background_mode){
            case 0:
            background_image=grid2cv(planner.distances, true);
            break;
            case 1:
            background_image=grid2cv(planner.obstacle_costs, true);
            break;
            case 2:
            background_image=grid2cv(planner.policy, true);
            break;
            default:;
            }
        }
        
        void redisplay(const GridMapping& mapping) {
            if (background_image.rows==0 || background_image.cols==0)
            return;
            background_image.copyTo(shown_image);
            drawPoints(shown_image, mapping, Eigen::Isometry2f::Identity(), path, 255);
            cerr << "robot_pose in redisplay: " << my_robot->position << endl;
            Eigen::Vector2f robot_position = my_robot->position;

            const auto robot_grid_position = mapping.w2g(robot_position); 
            cv::circle(shown_image, cv::Point(robot_grid_position.y(), robot_grid_position.x()), 5, cv::Scalar(255), -1);
        
        
            cv::imshow("planner", shown_image);
        }
        
        static void onMouse(int event, int x, int y, int val, void* arg) {
            DmapNode* node = static_cast<DmapNode*>(arg);  
            DMapPlanner* planner = &node->planner;  
            

            Eigen::Vector2f world_pose=planner->mapping.g2w(Eigen::Vector2f(y,x));

            if( event == cv::EVENT_LBUTTONDBLCLK ) {
            
            planner->computePolicy(node->my_robot->position);
            planner->computePath(node->path,
                                world_pose,
                                planner->mapping.resolution*2,
                                10000,
                                node->use_gradient);
            node->my_robot->followPath(node->path);

            node->redisplay(planner->mapping);
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
    rclcpp::spin(std::make_shared<DmapNode>());
    rclcpp::shutdown();
    return 0;
}