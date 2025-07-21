#include "rp_ros2_rviz/pathplanner.h"

PathPlanner::PathPlanner(const rclcpp::NodeOptions& options) 
: Node("pathplanner_node", options){

        RCLCPP_INFO(this->get_logger(), "PathPlanner started");
        bool_msg.data = false;

        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&PathPlanner::mapCallback, this, std::placeholders::_1)
        );

        bool_pub_path = this->create_publisher<std_msgs::msg::Bool>("stop_publish", 10);
        robot_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "Robot", 10, std::bind(&PathPlanner::robotCallback, this, std::placeholders::_1));
        
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path_poses", 10);
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "move_base/goal", 10, std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPlanner::publishBool, this));
        
        

    }


void PathPlanner::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    latest_goal_position = Eigen::Vector2f(msg->x, msg->y);        
    has_goal_ = true;

    if (!is_moving_ && has_robot_position_ && dmap_ready_ ) {
        is_moving_= true;
        RCLCPP_INFO(this->get_logger(), "Click received: (%.2f, %.2f)", latest_goal_position.x(), latest_goal_position.y());
        makePath(this->dmap, 0.05, 1.0,latest_goal_position);
        is_moving_= false;

    }
}


void PathPlanner::robotCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    latest_robot_position = Eigen::Vector2f(msg->x, msg->y);
    has_robot_position_ = true;
    RCLCPP_INFO(this->get_logger(), "Updated Robot Position: (%.2f, %.2f)", 
                latest_robot_position.x(), latest_robot_position.y());
    if (!dmap_ready_) return;
}

void PathPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    bool_msg.data = true;
    bool_pub_path->publish(bool_msg);
    // STOP SUBSCRIBING TO MAP ONCE
    if (sub_) {
        sub_.reset(); 
    }

    int rows=msg->info.width , cols=msg->info.height;


    dmap =std::make_shared<DMap>(rows, cols);
    dmap->clear();
    RCLCPP_INFO(this->get_logger(), "Getting the obstacles..");

    Vector2iVector obs = getObstaclesFromGrid(*msg);           

    RCLCPP_INFO(this->get_logger(), "OK obstacles,updating dmap..");

    dmap->update(obs);
    RCLCPP_INFO(this->get_logger(), "OK updating dmap..");
    dmap_ready_ = true;
    path_generated_ = false;

    
    planner.init(0.05, 0.3, 1.0, *dmap);

    RCLCPP_INFO(this->get_logger(), "Mapping size: %d rows × %d cols", 
    planner.mapping.rows, planner.mapping.cols);

    RCLCPP_INFO(this->get_logger(), "Mapping center: (%.2f, %.2f)", 
    planner.mapping.center.x(), planner.mapping.center.y());
    
    RCLCPP_INFO(this->get_logger(), "DMap ready planner initalized. Waiting Goal Click...");


}

void PathPlanner::makePath(const std::shared_ptr<DMap>& dmap, float resolution, float expansion_range, const Eigen::Vector2f& goal_position){
    int retries = 200;
    while (!has_robot_position_ && rclcpp::ok() && retries-- > 0) {
        RCLCPP_WARN(this->get_logger(), "Waiting Robot position...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!has_robot_position_) {
        RCLCPP_ERROR(this->get_logger(), "No robot position.");
        return;
    }
    auto inBounds = [&](const Eigen::Vector2f& p) {
        return (p.x() >= 0 && p.x() < planner.mapping.rows &&
                p.y() >= 0 && p.y() < planner.mapping.cols);
    };

    if (!inBounds(goal_position) || !inBounds(latest_robot_position)) {
        RCLCPP_ERROR(this->get_logger(), "Gol out of bound");
        return;
    }


    Eigen::Vector2f click = planner.mapping.g2w(goal_position);

    planner.computePolicy(click);

    Eigen::Vector2f world_goal = planner.mapping.g2w(latest_robot_position);

    RCLCPP_INFO(this->get_logger(), "Start Position: (%.2f, %.2f)", 
    latest_robot_position.x(), latest_robot_position.y());
    
    planner.computePath(path, world_goal, planner.mapping.resolution * 2, 3000, use_gradient);

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->now();
    
    Eigen::Vector2f previous;
    bool has_prev = false;

    for (const auto& p : path) {
        Eigen::Vector2f world_point = planner.mapping.w2g(p);

        geometry_msgs::msg::Pose pose;
        pose.position.x = world_point.x();
        pose.position.y = world_point.y();
        pose.position.z = 0.0;

        if (has_prev) {
            Eigen::Vector2f dir = (world_point - previous).normalized();
            float yaw = std::atan2(dir.y(), dir.x());

            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = std::sin(yaw * 0.5);
            pose.orientation.w = std::cos(yaw * 0.5);
        } else {
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
        }

        pose_array.poses.push_back(pose);
        previous = world_point;
        has_prev = true;
    }

    
    
    RCLCPP_INFO(this->get_logger(), "Publishing %ld poses to world node", pose_array.poses.size());
    pose_array_pub_->publish(pose_array);
    
}


using Vector2i = Eigen::Vector2i;
using Vector2iVector = std::vector<Vector2i>;


void PathPlanner::publishBool(){
    bool_pub_path->publish(bool_msg);
}

Vector2iVector PathPlanner::getObstaclesFromGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    Vector2iVector obstacles;

    int width = grid.info.width;
    int height = grid.info.height;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int flipped_y = height - 1 - y;
            int index = x + flipped_y * width;
            int value = grid.data[index];

            if (value >= 50) {
                obstacles.emplace_back(x, y); 
            }
        }
    }
    /*
    // Look Obstacles
    cv::Mat vis = cv::Mat::zeros(height, width, CV_8UC3);
    for (const auto& obs : obstacles) {
        vis.at<cv::Vec3b>(obs.y(), obs.x()) = cv::Vec3b(0, 0, 255);  // Red
    }

    cv::imshow("Ostacoli rilevati", vis);
    cv::imwrite("/tmp/ostacoli.png", vis);
    cv::waitKey(1); 
    */
    return obstacles;
}


    int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto path_planner_node = std::make_shared<PathPlanner>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(path_planner_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
