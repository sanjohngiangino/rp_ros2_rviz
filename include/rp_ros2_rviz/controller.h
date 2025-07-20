#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <sys/ioctl.h>
#include "std_msgs/msg/bool.hpp" 

#include "rp_ros2_rviz/world.h"
class ControllerNode : public rclcpp::Node {
public:
explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> path_;
    size_t current_index_ = 0;
    bool paused_ = false;

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void stepLoop();
    char readKey();
};