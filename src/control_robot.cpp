#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp> 
#include <termios.h>
#include <geometry_msgs/msg/pose.hpp>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "goal_pose", 10, std::bind(&ControlNode::goalCallback, this, std::placeholders::_1));
        
            control_pub_ = this->create_publisher<std_msgs::msg::String>("robot_control", 10);
    }

private:
    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        goal_x_ = msg->position.x;
        goal_y_ = msg->position.y;
        RCLCPP_INFO(this->get_logger(), "Goal ricevuto: x=%f, y=%f", goal_x_, goal_y_);
        auto message = std_msgs::msg::String();
        std::string command;

        command = "wwwaadddsssq";
        message.data = command;

        control_pub_->publish(message); 

        RCLCPP_INFO(this->get_logger(), "Comando pubblicato: %s", command.c_str());

    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double goal_x_ = 0.0, goal_y_ = 0.0; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
