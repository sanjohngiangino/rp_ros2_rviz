#include "rp_ros2_rviz/controller.h"


ControllerNode::ControllerNode(const rclcpp::NodeOptions& options) 
: Node("controller_node", options){

        sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "path_poses", 10,
            std::bind(&ControllerNode::pathCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Pose>("step_pose", 10);
        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "stop_controller", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    path_.clear();
                    current_index_ = 0;
                    paused_ = false;
                    RCLCPP_WARN(this->get_logger(), "Received stop. Actual path cancelled.");
                }
            });
    
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControllerNode::stepLoop, this));
    }


    void ControllerNode::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        path_ = msg->poses;
        current_index_ = 0;
        paused_ = false;
        RCLCPP_INFO(this->get_logger(), "Received Path with %zu pose", path_.size());
    }

    void ControllerNode::stepLoop() {
        char c = readKey();
        if (c == ' ') {
            paused_ = !paused_;
            RCLCPP_WARN(this->get_logger(), paused_ ? "Pause Movement" : "Movement Restarted");
        }

        if (paused_ || path_.empty() || current_index_ >= path_.size()) return;

        pub_->publish(path_[current_index_++]);
    }

    char ControllerNode::readKey() {
        struct termios oldt, newt;
        char c = 0;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        int bytesWaiting;
        ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
        if (bytesWaiting > 0) {
            c = getchar();
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
