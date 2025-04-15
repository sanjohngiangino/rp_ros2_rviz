#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <sys/ioctl.h>
#include "std_msgs/msg/bool.hpp" 

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
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
                    RCLCPP_WARN(this->get_logger(), "🛑 STOP ricevuto. Path eliminato.");
                }
            });
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControllerNode::stepLoop, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> path_;
    size_t current_index_ = 0;
    bool paused_ = false;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        path_ = msg->poses;
        current_index_ = 0;
        paused_ = false;
        RCLCPP_INFO(this->get_logger(), "📦 Nuovo path ricevuto con %zu pose", path_.size());
    }

    void stepLoop() {
        char c = readKey();
        if (c == ' ') {
            paused_ = !paused_;
            RCLCPP_WARN(this->get_logger(), paused_ ? "⏸️ Pausa attiva" : "▶️ Ripresa movimento");
        }

        if (paused_ || path_.empty() || current_index_ >= path_.size()) return;

        pub_->publish(path_[current_index_++]);
    }

    char readKey() {
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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
