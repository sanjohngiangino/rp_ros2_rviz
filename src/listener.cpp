#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node {
public:
    ListenerNode() : Node("listener_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("map_path", 10)
        
        RCLCPP_INFO(this->get_logger(), "Node Loaded. Please insert path of the map:");
        std::string map_path;
        std::getline(std::cin, map_path);

        auto message = std_msgs::msg::String();
        message.data = map_path;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Selected map: %s", map_path.c_str());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}
