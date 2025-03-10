#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/string.hpp"

class Canvas : public rclcpp::Node {
    public:
    Canvas() : Node("canvas_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "map_path", 10, std::bind(&Canvas::map_path_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for map...");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    cv::Mat canvas_;

    void map_path_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string map_path = msg->data;

        canvas_ = cv::imread(map_path, cv::IMREAD_COLOR);
        if (canvas_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot Load map from that file: %s", map_path.c_str());
            return;
        }

        int new_height = 500;
        float aspect_ratio = static_cast<float>(canvas_.cols) / canvas_.rows;
        int new_width = static_cast<int>(new_height * aspect_ratio);

        cv::resize(canvas_, canvas_, cv::Size(new_width, new_height));
        cv::imshow("Map Canvas", canvas_);
        cv::waitKey(0);

        RCLCPP_INFO(this->get_logger(), "Map loaded from: %s", map_path.c_str());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Canvas>());
    rclcpp::shutdown();
    return 0;
}
