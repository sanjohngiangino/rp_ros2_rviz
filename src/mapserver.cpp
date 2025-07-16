#include "rp_ros2_rviz/mapserver.hpp"

MapServer::MapServer(const rclcpp::NodeOptions& options)
    : Node("map_server", options)
{
    this->declare_parameter<std::string>("map_path", "");
    this->get_parameter("map_path", map_path_);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    if (loadMap(map_path_)) {
        RCLCPP_INFO(this->get_logger(), "Start publishin in /map");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapServer::publishMap, this)
        );
    }
}

bool MapServer::loadMap(const std::string& image_path) {
    cv::Mat img = cv::imread(map_path_, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Empty Image in '%s'", image_path.c_str());
        return false;
    }

    cv::Mat custom_image = img;
        if (!custom_image.empty()) {
            cv::resize(custom_image, custom_image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        }
    RCLCPP_INFO(this->get_logger(), "- [MAP] Successful loaded: %dx%d", custom_image.cols, custom_image.rows);

    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = 0.05;
    map_msg_.info.width = custom_image.cols;
    map_msg_.info.height = custom_image.rows;
    map_msg_.info.origin.position.x = 0.0;
    map_msg_.info.origin.position.y = 0.0;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;

    map_msg_.data.resize(custom_image.rows * custom_image.cols);

    for (int y = 0; y < custom_image.rows; ++y) {
        for (int x = 0; x < custom_image.cols; ++x) {
            uint8_t pixel = custom_image.at<uint8_t>(y, x);
            int i = x + (custom_image.rows - 1 - y) * custom_image.cols;
            map_msg_.data[i] = (pixel < 127) ? 100 : 0;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Map converted in occupancy Grid");
    return true;
}

void MapServer::publishMap() {
    map_msg_.header.stamp = this->now();
    map_pub_->publish(map_msg_);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<MapServer>(options));
    rclcpp::shutdown();
    return 0;
}
