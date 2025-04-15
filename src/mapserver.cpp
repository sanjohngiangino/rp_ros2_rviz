// map_server_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>

class MapServer : public rclcpp::Node {
public:
    MapServer() : Node("map_server") {
        this->declare_parameter<std::string>("map_path", "");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<std::vector<double>>("origin", {0.0, 0.0});

        this->get_parameter("map_path", map_path_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("origin", origin_);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapServer::publishMap, this));

        loadMap();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_path_;
    double resolution_;
    std::vector<double> origin_;

    nav_msgs::msg::OccupancyGrid grid_msg_;
    bool map_ready_ = false;

    void loadMap() {
        RCLCPP_INFO(this->get_logger(), "🗺️ Caricamento mappa da: %s", map_path_.c_str());
        cv::Mat img = cv::imread(map_path_, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Impossibile caricare l'immagine.");
            return;
        }

        grid_msg_.header.frame_id = "map";
        grid_msg_.info.resolution = resolution_;
        grid_msg_.info.width = img.cols;
        grid_msg_.info.height = img.rows;

        grid_msg_.info.origin.position.x = origin_[0];
        grid_msg_.info.origin.position.y = origin_[1];
        grid_msg_.info.origin.position.z = 0.0;
        grid_msg_.info.origin.orientation.w = 1.0;

        grid_msg_.data.resize(img.cols * img.rows);

        for (int y = 0; y < img.rows; ++y) {
            for (int x = 0; x < img.cols; ++x) {
                int index = x + (img.rows - y - 1) * img.cols;
                uchar pixel = img.at<uchar>(y, x);
                grid_msg_.data[index] = (pixel < 127) ? 100 : 0;
            }
        }

        map_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ Mappa caricata e pronta.");
    }

    void publishMap() {
        if (!map_ready_) return;
        grid_msg_.header.stamp = this->now();
        map_pub_->publish(grid_msg_);
        RCLCPP_INFO(this->get_logger(), "Mappa pubblicata.");

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapServer>());
    rclcpp::shutdown();
    return 0;
}
