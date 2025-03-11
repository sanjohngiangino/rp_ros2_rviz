#include <rclcpp/rclcpp.hpp>


class GridMapNode : public rclcpp::Node {
public:
    GridMapNode() : Node("grid_map_node") {

        world_subscriber_ = this->create_subscription<rp_ros2_rviz::msg::World>(
            "World", 10, std::bind(&GridMapNode::onWorldReceived, this, std::placeholders::_1));

        gridmap_publisher_ = this->create_publisher<rp_ros2_rviz::msg::GridMap>("gridmap", 10);

        RCLCPP_INFO(this->get_logger(), "GridMapNode avviato.");
    }

private:
    rclcpp::Subscription<rp_ros2_rviz::msg::World>::SharedPtr world_subscriber_;
    rclcpp::Publisher<rp_ros2_rviz::msg::GridMap>::SharedPtr gridmap_publisher_;

    void onWorldReceived(const rp_ros2_rviz::msg::World::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Ricevuto stato del mondo, creando GridMap...");

        GridMap* grid_map = new GridMap(image_path.c_str(), 0.1, msg.get(), Isometry2f(0, 10, 0.3));
            // Pubblicazione della GridMap
            gridmap_publisher_->publish(grid_map);
            RCLCPP_INFO(this->get_logger(), "GridMap pubblicata su /gridmap!");       
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
