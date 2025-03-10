#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rp_ros2_rviz/world.h"
#include "std_msgs/msg/string.hpp"
#include "rp_ros2_rviz/grid_map.h"


class WorldNode : public rclcpp::Node {
public:
    WorldNode() : Node("world_node") {
        // Iscrizione al topic /map_path per ricevere il percorso della mappa
        map_path_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "map_path", 10, std::bind(&WorldNode::loadGridMapCallback, this, std::placeholders::_1));

        // Creazione del mondo
        world_ = std::make_shared<World>();
        RCLCPP_INFO(this->get_logger(), "WorldNode avviato. In attesa del percorso della mappa...");
    }

private:
    // Callback per caricare la GridMap
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_path_subscriber_;
    std::shared_ptr<World> world_;  // Mondo che contiene gli oggetti

    void loadGridMapCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string image_path = msg->data;  // Percorso immagine PNG
        RCLCPP_INFO(this->get_logger(), "Pronto a caricare!");

        try {
            // Creazione della GridMap a partire dal file immagine

            GridMap* grid_map = new GridMap(image_path.c_str(), 0.1, world_.get(), Isometry2f(0, 10, 0.3));
            
            // Aggiungi la GridMap al mondo
           // world_->addItem(grid_map);
            RCLCPP_INFO(this->get_logger(), "GridMap caricata e aggiunta al mondo!");

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricare la GridMap: %s", e.what());
        }
    }

   
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorldNode>();

    // Esegui il nodo
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
