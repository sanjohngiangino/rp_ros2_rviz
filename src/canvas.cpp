#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

class Canvas : public rclcpp::Node {
public:
    Canvas() : Node("canvas") {
        std::string map_path = "/home/john/Desktop/rp_ros2_rviz/map/cappero_laser_odom_diag_2020-05-06-16-26-03.png";

        canvas_ = cv::imread(map_path, cv::IMREAD_COLOR);
        if (canvas_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Impossibile caricare la mappa dal file: %s", map_path.c_str());
            return;
        }
        int new_height = 500;  
        float aspect_ratio = static_cast<float>(canvas_.cols) / canvas_.rows;
        int new_width = static_cast<int>(new_height * aspect_ratio);
        cv::resize(canvas_, canvas_, cv::Size(new_width,new_height));
        cv::imshow("Map Canvas", canvas_);
        RCLCPP_INFO(this->get_logger(), "Nodo di canvas caricato");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&Canvas::update_canvas, this)
        );

    }

private:
    cv::Mat canvas_;
    rclcpp::TimerBase::SharedPtr timer_;  
    void update_canvas() {

        cv::imshow("Map Canvas", canvas_);
        
        cv::waitKey(1);  
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Canvas>());
    rclcpp::shutdown();
    return 0;
}
