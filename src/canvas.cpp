#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

class Canvas : public rclcpp::Node {
public:
    Canvas() : Node("canvas") {
        canvas_ = cv::Mat::zeros(500,500,CV_8UC3);
        //set for gray CV_8UC3
        cv::imshow("Blank Canvas", canvas_);
        RCLCPP_INFO(this->get_logger(),"Nodo di canva caricato");
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&Canvas::update_canvas, this)
        );

    }

private:
    cv::Mat canvas_;
    rclcpp::TimerBase::SharedPtr timer_;  

    void update_canvas() {
        cv::circle(canvas_, cv::Point(250, 250), 100, cv::Scalar(0, 0, 255), 2); 
        cv::imshow("Blank Canvas", canvas_);
        
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
