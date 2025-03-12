#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp> 
#include <termios.h>

char getKey() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        control_pub_ = this->create_publisher<std_msgs::msg::String>("robot_control", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::controlLoop, this)
        );
    }

private:
    void controlLoop() {
            RCLCPP_INFO(this->get_logger(), "Sono dentro a control");

            char key = getKey();

            std::string command;
            /*
            if (command == "q") {  // Puoi usare "q" per uscire dal ciclo se lo desideri
                RCLCPP_INFO(this->get_logger(), "Uscita dal nodo");
                rclcpp::shutdown();
                return;
            }
            */
            auto message = std_msgs::msg::String();
            if (key == 65) {  
                command = "tv=1";  
            } else if (key == 66) {  
                command = "tv=-1";  
            } else if (key == 67) {  
                command = "rv=0.5";  
            } else if (key == 68) { 
                command = "rv=-0.5";  
            } else if (key == 113) { 
                command ="stop";
            } 
            message.data = command;

            control_pub_->publish(message); 
    
            RCLCPP_INFO(this->get_logger(), "Comando pubblicato: %s", command.c_str());
        
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;  
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
