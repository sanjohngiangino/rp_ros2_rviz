#include <Eigen/Dense>
#include "rp_ros2_rviz/robot.h"


Robot::Robot(float x, float y, float orientation, int radius)
    : position(x, y), orientation(orientation), radius(radius) {
}

void Robot::move(const Eigen::Vector2f& direction) {
    position += direction;  
}

void Robot::setPosition(float x, float y) {
    position = Eigen::Vector2f(x, y);  
}

void Robot::followPath(const std::list<Eigen::Vector2f>& path) {
    // Itera sulla lista in ordine inverso
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        // Muovi il robot direttamente al prossimo punto
        this->position = *it;
        std::cout << "Robot posizione: (" << this->position << ")" << std::endl;
    }
}
