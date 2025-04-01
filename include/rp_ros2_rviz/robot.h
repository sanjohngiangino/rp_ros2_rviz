#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>
#include <list>  
#include <iostream> 

using namespace std;

class Robot {
public:
    Eigen::Vector2f position;  // Posizione del robot (x, y)
    float orientation;         // Orientamento in radianti
    int radius;                // Raggio del robot in pixel

    // 🔹 Costruttore di default
    Robot() : position(0, 0), orientation(0.0f), radius(5) {}

    // 🔹 Costruttore con parametri
    Robot(float x, float y, float orientation, int radius);

    void move(const Eigen::Vector2f& direction);
    void setPosition(float x, float y);
    void followPath(const std::list<Eigen::Vector2f>& path);

};

#endif  // ROBOT_H
