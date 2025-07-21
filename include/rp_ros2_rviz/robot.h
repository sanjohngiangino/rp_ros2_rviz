#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>
#include <list>  
#include <iostream> 

using namespace std;

class Robot {
public:
    Eigen::Vector2f position;  
    float orientation;         
    int radius;                
    Robot() : position(0, 0), orientation(0.0f), radius(5) {}

    Robot(float x, float y, float orientation, int radius);

    void move(const Eigen::Vector2f& direction);
    void setPosition(float x, float y);

};

#endif  
