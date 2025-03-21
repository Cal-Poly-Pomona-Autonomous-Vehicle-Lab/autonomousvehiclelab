#ifndef ARDUINO_CAR_HARDWARE_STEERING_HPP
#define ARDUINO_CAR_HARDWARE_STEERING_HPP

#include <string>
#include <cmath>

class Steering
{
public:
    std::string name = "";
    double pos = 0.0;
    double cmd = 0.0;

    Steering() = default;

    Steering(const std::string &steering_name)
    {
        setup(steering_name);
    }

    void setup(const std::string &steering_name)
    {
        name = steering_name;
    }
};


#endif // BICDRIVE_ARDUINO_STEERING_H