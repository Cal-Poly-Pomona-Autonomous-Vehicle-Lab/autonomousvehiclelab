
#ifndef BICDRIVE_ARDUINO_TRACTION_H
#define BICDRIVE_ARDUINO_TRACTION_H

#include <string>
#include <cmath>


class Traction
{
    public:

    std::string name = "";
    double cmd = 0.0;
    double vel = 0.0;
    double pos = 0.0;

    Traction() = default;

    Traction(const std::string &wheel_name)
    {
      setup(wheel_name);
    }

    
    void setup(const std::string &wheel_name)
    {
      name = wheel_name;
    }

};


#endif // BICDRIVE_ARDUINO_TRACTION_H
