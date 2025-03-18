#ifndef ARDUINO_COMMS_HPP
#define ARDUINO_COMMS_HPP

#include <boost/asio.hpp>
#include <vector>
#include <string>
#include <memory>

class ArduinoComms {
public:
    ArduinoComms() = default;

    ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

    std::vector<double> processSerialData(std::string &input);

    std::vector<double> getVelocityAndSteerValues();

    void setMotorValues(double speed, double steer);
    
    void disconnect();

    bool connected() const { return is_connected_; }

    double left_wheel_vel = 0.0;
    double right_wheel_vel = 0.0;
    double steering_angle = 0.0;

    std::string read();
    void write(const std::string &message);

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_{io_service_};
    bool is_connected_ = false;
};

#endif // ARDUINO_COMMS_HPP
