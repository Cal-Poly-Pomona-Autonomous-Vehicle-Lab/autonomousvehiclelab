#include "arduino_comms.hpp"
#include <boost/asio.hpp>
#include <sstream>
#include <iostream>
#include <vector>

ArduinoComms::ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    : serial_port_(io_service_) {
    setup(serial_device, baud_rate, timeout_ms);
}

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) {
    try {
        serial_port_.open(serial_device);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        is_connected_ = true;
    } catch (const std::exception &e) {
        std::cerr << "Error setting up serial connection: " << e.what() << std::endl;
        is_connected_ = false;
    }
}

void ArduinoComms::disconnect() {
    if (is_connected_) {
        try {
            serial_port_.close();
            is_connected_ = false;
            std::cout << "Disconnected from Arduino." << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "Error while disconnecting: " << e.what() << std::endl;
        }
    }
}

std::string ArduinoComms::read() {
    if (!is_connected_) {
        throw std::runtime_error("Serial port not connected");
    }

    serial_port_.cancel();  // Cancel pending I/O
    // serial_port_.flush();   // Flush buffer

    boost::asio::streambuf buffer;
    boost::asio::read_until(serial_port_, buffer, '\n');
    std::istream is(&buffer);
    std::string data;
    std::getline(is, data);
    
    return data;
}


void ArduinoComms::write(const std::string &message) {
    if (!is_connected_) {
        throw std::runtime_error("Serial port not connected");
    }

    boost::asio::write(serial_port_, boost::asio::buffer(message));
}

std::vector<double> ArduinoComms::processSerialData(std::string &input) {
    std::vector<double> values;

    try {
        // Remove newline and carriage return characters
        input.erase(std::remove(input.begin(), input.end(), '\r'), input.end());
        input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());

        // Split the input string using ',' as the delimiter
        std::istringstream token_stream(input);
        std::string token;

        while (std::getline(token_stream, token, ',')) {
            values.push_back(std::stod(token));
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing serial data: " << e.what() << std::endl;
    }

    return values;
}

std::vector<double> ArduinoComms::getVelocityAndSteerValues() {
    std::vector<double> values;
    try {
        if (serial_port_.is_open()) {
            std::string data = read();
            auto processed_data = processSerialData(data);
            if (processed_data.size() == 3) {
                left_wheel_vel = processed_data[0];
                right_wheel_vel = processed_data[1];
                steering_angle = processed_data[2];

                values.push_back((left_wheel_vel + right_wheel_vel)/2);
                values.push_back(steering_angle);
            } else {
                std::cerr << "Invalid number of tokens received: " << data << std::endl;
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Error reading velocity and steer values: " << e.what() << std::endl;
    }

    return values;
}

void ArduinoComms::setMotorValues(double speed, double steer) {
    try {
        std::stringstream ss;
        ss << speed << "," << steer << "\n";
        write(ss.str());
    } catch (const std::exception &e) {
        std::cerr << "Error writing motor values: " << e.what() << std::endl;
    }
}
