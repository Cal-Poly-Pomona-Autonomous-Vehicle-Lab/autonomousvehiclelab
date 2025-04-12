#include "arduino_comms.hpp"
#include <boost/asio.hpp>
#include <sstream>
#include <iostream>
#include <vector>
#include <termios.h>  // POSIX: for tcflush


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

    // Flush any existing data in the input buffer (POSIX-specific)
    ::tcflush(serial_port_.native_handle(), TCIFLUSH);

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

// std::vector<double> ArduinoComms::processSerialData(std::string &input) {
//     std::vector<double> values;

//     try {
//         // Remove newline and carriage return characters
//         input.erase(std::remove(input.begin(), input.end(), '\r'), input.end());
//         input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());

//         // Split the input string using ',' as the delimiter
//         std::istringstream token_stream(input);
//         std::string token;

//         while (std::getline(token_stream, token, ',')) {
//             values.push_back(std::stod(token));
//         }
//     } catch (const std::exception &e) {
//         std::cerr << "Error processing serial data: " << e.what() << std::endl;
//     }

//     return values;
// }

std::vector<double> ArduinoComms::processSerialData(std::string &input) {
    std::vector<double> values;
    try {
        // Remove newline and carriage return characters
        input.erase(std::remove(input.begin(), input.end(), '\r'), input.end());
        input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());

        // Split the input string using ',' as the delimiter
        std::istringstream token_stream(input);
        std::string token;
        int token_index = 0;
        while (std::getline(token_stream, token, ',')) {
            try {
                // For tokens 0 and 1 (velocities): default to 0 if empty.
                // For token 2 (steering): default to 511 if empty.
                double value = 0;
                if (token.empty()) {
                    value = 0.0;
                } else {
                    value = std::stod(token);
                }
                values.push_back(value);
            } catch (const std::exception &e) {
                // If conversion fails, log a warning and use default values.
                double default_value = 0.0;
                std::cerr << "Warning: token conversion failed for token index " 
                          << token_index << ". Using default value " 
                          << default_value << std::endl;
                values.push_back(default_value);
            }
            token_index++;
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

            // Log raw incoming string
            std::cout << "[Arduino RAW] " << data << std::endl;

            auto processed_data = processSerialData(data);

            if (processed_data.size() < 2) {
                std::cerr << "[ArduinoComms] Invalid number of values from serial (expected 2), got: "
                          << processed_data.size() << " | Raw: [" << data << "]" << std::endl;

                // Push default values to avoid segfault
                values.push_back(0.0);  // velocity
                values.push_back(0.0);  // steering
            } else {
                right_wheel_vel = processed_data[0];
                steering_angle  = processed_data[1];

                values.push_back(right_wheel_vel);
                values.push_back(steering_angle);
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "[ArduinoComms] Exception while reading serial: " << e.what() << std::endl;
        values.push_back(0.0);  // Safe fallback
        values.push_back(0.0);
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
