#include "arduino_comms.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::string test_port = "/dev/ttyACM0";  // Use the virtual serial port
    int baud_rate = 115200;
    int timeout = 1000;

    // Create an ArduinoComms instance
    ArduinoComms arduino(test_port, baud_rate, timeout);

    auto start_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::seconds(5);

    while (std::chrono::steady_clock::now() - start_time < duration) {
        // Send motor values
        arduino.setMotorValues(0.5, 590.0);
        std::cout << "Sent: 0.5, 590" << std::endl;

        // Wait before sending the next command (adjustable)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Read and process received data
        std::string received_data = arduino.read();
        std::cout << "Received: " << received_data << std::endl;

        auto values = arduino.processSerialData(received_data);
        if (values.size() == 3) {
            std::cout << "Parsed values - Left Wheel: " << values[0] 
                      << ", Right Wheel: " << values[1] 
                      << ", Steering Angle: " << values[2] << std::endl;
        } else {
            std::cerr << "Error: Unexpected number of values received" << std::endl;
        }
    }

    std::cout << "Finished sending values for 5 seconds." << std::endl;
    return 0;
}