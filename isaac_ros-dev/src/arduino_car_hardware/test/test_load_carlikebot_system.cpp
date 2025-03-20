#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "carlikebot_system.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
#include <thread>

using namespace bicdrive_arduino;

// Unit tests for CarlikeBotSystemHardware with actual Arduino communication
// Test fixture for CarlikeBotSystemHardwareTest
class CarlikeBotSystemHardwareTest : public ::testing::Test {
    protected:
        std::shared_ptr<CarlikeBotSystemHardware> hardware_system;  
    
        void SetUp() override {
            // Create a node for testing (necessary for ROS2 integration)
            rclcpp::init(0, nullptr);
    
            // Initialize the hardware system with the actual Arduino comms
            hardware_system = std::make_shared<CarlikeBotSystemHardware>();
    
            // Initialize the system with hardware info
            hardware_system->on_init(createHardwareInfo());
    
            // Set up Arduino comms
            hardware_system->on_configure(rclcpp_lifecycle::State());
        }
    
        void TearDown() override {
            rclcpp::shutdown();
        }
    
        // Create the hardware info directly here
        hardware_interface::HardwareInfo createHardwareInfo() {
            hardware_interface::HardwareInfo info;
        
            // Hardware parameters
            info.hardware_parameters["rear_wheel_name"] = "rear_wheel";
            info.hardware_parameters["front_wheel_name"] = "front_wheel";
            info.hardware_parameters["loop_rate"] = "50";
            info.hardware_parameters["device"] = "/dev/ttyACM0";  // Arduino port
            info.hardware_parameters["baud_rate"] = "115200";
            info.hardware_parameters["timeout_ms"] = "1000";
        
            // --- FRONT WHEEL JOINT (STEERING) ---
            hardware_interface::ComponentInfo front_wheel;
            front_wheel.name = "front_wheel";
            front_wheel.type = "joint";
        
            // Add "position" command and state interfaces as InterfaceInfo structs
            hardware_interface::InterfaceInfo front_cmd_interface;
            front_cmd_interface.name = hardware_interface::HW_IF_POSITION;
            front_wheel.command_interfaces.push_back(front_cmd_interface);
        
            hardware_interface::InterfaceInfo front_state_interface;
            front_state_interface.name = hardware_interface::HW_IF_POSITION;
            front_wheel.state_interfaces.push_back(front_state_interface);
        
            // --- REAR WHEEL JOINT (TRACTION) ---
            hardware_interface::ComponentInfo rear_wheel;
            rear_wheel.name = "rear_wheel";
            rear_wheel.type = "joint";
        
            // Add "velocity" command and state interfaces as InterfaceInfo structs
            hardware_interface::InterfaceInfo rear_cmd_interface;
            rear_cmd_interface.name = hardware_interface::HW_IF_VELOCITY;
            rear_wheel.command_interfaces.push_back(rear_cmd_interface);
        
            hardware_interface::InterfaceInfo rear_state_interface;
            rear_state_interface.name = hardware_interface::HW_IF_VELOCITY;
            rear_wheel.state_interfaces.push_back(rear_state_interface);
        
            // Add joints to hardware info
            info.joints.push_back(front_wheel);
            info.joints.push_back(rear_wheel);
        
            return info;
        }
        
    };  
    
// Test on_init()
TEST_F(CarlikeBotSystemHardwareTest, TestOnInitSuccess) {
    auto result = hardware_system->on_init(createHardwareInfo());
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

// Test on_configure() with actual Arduino communication
TEST_F(CarlikeBotSystemHardwareTest, TestOnConfigureSuccess) {
    auto result = hardware_system->on_configure(rclcpp_lifecycle::State());
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

// Test on_activate() with actual Arduino communication
TEST_F(CarlikeBotSystemHardwareTest, TestOnActivateSuccess) {
    auto result = hardware_system->on_activate(rclcpp_lifecycle::State());
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

// Test on_deactivate()
TEST_F(CarlikeBotSystemHardwareTest, TestOnDeactivate) {
    auto result = hardware_system->on_deactivate(rclcpp_lifecycle::State());
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

// Test write() functionality with actual Arduino comms
TEST_F(CarlikeBotSystemHardwareTest, TestWrite) {
    rclcpp::Time time(0, 0);
    rclcpp::Duration period(1, 0);

    // Set command values to be sent to the Arduino
    hardware_system->set_steering_pos(512);  // Proper setter
    hardware_system->set_traction_cmd(0.5);  // Proper setter

    // Simulate writing the values to the Arduino
    auto result = hardware_system->write(time, period);
    EXPECT_EQ(result, hardware_interface::return_type::OK);

    // Additional validation can be added to ensure data is sent correctly
    // For example, by using a serial monitor on the Arduino side to verify values
}
// Test read() functionality with actual Arduino comms
TEST_F(CarlikeBotSystemHardwareTest, TestRead) {
    rclcpp::Time time(0, 0);
    rclcpp::Duration period(1, 0);

    // Simulate reading values from the Arduino
    auto result = hardware_system->read(time, period);
    EXPECT_EQ(result, hardware_interface::return_type::OK);
    printf("[INFO] Steering position: %f\n", hardware_system->get_steering_pos());
    printf("[INFO] Traction command: %f\n", hardware_system->get_traction_cmd());
    // Check if we have valid values
    EXPECT_GE(hardware_system->get_steering_pos(), 102.0); // for some reason this is the value we get when we read the serial initially
    EXPECT_GE(hardware_system->get_traction_cmd(), 0.0);
}


// Test export_state_interfaces with actual data
TEST_F(CarlikeBotSystemHardwareTest, TestExportStateInterfaces) {
    auto state_interfaces = hardware_system->export_state_interfaces();
    EXPECT_EQ(state_interfaces.size(), 2);
    // You can add additional checks for the names or types of state interfaces if needed
}

// Test export_command_interfaces with actual data
TEST_F(CarlikeBotSystemHardwareTest, TestExportCommandInterfaces) {
    auto command_interfaces = hardware_system->export_command_interfaces();
    EXPECT_EQ(command_interfaces.size(), 2);
    // Additional checks can be added if necessary
}

// Test on_cleanup()
TEST_F(CarlikeBotSystemHardwareTest, TestOnCleanup) {
    auto result = hardware_system->on_cleanup(rclcpp_lifecycle::State());
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
