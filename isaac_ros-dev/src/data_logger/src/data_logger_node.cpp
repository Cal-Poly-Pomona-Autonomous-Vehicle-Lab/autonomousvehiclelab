#include <memory> 
#include <string>
#include <fstream>
#include <filesystem>
#include <ctime> 

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/types.h> 
#include <sys/stat.h>
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/image_encodings.hpp" 
#include "image_transport/image_transport.hpp" 
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" 

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

using std::placeholders::_1; 
namespace fs = std::filesystem;

class DataLogger: public rclcpp::Node {
	public: 
		DataLogger() : Node("data_logger_node"), m_log_count(0), m_image_count(0){

			m_subscription_steering_angle = this->create_subscription<geometry_msgs::msg::Twist>(
				"/twist_mux/cmd_vel", 10, std::bind(&DataLogger::topic_callback_steering, this, _1));
			
			/* TODO: Integrate throttle data */ 
			/* Read from front camera and check if it's open */

			m_subscription_front_camera = this->create_subscription<sensor_msgs::msg::Image>(
			 	"/video/front_camera", 10, std::bind(&DataLogger::front_camera_callback, this, _1)); 


			/* Read from left camera and check if it's open */
			m_subscription_left_camera = this->create_subscription<sensor_msgs::msg::Image>(
			 	"/video/left_camera", 10, std::bind(&DataLogger::left_camera_callback, this, _1)); 

			/* Read from right camera and check if it's open */
			m_subscription_right_camera = this->create_subscription<sensor_msgs::msg::Image>(
			 	"/video/right_camera", 10, std::bind(&DataLogger::right_camera_callback, this, _1)); 
			

			/* Create logging file if not existent */
			if (!fs::is_directory("logging")){
				fs::create_directory("logging"); 
			}

			/* Check whether directory exists or not */
			if (!fs::is_directory(M_DRIVE)){
				fs::create_directory(M_DRIVE);
			} else {
				/* TODO: Use std::distance or std::count_if */
				/* Count the number of log files in the directory */ 
				for (const auto &entry : fs::directory_iterator(M_DRIVE)) {
					m_log_count++; 
				} 
			} 

			/* Determine whether Image directory has been created or not */ 
			if (!fs::is_directory(M_IMAGE_DRIVE)){
				fs::create_directory(M_IMAGE_DRIVE); 
			} else {
				/* TODO: Use std::distance or std::count_if */
				/* Count the number of images in the directory */ 
				for (const auto &entry : fs::directory_iterator(M_IMAGE_DRIVE)) {
					m_image_count++; 
				}
			}

			/* Name the current logging file */
			m_logging_files = m_logging_files + std::to_string(m_log_count); 
			m_current_file.open(M_DRIVE + "/" + m_logging_files + M_TYPE_FILE); 

			chck_log_file_size(); 

			RCLCPP_DEBUG(this->get_logger(), "Finish Init"); 
		} 

		~DataLogger() {
			m_subscription_front_camera.reset();
			m_subscription_left_camera.reset(); 
			m_subscription_right_camera.reset();
			m_subscription_steering_angle.reset();

			if (m_current_file.is_open()) 
				m_current_file.close(); 
		}


	private:
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscription_steering_angle;  
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscription_front_camera; 
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscription_left_camera; 
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscription_right_camera;

		sensor_msgs::msg::Image::ConstSharedPtr m_left_cam; 
		sensor_msgs::msg::Image::ConstSharedPtr m_right_cam; 
		sensor_msgs::msg::Image::ConstSharedPtr m_front_cam;

		std::ofstream m_current_file; 

		/* '/logging' should contain logging folder */
		constexpr std::string M_DRIVE = "logging/logging_data"; // '/' is the location of 1tb drive  
		constexpr std::string M_IMAGE_DRIVE = "logging/image_data"; // 
		constexpr std::string M_TYPE_FILE = ".txt"; 

		std::string m_logging_files = "log_file_"; 

		size_t m_log_count;
		size_t m_image_count; 

		void chck_log_file_size() {
			double mb = fs::file_size(M_DRIVE + "/" + m_logging_files + M_TYPE_FILE) / 1024 / 1024; 

			if (mb >= 50) {
				m_logging_files.pop_back();
				m_log_count++; 
				m_logging_files = m_logging_files + std::to_string(m_log_count);

				m_current_file.close(); 
				m_current_file.open(M_DRIVE + "/" + m_logging_files + M_TYPE_FILE); 
			}
		}

		void front_camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
			m_front_cam = msg; 
		}

		void left_camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
			m_left_cam = msg; 
		}

		void right_camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
			m_right_cam = msg; 
		}

		void topic_callback_steering (const geometry_msgs::msg::Twist::SharedPtr msg) {

			if (!m_current_file.is_open()){
				RCLCPP_ERROR(this->get_logger(), "File is not open"); 
				return; 
			}

			double steering_angle = msg->angular.z;
            const int steering_angle_int = steering_angle * 61.0 + 512.0;

			if (m_front_cam == nullptr)
				return; 
			if (m_left_cam == nullptr)
				return; 
			if (m_right_cam == nullptr)
				return;

			cv_bridge::CvImagePtr cv_front_ptr; 
            cv_bridge::CvImagePtr cv_left_ptr; 
            cv_bridge::CvImagePtr cv_right_ptr; 

			cv_front_ptr = cv_bridge::toCvCopy(m_front_cam, sensor_msgs::image_encodings::BGR8);
            cv_left_ptr = cv_bridge::toCvCopy(m_left_cam, sensor_msgs::image_encodings::BGR8);
            cv_right_ptr = cv_bridge::toCvCopy(m_right_cam, sensor_msgs::image_encodings::BGR8);

			/* Check if the camera frame is empty */ 
			m_image_count++;
			std::string front_cam_result = M_IMAGE_DRIVE + "/" 
				+ "image_front_" + std::to_string(m_image_count) + ".jpg"; 
			std::string left_cam_result = M_IMAGE_DRIVE + "/" 
				+ "image_left_" + std::to_string(m_image_count) + ".jpg"; 
			std::string right_cam_result = M_IMAGE_DRIVE + "/" 
				+ "image_right_" + std::to_string(m_image_count) + ".jpg";


			chck_log_file_size();

			/* Get the time currently */
			time_t tt; 
			std::chrono::system_clock::time_point now_time = std::chrono::system_clock::now(); 
			tt = std::chrono::system_clock::to_time_t ( now_time ); 
			
			/* Checking whether imwrite fails, delete all images created previously */
			if (!cv::imwrite(right_cam_result, cv_right_ptr->image)) { 
				return;
			} 
			if (!cv::imwrite(left_cam_result, cv_left_ptr->image)) {
				fs::remove(right_cam_result);
				return;
			}
			if (!cv::imwrite(front_cam_result, cv_front_ptr->image)) {
				fs::remove(right_cam_result); 
				fs::remove(left_cam_result); 
				return;
			}

			/* Input seconds and reult to current logging file */ 
			m_current_file << tt << ": " << front_cam_result << 
				" " << left_cam_result << " " << right_cam_result <<  " " << steering_angle_int << std::endl; 

			RCLCPP_INFO(this->get_logger(), "%s", front_cam_result.c_str()); 
		} 
}; 

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<DataLogger>()); 
	rclcpp::shutdown(); 

	return 0; 
} 	
