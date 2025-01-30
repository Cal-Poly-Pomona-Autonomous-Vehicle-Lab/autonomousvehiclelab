import requests
from bs4 import BeautifulSoup
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSNode(Node):
    def __init__(self):
        super().__init__('GPS_publisher')
        self.coords_publisher = self.create_publisher(NavSatFix, '/nav/raw/fix', 10)

        self.modem_url = ""
        self.username = "user"
        self.password = "truHirv3"
        self.session = requests.Session()

        if not self.login():
            self.get_logger().error("Login failed. Shutting down node.")
            self.destroy_node()
        else:
            self.start_timer()

    def login(self):
        try:
            self.get_logger().info("Starting login process")

            # Set user-agent header to mimic a browser
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }

            # Get the login page to establish initial cookies
            response = self.session.get(self.modem_url, headers=headers)
            if response.status_code != 200:
                self.get_logger().error(f"Failed to load login page. Status code: {response.status_code}")
                return False

            soup = BeautifulSoup(response.text, 'html.parser')

            # Print the page HTML for debugging
            self.get_logger().info("Login page HTML:")
            print(soup.prettify())  # Inspect this output in your terminal

            # Find the login form action URL
            login_form = soup.find('form', {'name': 'index'})  # Adjust 'index' to the form's actual name or id
            if not login_form:
                self.get_logger().error("Login form not found")
                return False

            login_action = login_form.get('action', '')
            if not login_action:
                self.get_logger().warn("No action URL found, defaulting to the current modem URL")
                login_url = self.modem_url
            else:
                login_url = self.modem_url + login_action if login_action.startswith('/') else login_action

            # Handle hidden fields (such as CSRF tokens)
            hidden_inputs = login_form.find_all('input', {'type': 'hidden'})
            login_data = {
                'username': self.username,
                'password': self.password,
            }

            # Include hidden inputs in the login data
            for hidden_input in hidden_inputs:
                name = hidden_input.get('name')
                value = hidden_input.get('value', '')
                login_data[name] = value

            # Perform the login POST request
            response = self.session.post(login_url, data=login_data, headers=headers)
            if response.status_code != 200:
                self.get_logger().error(f"Login request failed. Status code: {response.status_code}")
                return False

            # Check for a successful login by looking for a specific element or text
            if "Logout" in response.text or "Device Status" in response.text:  # Adjust this based on actual response
                self.get_logger().info("Login successful")
                return True
            else:
                self.get_logger().error("Login failed")
                return False

        except Exception as e:
            self.get_logger().error(f"Login error: {e}")
            return False

    def start_timer(self):
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Timer started for GPS data retrieval")

    def timer_callback(self):
        try:
            # Fetch the GPS data page
            gps_url = self.modem_url + "status_gps.shtml"
            response = self.session.get(gps_url)

            if response.ok:
                soup = BeautifulSoup(response.text, 'html.parser')
                raw_message = self.extract_gps_message(soup)
                extracted_data = self.parse_gps_message(raw_message)

                if not extracted_data:
                    self.get_logger().warn("No valid GPS data to publish.")
                    return

                latitude, longitude, status = extracted_data
                if status != 1:  # Adjust the status code check as needed
                    self.get_logger().info(f"GPS not fixed yet. Status code: {status}")
                    return

                coords_msg = NavSatFix()
                coords_msg.header.frame_id = 'GPS_coords'
                coords_msg.header.stamp = self.get_clock().now().to_msg()
                coords_msg.latitude = latitude
                coords_msg.longitude = longitude
                self.coords_publisher.publish(coords_msg)
                self.get_logger().info(f"Published NavSatFix message with latitude: {latitude}, longitude: {longitude}")
            else:
                self.get_logger().error("Failed to retrieve GPS data")
        except Exception as e:
            self.get_logger().error(f"Error fetching GPS data: {e}")

    def extract_gps_message(self, soup):
        try:
            latitude = self.extract_value(soup, "Latitude")
            longitude = self.extract_value(soup, "Longitude")
            status = self.extract_value(soup, "Fix Status")  # Adjust field name as needed
            return f"{latitude},{longitude},{status}"
        except Exception as e:
            self.get_logger().error(f"Error extracting GPS message: {e}")
            return ""

    def parse_gps_message(self, raw_message):
        try:
            data_parts = raw_message.split(',')
            if len(data_parts) < 3:
                raise ValueError("Incomplete GPS data")

            latitude = float(data_parts[0]) if data_parts[0] else None
            longitude = float(data_parts[1]) if data_parts[1] else None
            status = int(data_parts[2])

            if latitude is None or longitude is None:
                raise ValueError("Missing latitude or longitude data")

            return [latitude, longitude, status]

        except ValueError as e:
            self.get_logger().error(f"BAD MSG: Could not convert data to float - {e}")
            return []

    def extract_value(self, soup, field_name):
        field = soup.find('td', text=field_name)
        if field and field.find_next('td'):
            return field.find_next('td').text.strip()
        return None

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSNode()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
