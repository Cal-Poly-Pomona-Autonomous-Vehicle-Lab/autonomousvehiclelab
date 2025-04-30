import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import serial

class Velocity:
    def __init__(self, brake=0, mode=1, linear=0.0, angular=0.0):
        self.brake = brake
        self.mode = mode
        self.linear = linear
        self.angular = angular

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial_conn = None
        self.velocity = Velocity()

        self.connect_serial()

        self.create_subscription(
            TwistStamped,
            '/cmd_vel_out',
            self.twist_callback,
            10
        )

        self.get_logger().info("Motor control node started. Listening to /cmd_vel_out...")

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to serial port {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_conn = None

    def send_serial_command(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn("Serial connection is not open.")
            return

        command = f"B:{self.velocity.brake},M:{self.velocity.mode},L:{self.velocity.linear:.2f},A:{self.velocity.angular:.2f}\n"

        try:
            self.serial_conn.write(command.encode())
            self.serial_conn.flush()
            self.get_logger().info(f"Sent command: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending serial command: {e}")

    def twist_callback(self, msg: TwistStamped):
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z

        # Check for true neutral (both velocities essentially zero)
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            mode = 1  # Neutral
            linear = 0.0
            angular = 0.0
        else:
            # Clamp linear and angular to allowed ranges
            linear = max(min(linear, 1.7), -1.7)
            angular = max(min(angular, 0.7), -0.7)

            # Determine mode based on linear direction
            if linear > 0:
                mode = 1  # Drive
            elif linear < 0:
                mode = 3  # Reverse
            else:
                mode = 1  # Still neutral fallback

        self.velocity = Velocity(
            brake=0,
            mode=mode,
            linear=abs(linear),  # Send magnitude, mode determines direction
            angular=-angular
        )

        self.send_serial_command()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
