import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, jsonify, Response, stream_with_context
from flask_socketio import SocketIO
from threading import Thread, Lock, Event


# Flask app setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', ping_interval=10, ping_timeout=5)
shutdown_flag = Event() 

# ROS2 Node Class
class Server2Node(Node):
    def __init__(self):
        super().__init__('server2')
        self.publisher_ = self.create_publisher(Twist, '/keyboard/cmd_vel', 10)
        self.bridge = CvBridge()
        self.camera_lock = Lock()

        # Movement Bindings
        self.move_bindings = {
            'w': (1, 0, 0, 0),
            's': (-1, 0, 0, 0),
            'a': (0, 0, 0, 1),
            'd': (0, 0, 0, -1),
        }

        self.speed = 0.3
        self.turn_speed = 1.0

        # ROS Camera Subscription
        self.subscriber = self.create_subscription(Image, '/video/front_camera', self.camera_callback, 10)

        # For MJPEG Streaming
        self.frame_generator = self.generate_frames()

    def camera_callback(self, msg):
        """
        Process incoming ROS image messages and convert to JPEG.
        """
        with self.camera_lock:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            _, jpeg = cv2.imencode('.jpg', frame)
            self.current_frame = jpeg.tobytes()

    def generate_frames(self):
        """
        Generator to stream frames in MJPEG format.
        """
        while True:
            Event().wait(0.03)  # Target ~30 FPS
            with self.camera_lock:
                if self.current_frame is not None:
                    yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + self.current_frame + b'\r\n')


    def publish_twist(self, key, action):
        """
        Handle keyboard events to publish Twist messages.
        """
        if key in self.move_bindings:
            x, y, z, th = self.move_bindings[key]
            if action == 'press':
                twist = Twist()
                twist.linear.x = x * self.speed
                twist.angular.z = th * self.turn_speed
                self.publisher_.publish(twist)
            elif action == 'release':
                twist = Twist()  # Stop motion
                self.publisher_.publish(twist)


server2_node = None

# Flask Endpoints
@app.route('/')
def index():
    return jsonify({"status": "Server2 is running"})

@app.route('/camera/stream')
def camera_stream():
    """
    Endpoint to stream MJPEG frames.
    """
    return Response(server2_node.frame_generator, content_type='multipart/x-mixed-replace; boundary=frame')


# SocketIO Handlers
@socketio.on('keypress')
def handle_keypress(data):
    key = data.get('key')
    action = data.get('action')  # 'press' or 'release'
    if key and action:
        server2_node.update_keys(key, action)  # Update pressed keys
        socketio.emit('keypress_ack', {'status': 'received'}, broadcast=False)  # Optional ACK for debugging
        print(f"Received keypress: {key}, action: {action}")



# ROS2 Node Runner
def run_ros2_node():
    global server2_node
    rclpy.init()
    server2_node = Server2Node()

    try:
        rclpy.spin(server2_node)
    except Exception as e:
        print(e)
    finally:
        server2_node.destroy_node()
        rclpy.shutdown()


# Main Application
def main():
    print("Starting Server2...")

    # Run Flask server in a separate thread
    flask_thread = Thread(target=socketio.run, kwargs={'app': app, 'host': '0.0.0.0', 'port': 5002})
    flask_thread.start()

    # Run ROS2 node in the main thread
    run_ros2_node()


if __name__ == '__main__':
    main()
