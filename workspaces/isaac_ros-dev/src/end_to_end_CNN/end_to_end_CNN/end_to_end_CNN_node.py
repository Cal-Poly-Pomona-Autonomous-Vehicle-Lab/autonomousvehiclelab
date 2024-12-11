import cv2
import rclpy
import torch 
import numpy as np  
import torchvision.transforms as transforms

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.node import Node

from end_to_end_CNN import end_to_end_CNN_model

model_dir = "/home/agxorin1/autonomousvehiclelab/workspaces/isaac_ros-dev/src/end_to_end_CNN/models/checkpoints/model.pth"
transform = transforms.Compose([
    transforms.ToPILImage(), 
    transforms.Resize((66, 200)),
    transforms.ToTensor(), 
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
])

class end_to_end_CNN_node(Node):
    def __init__(self):
        super().__init__("End_to_End_CNN")
        # Initialize Objects and variables
        self.drive_twist_ = Twist() 
        self.bridge_ = CvBridge()

        self.front_image_ = None
        self.device_ = torch.device('cpu')

        # Subscribe to the front camera 
        self.front_image_subscriber_ = self.create_subscription(Image, '/video/front_camera', self.front_image_callback, 2)

        # Create a publisher for the steering commands
        self.steering_ = self.create_publisher(Twist, '/twist_mux/cmd_vel', 2)
        self.steering_timer_ = self.create_timer(.2, self.steering_publisher)

        # Init the End to End Convolutional Neural Network
        self.end_to_end_CNN_ = end_to_end_CNN_model.SelfDrivingCarCNN()

        if torch.cuda.is_available():
            self.device_ = torch.device('cuda:0')

        print('using device:', self.device_)
        
        self.end_to_end_CNN_.to(self.device_)

        self.end_to_end_CNN_.load_state_dict(torch.load(model_dir))
        self.end_to_end_CNN_.eval()

    def front_image_callback(self, msg: Image) -> None: 
        ''' 
            Front Image is passed via a Subscriber (Callback) and passed to class variable 

            @params 
                msg: Frame
            @return
                None
        ''' 
        img = self.bridge_.imgmsg_to_cv2(msg)
        self.front_image_ = img

    def steering_publisher(self) -> None:
        ''' 
            Model is given a frame to process and returns the steering value + speed to the publisher

            @params 
                None 
            @return 
                None
        ''' 

        if self.front_image_ is None: 
            self.get_logger().error(f"Error when attempting to get Front Camera")
            return

        front_frame = self.front_image_ 
        front_frame = cv2.cvtColor(front_frame, cv2.COLOR_RGB2YUV)

        steering_value = None 
        with torch.no_grad(): 
            front_frame_tensor = transform(front_frame)
            front_frame_tensor = torch.unsqueeze(front_frame_tensor, 1)
            front_frame_tensor = front_frame_tensor.permute(1, 0, 2, 3)

            steering_value = self.end_to_end_CNN_(front_frame_tensor).squeeze() 
        
        self.get_logger().info("Steering value: " + str(steering_value.cpu().numpy().item()))

        self.drive_twist_.angular.z = steering_value.cpu().numpy().item()
        self.drive_twist_.linear.x = .25

        self.steering_.publish(self.drive_twist_)
        
        

def main(args = None):
    rclpy.init(args=args)

    end_to_end_CNN = end_to_end_CNN_node()
    rclpy.spin(end_to_end_CNN)
    end_to_end_CNN.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()