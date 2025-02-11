import sys
import geometry_msgs.msg
import rclpy
from pynput import keyboard

# Instructions
msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   w
a  s  d

Up arrow: increase speed
Down arrow: decrease speed
CTRL-C to quit
"""
# Track pressed keys
pressed_keys = set()

# Constants for gradual movement and turning
max_linear_speed = 1.5  # Max forward/backward speed
linear_increment = 0.2  # Acceleration rate
linear_decrement = 0.3  # Deceleration rate

max_turn_speed = 0.7  # Max turn speed
turn_increment = 0.1  # Turning acceleration
turn_decrement = 0.1  # Turning deceleration

# Initial velocities
linear_velocity = 0.0
turn_velocity = 0.0


def on_press(key):
    """Callback for key press."""
    try:
        if hasattr(key, 'char'):
            pressed_keys.add(key.char)
        else:
            if key == keyboard.Key.up:
                pressed_keys.add('up')
            elif key == keyboard.Key.down:
                pressed_keys.add('down')
    except AttributeError:
        pass


def on_release(key):
    """Callback for key release."""
    try:
        if hasattr(key, 'char'):
            pressed_keys.discard(key.char)
        else:
            if key == keyboard.Key.up:
                pressed_keys.discard('up')
            elif key == keyboard.Key.down:
                pressed_keys.discard('down')

        if key == keyboard.Key.esc:
            return False
    except KeyError:
        pass


def main():
    global linear_velocity, turn_velocity

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)

    speed = 0.3
    y = 0.0
    z = 0.0
    status = 0.0

    try:
        print(msg)

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        while rclpy.ok():
            moving = False  # Track if we are actively moving forward/backward
            turning = False  # Track if we are actively turning

            # Check for pressed keys
            if pressed_keys:
                for key in pressed_keys.copy():
                    # Forward motion
                    if key == 'w':
                        linear_velocity += linear_increment
                        if linear_velocity > max_linear_speed:
                            linear_velocity = max_linear_speed  # Cap max forward speed
                        moving = True  # Mark that we are moving

                    # Backward motion
                    elif key == 's':
                        linear_velocity -= linear_increment
                        if linear_velocity < -max_linear_speed:
                            linear_velocity = -max_linear_speed  # Cap max backward speed
                        moving = True  # Mark that we are moving

                    # Gradual left turn
                    elif key == 'a':
                        turn_velocity += turn_increment
                        if turn_velocity < max_turn_speed:
                            turn_velocity = max_turn_speed  # Cap left turn
                        turning = True  # Mark that we are turning

                    # Gradual right turn
                    elif key == 'd':
                        turn_velocity -= turn_increment
                        if turn_velocity > -max_turn_speed:
                            turn_velocity = -max_turn_speed  # Cap right turn
                        turning = True  # Mark that we are turning

            # Apply gradual deceleration when no movement keys are pressed
            if not moving:
                if linear_velocity > 0:
                    linear_velocity -= linear_decrement
                    if linear_velocity < 0:
                        linear_velocity = 0  # Prevent overshoot
                elif linear_velocity < 0:
                    linear_velocity += linear_decrement
                    if linear_velocity > 0:
                        linear_velocity = 0  # Prevent overshoot

            # Apply gradual turn decay when no turn keys are pressed
            if not turning:
                if turn_velocity > 0:
                    turn_velocity -= turn_decrement
                    if turn_velocity < 0:
                        turn_velocity = 0  # Prevent overshoot
                elif turn_velocity < 0:
                    turn_velocity += turn_decrement
                    if turn_velocity > 0:
                        turn_velocity = 0  # Prevent overshoot

            if turn_velocity > max_turn_speed:
                turn_velocity = max_turn_speed
            elif turn_velocity < -max_turn_speed:
                turn_velocity = -max_turn_speed
                
            # Create Twist message
            # Create Twist message
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = float(linear_velocity)  # Apply gradual movement
            twist.linear.y = float(y) * speed
            twist.linear.z = float(z) * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0

            # Apply the same turning logic for both forward and reverse movement
            twist.angular.z = float(turn_velocity)
            # Adjust the turning direction when moving backward
            if linear_velocity > 0:  # Moving forward
                twist.angular.z = float(turn_velocity)
            elif linear_velocity < 0:  # Moving backward
                twist.angular.z = float(-turn_velocity)  # Reverse the direction of turning

            # Apply the correct turning direction based on user input
            pub.publish(twist)
            print(f'Speed: {twist.linear.x:.2f} | Turn: {twist.angular.z:.2f}')

            rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot on exit
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        listener.stop()


if __name__ == '__main__':
    main()
