import geometry_msgs.msg
import rclpy
from pynput import keyboard

# Messages for instructions
msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   w
a  s  d

CTRL-C to quit
"""

# Define movement and turning
move = {
    'w': (1, 0, 0, 0),  # Move forward
    's': (-1, 0, 0, 0),  # Move backward
}
turn = {
    'a': (0, 0, 0, 1),  # Turn left
    'd': (0, 0, 0, -1),  # Turn right
}

# Set to track current keys pressed
pressed_keys = set()

# Gradual movement constants
max_linear_speed = 1.5  # Max forward/backward speed
linear_increment = 0.2  # Acceleration rate
linear_decrement = 0.3  # Deceleration rate

max_turn_speed = 0.7  # Max turn speed
turn_increment = 0.1  # Turning acceleration
turn_decrement = 0.1  # Turning deceleration rate

# Initial velocities
linear_velocity = 0.0
turn_velocity = 0.0


def on_press(key):
    """Callback function for key press events."""
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
    """Callback function for key release events."""
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


def update_velocity():
    """Update the linear and angular velocities based on key presses."""
    global linear_velocity, turn_velocity

    moving = False
    turning = False

    # Check if any movement keys are pressed
    if pressed_keys:
        for key in pressed_keys.copy():
            if key in move:
                x, _, _, _ = move[key]
                if x > 0:  # Moving forward
                    linear_velocity += linear_increment
                    if linear_velocity > max_linear_speed:  # Ensure max speed is not exceeded
                        linear_velocity = max_linear_speed
                elif x < 0:  # Moving backward
                    linear_velocity -= linear_increment
                    if linear_velocity < -max_linear_speed:  # Ensure min speed is not exceeded
                        linear_velocity = -max_linear_speed
                moving = True
            elif key in turn:
                _, _, _, th = turn[key]
                if th > 0:  # Turning left
                    turn_velocity += turn_increment
                    if turn_velocity > max_turn_speed:  # Ensure max turn speed is not exceeded
                        turn_velocity = max_turn_speed
                elif th < 0:  # Turning right
                    turn_velocity -= turn_increment
                    if turn_velocity < -max_turn_speed:  # Ensure min turn speed is not exceeded
                        turn_velocity = -max_turn_speed
                turning = True

    # Gradual deceleration
    if not moving:
        if linear_velocity > 0:
            linear_velocity = max(0, linear_velocity - linear_decrement)
        elif linear_velocity < 0:
            linear_velocity = min(0, linear_velocity + linear_decrement)

    # Gradual turning decay
    if not turning:
        if turn_velocity > 0:
            turn_velocity = max(0, turn_velocity - turn_decrement)
        elif turn_velocity < 0:
            turn_velocity = min(0, turn_velocity + turn_decrement)


def main():
    global linear_velocity, turn_velocity

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)

    try:
        print(msg)

        # Start the listener for keyboard events in a background thread
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        while rclpy.ok():
            update_velocity()

            # Create the Twist message to publish
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = linear_velocity*.3
            twist.angular.z = turn_velocity*.3

            # Publish the twist message
            pub.publish(twist)
            print(f'Direction: {twist.linear.x} Turn: {twist.angular.z}')

            # Reduce CPU load with a short sleep
            rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot on exit
        twist = geometry_msgs.msg.Twist()
        pub.publish(twist)

        # Stop the listener when the program exits
        listener.stop()


if __name__ == '__main__':
    main()
