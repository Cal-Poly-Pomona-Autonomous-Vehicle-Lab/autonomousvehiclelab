
import sys
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

Up arrow: increase speed
Down arrow: decrease speed
CTRL-C to quit
"""

move = {
    'w': (1, 0, 0, 0),  # Move forward
    's': (-1, 0, 0, 0),  # Move backward
}
turn = {
    'a': (0, 0, 0, 1),  # Turn left
    'd': (0, 0, 0, -1),  # Turn right
}
# Use arrow keys for speed adjustment
speedBindings = {
    'up': (1.1, 1),  # Up arrow for increasing speed
    'down': (0.9, 1),  # Down arrow for decreasing speed
}

# Set to track current keys pressed
pressed_keys = set()


def on_press(key):
    """Callback function for key press events."""
    try:
        # Handle regular keys (e.g., 'a', 'w', etc.)
        if hasattr(key, 'char'):
            pressed_keys.add(key.char)
        # Handle special keys (e.g., arrow keys)
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
        # Handle regular keys
        if hasattr(key, 'char'):
            pressed_keys.discard(key.char)
        # Handle special keys
        else:
            if key == keyboard.Key.up:
                pressed_keys.discard('up')
            elif key == keyboard.Key.down:
                pressed_keys.discard('down')

        # Stop listener if 'esc' is pressed
        if key == keyboard.Key.esc:
            return False
    except KeyError:
        pass


def main():
    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)

    speed = 0.3
    turn_speed = 1.0  # renamed to avoid confusion with turn dictionary
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    
    try:
        print(msg)
        
        # Start the listener for keyboard events in a background thread
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        while rclpy.ok():
            x = 0.0  # Reset movement each loop
            y = 0.0
            z = 0.0
            th = 0.0

            # Check if any movement keys are pressed
            if pressed_keys:
                for key in pressed_keys.copy():
                    if key in move:
                        move_x, move_y, move_z, move_th = move[key]
                        x += move_x  # Add movement to x
                        y += move_y
                        z += move_z
                        th += move_th  # Not expected for movement keys but keeps structure
                    elif key in turn:
                        turn_x, turn_y, turn_z, turn_th = turn[key]
                        x += turn_x  # Likely 0
                        y += turn_y  # Likely 0
                        z += turn_z  # Likely 0
                        th += turn_th  # Add turning to th
                    elif key in speedBindings:
                        speed = speed * speedBindings[key][0]
                        if status == 14:
                            print(msg)
                        status = (status + 1) % 15
            else:
                # Stop movement if no keys are pressed
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0

            # Create the Twist message to publish
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = float(x) * speed
            twist.linear.y = float(y) * speed
            twist.linear.z = float(z) * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(th) * turn_speed  # Ensure it's a float
            pub.publish(twist)
            print(f'Direction: {twist.linear.x},{twist.linear.y},{twist.linear.z}\nTurn: {twist.angular.z}')
            
            # Reduce CPU load with a short sleep
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

        # Stop the listener when the program exits
        listener.stop()


if __name__ == '__main__':
    main()
23235235