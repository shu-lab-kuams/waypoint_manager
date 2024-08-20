import os
import csv
from math import sqrt, pow
from pynput import keyboard
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')

        # Parameters
        self.declare_parameter('use_keyboard', False)
        self.declare_parameter('use_joy', False)
        self.declare_parameter('save_button', 0)
        self.declare_parameter('quit_button', 1)
        self.declare_parameter('auto_record', False)
        self.declare_parameter('waypoint_interval', 1)
        self.use_keyboard = self.get_parameter('use_keyboard').value
        self.use_joy = self.get_parameter('use_joy').value
        self.save_button = self.get_parameter('save_button').value
        self.quit_button = self.get_parameter('quit_button').value
        self.auto_record = self.get_parameter('auto_record').value
        self.waypoint_interval = self.get_parameter('waypoint_interval').value

        # Subscription
        self.create_subscription(PoseWithCovarianceStamped, '/mcl_pose', self.pose_callback, 10)        

        # Publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/navigation_manager/waypoint_pose', 10) 

        # Keyboard save mode
        if self.use_keyboard:
            self.get_logger().info('Press "s" to save the current pose, "q" to quit and save to csv.')
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()
        
        # Joy save mode
        if self.use_joy:
            self.get_logger().info(f'Press "{self.save_button}" to save the current pose, "{self.quit_button}" to quit and save to csv.')
            self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Variables
        self.poses = []
        self.current_pose = None
        self.pose_id = 0
        self.prev_joy_buttons = []
        self.last_pose = PoseStamped()

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()

    def on_key_press(self, key):
        try:
            if key.char == 's':
                self.save_pose()
            elif key.char == 'q':
                self.save_poses_to_csv()
                self.destroy_node()
                rclpy.shutdown()
        except AttributeError:
            pass

    def joy_callback(self, msg):
        if not self.prev_joy_buttons:
            self.prev_joy_buttons = [0] * len(msg.buttons)

        # Check if save_button is pressed
        if msg.buttons[self.save_button] == 1 and self.prev_joy_buttons[self.save_button] == 0:
            self.save_pose()

        # Check if quit_button is pressed
        if msg.buttons[self.quit_button] == 1 and self.prev_joy_buttons[self.quit_button] == 0:
            self.save_poses_to_csv()
            self.destroy_node()
            rclpy.shutdown()

        # Update the previous button states
        self.prev_joy_buttons = msg.buttons

    def save_pose(self):
        if self.current_pose is not None:
            pose_data = [
                str(self.pose_id),
                str(self.current_pose.position.x),
                str(self.current_pose.position.y),
                str(self.current_pose.position.z),
                str(self.current_pose.orientation.x),
                str(self.current_pose.orientation.y),
                str(self.current_pose.orientation.z),
                str(self.current_pose.orientation.w),
            ]
            self.poses.append(pose_data)
            self.get_logger().info(f'saved! ID: {self.pose_id}')
            self.pose_id += 1

            # Publish the saved pose as PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"  # frame_idに注意
            pose_stamped.pose = self.current_pose
            self.pose_publisher.publish(pose_stamped)

            # Memorize the last recorded pose
            self.last_pose.header.stamp = self.get_clock().now().to_msg()
            self.last_pose.header.frame_id = "map"
            self.last_pose.pose = self.current_pose
        else:
            self.get_logger().warn('Warning: No pose received yet.')

    def save_poses_to_csv(self):
        filename = datetime.now().strftime('%Y%m%d%H%M%S') + '_waypoints.csv'
        path = os.path.join(os.path.curdir, 'src', 'waypoint_manager', 'waypoints', filename)  

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w'])
            writer.writerows(self.poses)

        self.get_logger().info(f'quited. Data saved to {path}!')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        dist_between_waypoints = None
        if self.auto_record:
            if self.last_pose is None:
                self.last_pose = self.current_pose
            else:
                dist_between_waypoints = sqrt(pow(self.current_pose.position.x - self.last_pose.pose.position.x, 2) + 
                                            pow(self.current_pose.position.y - self.last_pose.pose.position.y, 2) + 
                                            pow(self.current_pose.position.z - self.last_pose.pose.position.z, 2))
                if dist_between_waypoints >= self.waypoint_interval:
                    self.save_pose()
                    self.last_pose = self.current_pose


def main(args=None):
    rclpy.init(args=args)
    pose_recorder = WaypointSaver()

    try:
        rclpy.spin(pose_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
