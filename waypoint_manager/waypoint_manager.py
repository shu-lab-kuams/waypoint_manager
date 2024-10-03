import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        # Parameters
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('action_server_name', '')
        self.declare_parameter('loop_enable', False)
        self.declare_parameter('loop_count', 0)
        waypoints_csv = self.get_parameter('waypoints_csv').value
        action_server_name = self.get_parameter('action_server_name').value
        self.loop_enable = self.get_parameter('loop_enable').value
        self.loop_count = self.get_parameter('loop_count').value

        # Subscriber
        self.skip_flag_subscriber = self.create_subscription(Bool, '/skip_flag', self.skip_flag_callback, 10)

        # Publisher
        self.waypoint_id_publisher = self.create_publisher(Int32, '/waypoint_id', 10)
        
        # Action client
        self._action_client = ActionClient(self, NavigateToPose, action_server_name)

        # Load waypoints from CSV file
        self.waypoints_data = self.load_waypoints_from_csv(waypoints_csv)
        if not self.waypoints_data:
            self.get_logger().error("No waypoints loaded. Please check the CSV file.")
            return

        # Initialize variables
        self.current_waypoint_index = 0
        self.current_loop_count = 0
        self.skip_flag = False
        self._last_feedback_time = self.get_clock().now()
    
    def load_waypoints_from_csv(self, waypoints_csv):
        waypoints_data = []
        try:
            with open(waypoints_csv, mode='r') as file:
                reader = csv.reader(file)
                header = next(reader)

                for row in reader:
                    if len(row) < 9:
                        self.get_logger().error(f"Row in CSV file is too short: {row}")
                        continue
                    pose_stamped_msg = PoseStamped()
                    pose_stamped_msg.header.frame_id = 'map'
                    pose_stamped_msg.pose.position.x = float(row[1])
                    pose_stamped_msg.pose.position.y = float(row[2])
                    pose_stamped_msg.pose.position.z = float(row[3])
                    pose_stamped_msg.pose.orientation.x = float(row[4])
                    pose_stamped_msg.pose.orientation.y = float(row[5])
                    pose_stamped_msg.pose.orientation.z = float(row[6])
                    pose_stamped_msg.pose.orientation.w = float(row[7])

                    waypoint_data = {"pose": pose_stamped_msg,
                                     "skip_flag": int(row[8])
                                     }
                    waypoints_data.append(waypoint_data)
            
            self.get_logger().info(f"Loaded {len(waypoints_data)} waypoints from {waypoints_csv}.")
        except FileNotFoundError:
            self.get_logger().error(f"File {waypoints_csv} not found.")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
        
        return waypoints_data
    
    def skip_flag_callback(self, msg):
        if msg.data and self.current_waypoint_index < len(self.waypoints_data):
            self.skip_flag = True

    def send_goal(self, waypoint_data):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint_data["pose"]

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, waiting...')

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.publish_waypoint_id()

    def feedback_callback(self, feedback_msg):
        current_time = self.get_clock().now()
        if (current_time - self._last_feedback_time).nanoseconds >= 3e9:
            self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.distance_remaining))
            self._last_feedback_time = current_time

        # Check the skip flag during feedback, and immediately move to the next waypoint if flagged
        if self.skip_flag:
            if self.waypoints_data[self.current_waypoint_index]["skip_flag"] == 1:
                self.get_logger().info(f'Skip flag is True. Skipping waypoint {self.current_waypoint_index}.')
                self.skip_flag = False
                self.current_waypoint_index += 1
                self.advance_to_next_waypoint()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            return

        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.skip_flag = False
            self.current_waypoint_index += 1
            self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints_data):
            next_waypoint_data = self.waypoints_data[self.current_waypoint_index]
            next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
            self.send_goal(next_waypoint_data)
        else:
            if self.current_waypoint_index == len(self.waypoints_data):
                if self.loop_enable and self.current_loop_count < self.loop_count - 1:
                    self.current_waypoint_index = 0
                    self.current_loop_count += 1
                    self.get_logger().info(f'Starting loop {self.current_loop_count + 1}/{self.loop_count}.')
                    self.advance_to_next_waypoint()
                else:
                    self.get_logger().info('Arrived at the last waypoint. Navigation complete.')
            self.skip_flag = False

    def publish_waypoint_id(self):
        if self.current_waypoint_index < len(self.waypoints_data):
            next_waypoint_id = self.current_waypoint_index
            self.waypoint_id_publisher.publish(Int32(data=next_waypoint_id))

    def run(self):
        if self.waypoints_data:
            self.advance_to_next_waypoint()

def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointManager()
    try:
        waypoint_manager.run()
        rclpy.spin(waypoint_manager)
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt, shutting down...")
    finally:
        waypoint_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
