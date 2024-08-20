import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Int32MultiArray
import csv
import math
from datetime import datetime

class CSVtoPath:
    def __init__(self, filename, map_frame, logger):
        self.path = Path()
        self.type = Int32MultiArray()
        self.logger = logger
        self.load_csv(filename, map_frame)

    def load_csv(self, filename, map_frame):
        self.path.header.frame_id = map_frame
        self.path.header.stamp = self.get_current_time()

        self.type.data = []

        try:
            with open(filename, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)  # Skip the header row
                for row in reader:
                    if len(row) == 8:  # 行の長さが8つある場合のみ処理する
                        pose = PoseStamped()
                        pose.pose.position.x = float(row[1])
                        pose.pose.position.y = float(row[2])
                        pose.pose.position.z = float(row[3])

                        pose.pose.orientation.x = float(row[4])
                        pose.pose.orientation.y = float(row[5])
                        pose.pose.orientation.z = float(row[6])
                        pose.pose.orientation.w = float(row[7])

                        pose.header.frame_id = map_frame
                        pose.header.stamp = self.get_current_time()

                        self.type.data.append(int(row[0]))  # IDをtypeとして使う場合

                        self.path.poses.append(pose)
                    else:
                        self.logger.error(f"Unexpected row length: {len(row)}. Row content: {row}")

            self.path.header.frame_id = map_frame
            self.path.header.stamp = self.get_current_time()

        except FileNotFoundError:
            self.logger.error(f"Error! File {filename} cannot be opened")

    def get_current_time(self):
        now = datetime.now()
        return rclpy.time.Time(seconds=now.timestamp()).to_msg()

    def geometry_quat_getyaw(self, quaternion: Quaternion):
        """
        クオータニオンからヨー角を取得する関数。
        """
        _, _, yaw = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        return yaw

    def quaternion_to_euler(self, x, y, z, w):
        """
        クオータニオンからオイラー角（ロール、ピッチ、ヨー）を計算する関数。
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

class WPLoaderNode(Node):
    def __init__(self):
        super().__init__('wp_loader_node')

        self.declare_parameter('waypointfile', '')
        self.declare_parameter('map_frame', 'map')

        filename = self.get_parameter('waypointfile').get_parameter_value().string_value
        if filename == '':
            self.get_logger().error("Waypoint file not specified")
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        if filename:
            self.csv_to_path = CSVtoPath(filename, map_frame, self.get_logger())
        else:
            self.get_logger().error("No waypoint file provided")
            return

        self.path_pub = self.create_publisher(Path, 'waypoint/path', 10)
        self.type_pub = self.create_publisher(Int32MultiArray, 'waypoint/type', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.type_pub.publish(self.csv_to_path.type)
        self.path_pub.publish(self.csv_to_path.path)

def main(args=None):
    rclpy.init(args=args)

    node = WPLoaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
