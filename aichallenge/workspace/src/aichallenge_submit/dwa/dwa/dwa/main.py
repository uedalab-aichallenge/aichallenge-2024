import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Twist
from controller import Main_controller
from utils import load_obstacles
from config import DT, ROBOT_RADIUS
import math
import numpy as np

class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_node')
        self.controller = Main_controller()
        self.declare_parameter('robot_radius', ROBOT_RADIUS)
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value

        # Load obstacles
        self.obstacles = load_obstacles()

        # Publishers
        self.robot_marker_pub = self.create_publisher(Marker, 'robot_marker', 10)
        self.obstacles_marker_pub = self.create_publisher(MarkerArray, 'obstacles_marker', 10)
        self.path_marker_pub = self.create_publisher(Marker, 'planned_path', 10)
        self.opt_path_marker_pub = self.create_publisher(Marker, 'optimal_path', 10)
        self.multiple_paths_pub = self.create_publisher(MarkerArray, 'multiple_paths', 10)  # 新しいPublisher
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)  # 新規追加
        self.cmd_vel_pub = self.create_publisher(Twist, 'dwa_velocity', 10)  # 速度指令Publisherを追加

        self.timer = self.create_timer(DT, self.timer_callback)

        # Initialize markers
        self.init_markers()

        # Subscribers
        self.pose_sub = self.create_subscription(Pose, 'robot_pose', self.pose_callback, 10)
        self.velocity_sub = self.create_subscription(Twist, 'robot_velocity', self.velocity_callback, 10)

        # Initialize robot state
        self.current_pose = None
        self.current_velocity = None

    def init_markers(self):
        # Initialize robot marker
        self.robot_marker = Marker()
        self.robot_marker.header.frame_id = "map"
        self.robot_marker.ns = "robot"
        self.robot_marker.id = 0
        self.robot_marker.type = Marker.CYLINDER
        self.robot_marker.action = Marker.ADD
        self.robot_marker.scale.x = self.robot_radius * 2
        self.robot_marker.scale.y = self.robot_radius * 2
        self.robot_marker.scale.z = 0.5  # Height of the cylinder
        self.robot_marker.color.a = 1.0
        self.robot_marker.color.r = 0.0
        self.robot_marker.color.g = 1.0
        self.robot_marker.color.b = 0.0

        # Initialize obstacles marker array
        self.obstacles_marker = MarkerArray()
        for i, obs in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = obs.size * 2
            marker.scale.y = obs.size * 2
            marker.scale.z = 0.5
            marker.pose.position.x = obs.x
            marker.pose.position.y = obs.y
            marker.pose.position.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.obstacles_marker.markers.append(marker)

        # Initialize planned path marker
        self.path_marker = Marker()
        self.path_marker.header.frame_id = "map"
        self.path_marker.ns = "planned_path"
        self.path_marker.id = 0
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD
        self.path_marker.scale.x = 0.05
        self.path_marker.color.a = 1.0
        self.path_marker.color.r = 0.0
        self.path_marker.color.g = 0.0
        self.path_marker.color.b = 1.0

        # Initialize optimal path marker
        self.opt_path_marker = Marker()
        self.opt_path_marker.header.frame_id = "map"
        self.opt_path_marker.ns = "optimal_path"
        self.opt_path_marker.id = 1
        self.opt_path_marker.type = Marker.LINE_STRIP
        self.opt_path_marker.action = Marker.ADD
        self.opt_path_marker.scale.x = 0.1
        self.opt_path_marker.color.a = 1.0
        self.opt_path_marker.color.r = 1.0
        self.opt_path_marker.color.g = 1.0
        self.opt_path_marker.color.b = 0.0

        # Initialize local path marker
        self.local_path_marker = Marker()
        self.local_path_marker.header.frame_id = "map"
        self.local_path_marker.ns = "local_path"
        self.local_path_marker.id = 2
        self.local_path_marker.type = Marker.LINE_STRIP
        self.local_path_marker.action = Marker.ADD
        self.local_path_marker.scale.x = 0.05
        self.local_path_marker.color.a = 1.0
        self.local_path_marker.color.r = 0.0
        self.local_path_marker.color.g = 1.0
        self.local_path_marker.color.b = 1.0

        # Initialize goal marker 新規追加
        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = "map"
        self.goal_marker.ns = "goal"
        self.goal_marker.id = 0
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = 0.3
        self.goal_marker.scale.y = 0.3
        self.goal_marker.scale.z = 0.3
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 1.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 0.0
        self.goal_marker.pose.orientation.w = 1.0  # Neutral orientation

    def pose_callback(self, msg):
        self.current_pose = msg

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def timer_callback(self):
        if self.current_pose is None or self.current_velocity is None:
            self.get_logger().warn('まだロボットの状態データを受信していません。')
            return

        # コントローラーに現在の状態を設定
        self.controller.robot.x = self.current_pose.position.x
        self.controller.robot.y = self.current_pose.position.y
        # クォータニオンから角度を取得
        self.controller.robot.th = 2 * math.atan2(self.current_pose.orientation.z, self.current_pose.orientation.w)
        self.controller.robot.u_v = self.current_velocity.linear.x
        self.controller.robot.u_th = self.current_velocity.angular.z

        traj_x, traj_y, traj_th, traj_paths, traj_opt, traj_g_x, traj_g_y, course = self.controller.run_step(self.obstacles)

        # ロボットのマーカー更新
        self.robot_marker.header.stamp = self.get_clock().now().to_msg()
        self.robot_marker.pose.position.x = traj_x[-1]
        self.robot_marker.pose.position.y = traj_y[-1]
        self.robot_marker.pose.orientation.z = math.sin(traj_th[-1]/2)
        self.robot_marker.pose.orientation.w = math.cos(traj_th[-1]/2)
        self.robot_marker_pub.publish(self.robot_marker)

        # 障害物のパブリッシュ
        self.obstacles_marker_pub.publish(self.obstacles_marker)

        # 計画されたパスのマーカー更新
        self.path_marker.header.stamp = self.get_clock().now().to_msg()
        self.path_marker.points = [Point(x=x, y=y, z=0.0) for x, y in zip(traj_x, traj_y)]
        self.path_marker_pub.publish(self.path_marker)

        # 最適パスのマーカー更新
        if traj_opt:
            opt_path = traj_opt[-1]
            self.opt_path_marker.header.stamp = self.get_clock().now().to_msg()
            self.opt_path_marker.points = [Point(x=x, y=y, z=0.0) for x, y in zip(opt_path.x, opt_path.y)]
            self.opt_path_marker_pub.publish(self.opt_path_marker)

            self.goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_x = traj_g_x[-1]
            goal_y = traj_g_y[-1]
            self.goal_marker.pose.position.x = goal_x
            self.goal_marker.pose.position.y = goal_y
            self.goal_marker_pub.publish(self.goal_marker)

            # 最適パスに基づいて速度指令を生成
            cmd_vel_msg = Twist()
            # 最適パスから速度指令値を設定
            cmd_vel_msg.linear.x = self.controller.robot.u_v
            cmd_vel_msg.angular.z = self.controller.robot.u_th
            self.cmd_vel_pub.publish(cmd_vel_msg)

        # 複数のパスのパブリッシュ
        if traj_paths:
            multiple_paths_marker_array = MarkerArray()

            for idx, path in enumerate(traj_paths[-1]):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = f"multiple_paths_{idx}"
                marker.id = idx
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # パスの太さを調整
                # 色をランダムに設定（例）
                marker.color.a = 1.0
                marker.color.r = float(idx % 255) / 255.0
                marker.color.g = float((idx * 50) % 255) / 255.0
                marker.color.b = float((idx * 80) % 255.0)

                # パスのポイントを設定
                marker.points = [Point(x=x, y=y, z=0.0) for x, y in zip(path.x, path.y)]

                multiple_paths_marker_array.markers.append(marker)

            # MarkerArrayをパブリッシュ
            self.multiple_paths_pub.publish(multiple_paths_marker_array)

def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWANode()
    try:
        rclpy.spin(dwa_node)
    except KeyboardInterrupt:
        pass
    finally:
        dwa_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()