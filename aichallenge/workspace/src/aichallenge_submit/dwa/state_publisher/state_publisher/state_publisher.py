import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from .robot import Robot
from .config import DT
import math

class StatePublisherNode(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.pose_pub = self.create_publisher(Pose, 'robot_pose', 10)
        self.velocity_pub = self.create_publisher(Twist, 'robot_velocity', 10)
        self.velocity_sub = self.create_subscription(Twist, 'dwa_velocity', self.velocity_callback, 10)
        self.timer = self.create_timer(DT, self.timer_callback)  # 定期的に状態を更新
        self.robot = Robot()

    def velocity_callback(self, msg):
        # 速度指令を受け取った際に状態を更新
        u_v = msg.linear.x
        u_th = msg.angular.z
        self.robot.u_v = u_v
        self.robot.u_th = u_th

    def timer_callback(self):
        # 状態の更新
        self.update_state()

        # Poseメッセージの作成
        pose_msg = Pose()
        pose_msg.position.x = self.robot.x
        pose_msg.position.y = self.robot.y
        pose_msg.position.z = 0.0
        # 角度をクォータニオンに変換
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = math.sin(self.robot.th / 2)
        pose_msg.orientation.w = math.cos(self.robot.th / 2)

        # Twistメッセージの作成
        twist_msg = Twist()
        twist_msg.linear.x = self.robot.u_v
        twist_msg.angular.z = self.robot.u_th

        # メッセージをパブリッシュ
        self.pose_pub.publish(pose_msg)
        self.velocity_pub.publish(twist_msg)
        # 必要に応じて他のメッセージもパブリッシュ可能

    def update_state(self):
        dt = DT
        u_v = self.robot.u_v
        u_th = self.robot.u_th
        th = self.robot.th

        next_x = u_v * math.cos(th) * dt + self.robot.x
        next_y = u_v * math.sin(th) * dt + self.robot.y
        next_th = u_th * dt + th

        self.robot.traj_x.append(next_x)
        self.robot.traj_y.append(next_y)
        self.robot.traj_th.append(next_th)

        self.robot.x = next_x
        self.robot.y = next_y
        self.robot.th = next_th

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()