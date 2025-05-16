#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry

class OdometryBridge(Node):
    def __init__(self):
        super().__init__('odometry_bridge_node')

        # PX4 -> VehicleOdometry 구독용 QoS 설정
        px4_qos = QoSProfile(depth=10)
        px4_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        px4_qos.durability   = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.px4_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            px4_qos
        )

        # nav_msgs/Odometry 퍼블리시용 QoS (keep_last=100)
        odom_qos = QoSProfile(depth=100)
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            odom_qos
        )

        self.get_logger().info('Odometry Bridge Started')

    def odometry_callback(self, msg: VehicleOdometry):
        odom = Odometry()

        # header 설정
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id    = 'odom'
        odom.child_frame_id     = 'base_link'

        # 위치 (float 캐스팅)
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = -float(msg.position[1])
        odom.pose.pose.position.z = -float(msg.position[2])

        # 방향 (PX4 쿼터니언 순서: w, x, y, z)
        odom.pose.pose.orientation.x = float(msg.q[1])
        odom.pose.pose.orientation.y = float(msg.q[2])
        odom.pose.pose.orientation.z = float(msg.q[3])
        odom.pose.pose.orientation.w = float(msg.q[0])

        # 속도 (선형 속도만, 역시 float 캐스팅)
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])

        # 퍼블리시
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
