###############################################################
# WGS84 좌표 기반으로 장거리 멀티로터 자율비행 (~5km)을 구현하는 코드 #
###############################################################
import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition

# from tf_transformations import euler_from_quaternion, quaternion_from_euler

EARTH_RADIUS = 6371000.0  # 지구 반경 (미터)

class OffboardControl(Node):
    def __init__(self):
        """ Initialize Node """
        """ Setting QoS Profile and Topic Pub / Sub """
        super().__init__("Offboard_control")

        # PX4와의 통신을 위한 QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # config 파일 활용을 위한 파라미터 선언 및 디폴트 값 지정
        self.declare_parameter("topic_offboard_control_mode", "/fmu/in/offboard_control_mode")
        self.declare_parameter("topic_vehicle_attitude_setpoint", "/fmu/in/vehicle_attitude_setpoint")
        self.declare_parameter("topic_trajectory_setpoint", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("topic_vehicle_command", "/fmu/in/vehicle_command")
        self.declare_parameter("topic_vehicle_local_position", "/fmu/out/vehicle_local_position")
        self.declare_parameter("topic_vehicle_attitude", "/fmu/out/vehicle_attitude")
        self.declare_parameter("topic_vehicle_status", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("topic_vehicle_global_position", "/fmu/out/vehicle_global_position")

        # 파라미터 파일을 읽어오기
        topic_offboard_control_mode = self.get_parameter("topic_offboard_control_mode").value
        topic_vehicle_attitude_setpoint = self.get_parameter("topic_vehicle_attitude_setpoint").value
        topic_trajectory_setpoint = self.get_parameter("topic_trajectory_setpoint").value
        topic_vehicle_command = self.get_parameter("topic_vehicle_command").value
        topic_vehicle_local_position = self.get_parameter("topic_vehicle_local_position").value
        topic_vehicle_attitude = self.get_parameter("topic_vehicle_attitude").value
        topic_vehicle_status = self.get_parameter("topic_vehicle_status").value
        topic_vehicle_global_position = self.get_parameter("topic_vehicle_global_position").value

        # Topic Publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, topic_offboard_control_mode, qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, topic_vehicle_attitude_setpoint, qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, topic_trajectory_setpoint, qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, topic_vehicle_command, qos_profile)

        # Topic Subscriber
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, topic_vehicle_local_position, self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, topic_vehicle_attitude, self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, topic_vehicle_status, self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, topic_vehicle_global_position, self.vehicle_global_position_callback, qos_profile)


        # WGS84에서 NED로 변환된 Local 경로점들
        self.ned_waypoints = {
            "WP0": {"x":0, "y":0, "z": 0},
            "WP1": {"x":0, "y":0, "z": -20},
            "WP2": {"x":100, "y":50, "z": -20},
            "WP3": {"x":140, "y":75, "z": -10},
            "WP4": {"x":300, "y":25, "z": -30},
            "WP5": {"x":175, "y":-50, "z": -15},
            "WP6": {"x":50, "y":-15, "z": -20},
            "WP7": {"x":0, "y":0, "z": -5},
        }

        self.pre_calculate_waypoint_and_initialize_variable()
        self.get_logger().info("Initialization complete.")

####################### Initialize and Coordinate Conversion Functions #######################
    def pre_calculate_waypoint_and_initialize_variable(self):
        """ Pre-calculate waypoints and ready vehicle """
        """ Calculate WGS84 to NED Coordinate for using in px4-msgs"""
        """ And Initialize various variables, Setting Timer Callback"""
        # 각종 변수들 초기화 및 선언
        self.state = "NOT_READY"
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.global_dist = 0.0
        self.takeoff_height = self.ned_waypoints["WP1"]["z"]
        self.waypoint_reach_or_not = False

        self.declare_parameter("threshold_range", 0.5)
        self.threshold_range = self.get_parameter("threshold_range").value

        # timer 콜백 함수 설정   
        self.timer = self.create_timer(0.01, self.timer_callback)


    def get_distance_between_ned(self, x_now, y_now, z_now, x_next, y_next, z_next):
        # NED 좌표로 주어진 두 점 사이의 거리 구하기
        dx = x_next - x_now
        dy = y_next - y_now
        dz = z_next - z_now

        dist_xy = math.sqrt(dx*dx+dy*dy)
        dist_z = abs(dz)
        total_dist = math.sqrt(dist_xy*dist_xy + dist_z*dist_z)

        return total_dist, dist_xy, dist_z

    def is_waypoint_reached(self, waypoint):
        # NED 좌표를 기반으로 계산했을 때 경로점에 도착했는지 여부를 판단하기
        total_dist, dist_xy, dist_z = self.get_distance_between_ned(
            self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z,
            waypoint["x"], waypoint["y"], waypoint["z"])
        
        if dist_xy <= self.threshold_range and dist_z <= self.threshold_range:
            return True
        else:
            return False

    def get_attitude(self, current_wp, next_wp):
        # NED 좌표로 주어진 두 점 사이의 yaw 값 계산하기 (제어에 사용)
        dx = next_wp["x"] - current_wp["x"]
        dy = next_wp["y"] - current_wp["y"]

        # arctan2는 (dy, dx)를 인자로 받아 라디안 단위의 각도를 반환.
        yaw_rad = math.atan2(dy, dx)

        # 라디안을 도 단위로 변환
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg

######################## Callback Functions #######################
                # Functions for subscribing messages
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback for vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback for vehicle attitude."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status."""
        self.vehicle_status = vehicle_status

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback for vehicle global position."""
        self.vehicle_global_position = vehicle_global_position

########################## Command Functions #######################
                    # Functions for Offboard Control
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Send a command to engage offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Send a command to land the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_heartbeat_ob_pos_sp(self):
        """Publish heartbeat message for offboard control."""
        msg = OffboardControlMode()
        msg.position    = True
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d: float):
        """Publish position setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = np.clip(np.deg2rad(yaw_d), -np.pi, np.pi)
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
############################## Timer Callback Function #######################
# Functions for Finite State Machine and Sequential Offboard Control
    def timer_callback(self):
        # Take-off 상태 처리
        if self.state == "NOT_READY":
            self.publish_heartbeat_ob_pos_sp()
            if self.offboard_setpoint_counter < 10:
                self.offboard_setpoint_counter += 1
            self.offboard_setpoint_counter %= 11
            if self.offboard_setpoint_counter < 5:
                self.pos_x = 0.0
                self.pos_y = 0.0
                self.pos_z = self.takeoff_height
                # self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
                self.pos_yaw = self.get_attitude(self.ned_waypoints["WP0"], self.ned_waypoints["WP2"])
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter == 9:
                self.arm()

            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP1"])
            if (self.waypoint_reach_or_not == True and
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.state = "TAKEOFF"
                self.get_logger().info("Take off!")

        elif self.state == "TAKEOFF":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP0"], self.ned_waypoints["WP2"])
            self.pos_x = self.ned_waypoints["WP2"]["x"]
            self.pos_y = self.ned_waypoints["WP2"]["y"]
            self.pos_z = self.ned_waypoints["WP2"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP2"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Takeoff to WP2")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_2"
                self.get_logger().info("WP2")

        elif self.state == "WAYPOINT_2":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP2"], self.ned_waypoints["WP3"])
            self.pos_x = self.ned_waypoints["WP3"]["x"]
            self.pos_y = self.ned_waypoints["WP3"]["y"]
            self.pos_z = self.ned_waypoints["WP3"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP3"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_3"
                self.get_logger().info("WP3!!")

        elif self.state == "WAYPOINT_3":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP3"], self.ned_waypoints["WP4"])
            self.pos_x = self.ned_waypoints["WP4"]["x"]
            self.pos_y = self.ned_waypoints["WP4"]["y"]
            self.pos_z = self.ned_waypoints["WP4"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP4"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP4")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_4"
                self.get_logger().info("WP4!!")

        elif self.state == "WAYPOINT_4":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP4"], self.ned_waypoints["WP5"])
            self.pos_x = self.ned_waypoints["WP5"]["x"]
            self.pos_y = self.ned_waypoints["WP5"]["y"]
            self.pos_z = self.ned_waypoints["WP5"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP5"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP5")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_5"
                self.get_logger().info("WP5!!")

        elif self.state == "WAYPOINT_5":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP5"], self.ned_waypoints["WP6"])
            self.pos_x = self.ned_waypoints["WP6"]["x"]
            self.pos_y = self.ned_waypoints["WP6"]["y"]
            self.pos_z = self.ned_waypoints["WP6"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP6"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP6")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_6"
                self.get_logger().info("WP6!!")

        elif self.state == "WAYPOINT_6":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP6"], self.ned_waypoints["WP7"])
            self.pos_x = self.ned_waypoints["WP7"]["x"]
            self.pos_y = self.ned_waypoints["WP7"]["y"]
            self.pos_z = self.ned_waypoints["WP7"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP7"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP7")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_7"
                self.get_logger().info("WP7!!")
    
        elif self.state == "WAYPOINT_7":
            self.publish_heartbeat_ob_pos_sp()
            self.land()
            self.get_logger().info("Landing...")
            rclpy.shutdown()

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)