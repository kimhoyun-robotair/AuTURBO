### Introduction

---

- 픽스호크 내부의 여러 시스템 (예: `EKF` 등)을 **모듈**이라고 부른다.
- **모듈 간의 통신**에서 사용하는 것이 **`uORB`** 메시지이다.
- 즉, `PX4` 내부에서 **IPC (Inter - Process Communication) / Inter - Thread Communication** 용도로 사용하기 위해서 정의된 메시지라고 보면 된다.

### uORB와 ROS2

---

![스크린샷, 2025-05-04 11-59-56.png](attachment:5cd6f88c-d8f7-4cb3-be70-01ca7e5e2bdf:스크린샷_2025-05-04_11-59-56.png)

- `uORB` 자체도 ROS2와 강력히 연동이 되도록 설계되어있다.
    - 이는 `uXRCE-DDS` 를 통해서 이루어지게 된다.
- `PX4` 내부에는 `uXRCE-DDS` 의 client가 자동으로 설치되어 있어서, `PX4` 펌웨어를 실행하게 되면 자동으로 client가 실행이 된다.
- `ROS2` 에서 `uXRCE-DDS` 의 agent를 켜게 되면, agent와 client가 연결되면서 `PX4` 와 `ROS2` 의 연결이 이루어지면서 `topic` 이 교환이 된다.
- `uORB` 내부에서는 `ROS2` 와 유사하게, 토픽을 사용해서 통신을 진행한다.
    - 이때 통신이 가능한 포트는 `UART` , `UDP` , `TCP` 등이 있다.
- `uORB` 를 통해서 `publish / subscribe` 되는 토픽은, `~/PX4-Autopilot/src/modules/uxrce_dds_client` 디렉터리의 `dds_topic.yaml` 파일로 정의된다.
    
    ```yaml
    #####
    #
    # This file maps all the topics that are to be used on the uXRCE-DDS client.
    #
    #####
    publications:
    
      - topic: /fmu/out/register_ext_component_reply
        type: px4_msgs::msg::RegisterExtComponentReply
    
      - topic: /fmu/out/arming_check_request
        type: px4_msgs::msg::ArmingCheckRequest
    
      - topic: /fmu/out/mode_completed
        type: px4_msgs::msg::ModeCompleted
    
      - topic: /fmu/out/battery_status
        type: px4_msgs::msg::BatteryStatus
    
      - topic: /fmu/out/collision_constraints
        type: px4_msgs::msg::CollisionConstraints
    
      - topic: /fmu/out/estimator_status_flags
        type: px4_msgs::msg::EstimatorStatusFlags
    
      - topic: /fmu/out/failsafe_flags
        type: px4_msgs::msg::FailsafeFlags
    
      - topic: /fmu/out/manual_control_setpoint
        type: px4_msgs::msg::ManualControlSetpoint
    
      - topic: /fmu/out/message_format_response
        type: px4_msgs::msg::MessageFormatResponse
    
      - topic: /fmu/out/position_setpoint_triplet
        type: px4_msgs::msg::PositionSetpointTriplet
    
      - topic: /fmu/out/sensor_combined
        type: px4_msgs::msg::SensorCombined
    
      - topic: /fmu/out/timesync_status
        type: px4_msgs::msg::TimesyncStatus
    
      # - topic: /fmu/out/vehicle_angular_velocity
      #   type: px4_msgs::msg::VehicleAngularVelocity
    
      - topic: /fmu/out/vehicle_land_detected
        type: px4_msgs::msg::VehicleLandDetected
    
      - topic: /fmu/out/vehicle_attitude
        type: px4_msgs::msg::VehicleAttitude
    
      - topic: /fmu/out/vehicle_control_mode
        type: px4_msgs::msg::VehicleControlMode
    
      - topic: /fmu/out/vehicle_command_ack
        type: px4_msgs::msg::VehicleCommandAck
    
      - topic: /fmu/out/vehicle_global_position
        type: px4_msgs::msg::VehicleGlobalPosition
    
      - topic: /fmu/out/vehicle_gps_position
        type: px4_msgs::msg::SensorGps
    
      - topic: /fmu/out/vehicle_local_position
        type: px4_msgs::msg::VehicleLocalPosition
    
      - topic: /fmu/out/vehicle_odometry
        type: px4_msgs::msg::VehicleOdometry
    
      - topic: /fmu/out/vehicle_status
        type: px4_msgs::msg::VehicleStatus
    
      - topic: /fmu/out/airspeed_validated
        type: px4_msgs::msg::AirspeedValidated
    
      - topic: /fmu/out/vtol_vehicle_status
        type: px4_msgs::msg::VtolVehicleStatus
    
      - topic: /fmu/out/home_position
        type: px4_msgs::msg::HomePosition
    
    # Create uORB::Publication
    subscriptions:
      - topic: /fmu/in/register_ext_component_request
        type: px4_msgs::msg::RegisterExtComponentRequest
    
      - topic: /fmu/in/unregister_ext_component
        type: px4_msgs::msg::UnregisterExtComponent
    
      - topic: /fmu/in/config_overrides_request
        type: px4_msgs::msg::ConfigOverrides
    
      - topic: /fmu/in/arming_check_reply
        type: px4_msgs::msg::ArmingCheckReply
    
      - topic: /fmu/in/message_format_request
        type: px4_msgs::msg::MessageFormatRequest
    
      - topic: /fmu/in/mode_completed
        type: px4_msgs::msg::ModeCompleted
    
      - topic: /fmu/in/config_control_setpoints
        type: px4_msgs::msg::VehicleControlMode
    
      - topic: /fmu/in/distance_sensor
        type: px4_msgs::msg::DistanceSensor
    
      - topic: /fmu/in/manual_control_input
        type: px4_msgs::msg::ManualControlSetpoint
    
      - topic: /fmu/in/offboard_control_mode
        type: px4_msgs::msg::OffboardControlMode
    
      - topic: /fmu/in/onboard_computer_status
        type: px4_msgs::msg::OnboardComputerStatus
    
      - topic: /fmu/in/obstacle_distance
        type: px4_msgs::msg::ObstacleDistance
    
      - topic: /fmu/in/sensor_optical_flow
        type: px4_msgs::msg::SensorOpticalFlow
    
      - topic: /fmu/in/goto_setpoint
        type: px4_msgs::msg::GotoSetpoint
    
      - topic: /fmu/in/telemetry_status
        type: px4_msgs::msg::TelemetryStatus
    
      - topic: /fmu/in/trajectory_setpoint
        type: px4_msgs::msg::TrajectorySetpoint
    
      - topic: /fmu/in/vehicle_attitude_setpoint
        type: px4_msgs::msg::VehicleAttitudeSetpoint
    
      - topic: /fmu/in/vehicle_mocap_odometry
        type: px4_msgs::msg::VehicleOdometry
    
      - topic: /fmu/in/vehicle_rates_setpoint
        type: px4_msgs::msg::VehicleRatesSetpoint
    
      - topic: /fmu/in/vehicle_visual_odometry
        type: px4_msgs::msg::VehicleOdometry
    
      - topic: /fmu/in/vehicle_command
        type: px4_msgs::msg::VehicleCommand
    
      - topic: /fmu/in/vehicle_command_mode_executor
        type: px4_msgs::msg::VehicleCommand
    
      - topic: /fmu/in/vehicle_thrust_setpoint
        type: px4_msgs::msg::VehicleThrustSetpoint
    
      - topic: /fmu/in/vehicle_torque_setpoint
        type: px4_msgs::msg::VehicleTorqueSetpoint
    
      - topic: /fmu/in/actuator_motors
        type: px4_msgs::msg::ActuatorMotors
    
      - topic: /fmu/in/actuator_servos
        type: px4_msgs::msg::ActuatorServos
    
      - topic: /fmu/in/aux_global_position
        type: px4_msgs::msg::VehicleGlobalPosition
    
    # Create uORB::PublicationMulti
    subscriptions_multi:
    
    ```
    
    - 각 토픽에 대한 설명은 PX4-Autopilot 공식 문서의 **uORB reference** 문서에 가게 되면 각 토픽들에 대한 자세한 설명과, 내부에 담겨져 있는 데이터에 대해서 확인할 수 있다.
- `uORB` 메시지는 2가지 종류가 있다.
    - **versioned message** : 픽스호크와 외부 통신을 담당하며, 펌웨어 버젼에 따라서 정의 및 내부 데이터 등이 일부 달라지게 된다.
    - **unversioned message** : 픽스호크 내부 통신을 담당하며, 펌웨어 버젼이랑 상관이 없다.
- 만약 `uORB` 토픽을 새롭게 개발하거나 하는 식으로 펌웨어 내부 프로그래밍을 하려면, C++을 통해서 수행해야 한다.

### uORB 토픽 예제

---

![스크린샷, 2025-05-04 14-19-40.png](attachment:e45ebcea-b216-469f-8d93-001ce228e030:스크린샷_2025-05-04_14-19-40.png)

```bash
listener sensor_accel 5
```

- 이런 식으로 명령어를 입력해서 `uORB` 토픽을 들을 수 있다.