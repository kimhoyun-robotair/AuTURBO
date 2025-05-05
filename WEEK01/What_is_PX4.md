# What is PX4?

## PX4란?

---

- 회전익 드론, 고정익 드론, VTOL, 틸트로터, USV, UGV, 로버 등 다양한 무인 이동체에 사용 가능한 멀티 플랫폼 (비행) 제어 스택
- 하드웨어 및 소프트웨어 일체형으로 개발
- 2011년 스위스 ETH 취리히에서 개발이 시작되었음
- jMAVSim, Airsim, Gazebo 등 다양한 시뮬레이션 툴로 SILS 기능도 지원함
- 오픈소스 기반이므로 매우 광범위한 커뮤니티가 형성되어 있고, 커뮤니티 기반 개발이 주도 중에 있음
- 실시간 운영체제인 NuttX 베이스이며, POSIX 호환 환경을 지원함

## PX4 아키텍쳐 개요

---

### 하드웨어 추상화 계층

- NuttX, Linux/POSIX 환경에서 동일한 드라이버 API 제공
- 센서(I2C, SPI, UART) 및 제어 출력(PWM, DShot) 드라이버 포함

### 미들웨어 (uORB 메시지 버스)

- 게시/구독(pub/sub) 구조로 모듈 간 데이터 교환
- uORB 토픽 단위로 센서 데이터, 상태, 명령을 주고받음
- **uORB**란?
    - ORB는 기본적으로 분산 컴퓨팅 환경에서 서로 다른 컴퓨터들 간의 통신 환경을 구축해주는 **미들웨어(Middleware)**의 일종
    - **uORB**는 PX4를 위해서, 더 가볍고 빠르게 설계된 ORB이다.

### 상위 어플리케이션 계층

- **모듈** 간의 uORB 토픽의 Publish와 Subscribe를 통해서 다양한 기능을 구현함
- 또한, **QGC, ROS2**와 같은 다른 어플리케이션이나 미들웨어와 연동해서 보다 복잡한 제어 혹은 미션 수행이 가능하도록 구성됨
- 이를 위해서 다른 어플리케이션 혹은 미들웨어에 uORB 메시지 bridging이 가능하도록 설계되어있음 (예: `uXRCE-DDS` 를 통한 ROS2와 PX4 간의 통신)

## PX4 주요 기능

---

- **비행체 유형 지원**
    - 멀티콥터(quad, hexa, octo)
    - 고정익(fixed-wing)
    - VTOL(tilt-rotor, tilt-wing, tail-sitter, standard VTOL)
- **비행 모드**
    - Manual, Stabilized, Acro(자이로 기반 자세 제어)
    - Altitude, Position, Offboard(외부 명령)
    - RTL(Return To Launch), Pause, Mission 등
- **안전·Failsafe**
    - 저전압, 통신 단절, GPS 오류 시 자동 착륙/RTL
    - 지오펜싱(지정된 영역 밖으로 이탈 방지)
- **미션 플래닝**
    - Waypoint, Survey(그리드·패턴), Follow-me
    - QGroundControl 연동, Mavlink 미션 동기화
- **고급 기능**
    - 장애물 회피(Obstacle Avoidance) 모듈 실험적 지원
    - 블루투스, Wi-Fi, LTE 모듈과 연동

## 비행 모드와 제어 루프

---

1. **자세 제어 (Attitude Control)**
    - 각 축(Roll, Pitch, Yaw)에 대한 PID 제어기
    - 센서(가속도계·자이로) 피드백 반영
2. **속도 제어 (Velocity Control)**
    - 로컬/글로벌 프레임에서 선속도·고도 제어
    - 속도 및 고도 PID 루프
3. **위치 제어 (Position Control)**
    - GPS/Landmark 기반 위치 고정(Loiter)
    - 궤적 추종(Trajectory Tracking)
4. **미션 실행 (Mission Execution)**
    - 기 설정된 웨이포인트 순차 수행
    - 고도·속도·카메라 트리거 동기화 가능

## MAVLink 프로토콜

---

- **MAVLink 1 & 2**
    - 경량 직렬(Protobuf 유사) 메시지 포맷
    - 다양한 메시지(HEARTBEAT, PARAM_VALUE, MISSION_ITEM 등) 정의
- **특징**
    - 패킷 재전송, 보안(버전2의 서명 기능)
    - 최대 255개 컴포넌트 간 통신 지원
- **용도**
    - 지상국(QGroundControl) ↔ 비행 컨트롤러
    - Companion Computer ↔ Autopilot
    - 매개변수 설정, 실시간 Telemetry 전송
- **확장성**
    - 커스텀 메시지 정의 가능
    - MAVSDK, pymavlink 등 라이브러리 지원.

## uXRCE‑DDS와 ROS2 연동

---

- **Micro XRCE‑DDS**
    - RTPS(DDS)의 경량화 버전, 임베디드 장치용
    - 에이전트(PC)↔클라이언트(픽스호크) 통신 구조
    - UDP/TCP/Serial 전송 지원
- **PX4‑ROS2 브리지**
    - 예: `px4_ros_com` 패키지: uORB ↔ DDS 주제 토픽 매핑
    - ROS2 노드로 센서·비행 상태, 제어 명령 퍼블리시/구독
- **장점**
    - 표준 DDS QoS 설정 가능
    - 분산 멀티-로봇 시스템 구축 용이
    - ROS2 패키지 연동 및 개발 가능(nav2 등)
- **실행 예시**
    1. PX4 SITL 모드 실행
    2. `ros2 launch px4_ros_com sensor_bridge.launch.py`
    3. ROS2 토픽 `/fmu/vehicle_odometry/out` 구독.

## 지원 하드웨어

---

- **Autopilot 보드**
    - **Pixhawk 시리즈**: Pixhawk 4, Pixhawk Mini, Pixhawk Cube(Orange, Purple)
    - **CUAV**: V5+, Nora, X7
    - **mRo**: Control Zero, Pocket
    - **Navio2**: 라즈베리파이 HAT 형태
- **센서·통신**
    - IMU(InvenSense ICM‑20602 등), Barometer, Magnetometer
    - GPS(U‑blox M8N, ZED‑F9P)
    - Telemetry(RF‑868/915, SiK 433/915)
- **Companion Computer**
    - Raspberry Pi, NVIDIA Jetson, Intel NUC
    - USB/Serial 연결, MAVLink 통신 가능
- **모터 드라이버**
    - ESC(DSHOT, BLHeli) 지원
    - CAN bus 기반 드라이버도 사용 가능

## 시뮬레이션 및 테스트

---

- **SITL (Software‑In‑The‑Loop)**
    - jMAVSim, Gazebo Classic / Gazebo Harmonic
    - FLIGHTGOGGLES, Blender 기반 시각화
- **HIL (Hardware‑In‑The‑Loop)**
    - PX4 하드웨어와 시뮬레이터 연동
    - 실제 ESC·모터·라디오 테스트 가능
- **로그 분석**
    - Flight Review, pyulog 패키지로 .ulg 파일 파싱
    - 비행 성능·안정성 분석, 리그레션 테스트