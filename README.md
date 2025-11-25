# 🏛️ Gochang Dolmen Park SLAM Robot Simulation

고창 돌멘 공원 자율주행 로봇 시뮬레이션 및 SLAM 매핑 시스템

## 🎯 프로젝트 개요

- **목적**: 고창 돌멘 공원 환경에서 자율주행 로봇의 SLAM 기반 맵핑
- **플랫폼**: ROS2 Humble + Gazebo + SLAM Toolbox
- **센서**: 2D LiDAR, 3D LiDAR (Velodyne), Depth Camera, RGB Camera

## 🚀 빠른 시작

### 1. 저장소 클론
```bash
cd ~
git clone https://github.com/aaabbcca/yanyan.git
cd yanyan
```

### 2. 의존성 설치
```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-slam-toolbox -y
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
sudo apt install xterm -y

# 추가 도구
sudo apt install ros-humble-teleop-twist-keyboard -y
sudo apt install ros-humble-rqt-image-view -y
```

### 3. 빌드
```bash
cd ~/yanyan
colcon build --symlink-install
source install/setup.bash
```

### 4. 실행!
```bash
./start_complete_system.sh
```

## 📦 실행되는 컴포넌트

실행 시 자동으로 다음 창들이 열립니다:

1. 🏞️ **Gazebo** - 고창 돌멘 공원 3D 시뮬레이션
2. 📊 **RViz** - SLAM 맵 + 센서 데이터 시각화
3. 🗺️ **SLAM Toolbox** - 실시간 맵 생성 (백그라운드)
4. 📸 **Camera View** - 로봇 카메라 이미지
5. 🎮 **Teleop** - 키보드 제어 인터페이스

## 🎮 로봇 조작

Teleop 창에서 키보드로 로봇 제어:
```
   w    : 전진
   x    : 후진
   a    : 좌회전
   d    : 우회전
   s    : 정지
   q    : 속도 증가
   e    : 속도 감소
```

## 💾 맵 저장
```bash
# 매핑 완료 후 맵 저장
cd ~/yanyan
source install/setup.bash

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'gochang_dolmen_park_map'}}"
```

## 📁 프로젝트 구조
```
yanyan/
├── src/
│   └── my_robot/
│       ├── launch/
│       │   ├── complete_system.launch.py       # 통합 실행
│       │   ├── complete_slam_visualization.launch.py
│       │   ├── gazebo_with_robot.launch.py
│       │   └── slam_mapping.launch.py
│       ├── config/
│       │   └── slam_params.yaml                # SLAM 설정
│       ├── worlds/
│       │   └── gochang_dolmen_park.world       # 고창 공원 World
│       ├── models/                             # 3D 모델들
│       ├── rviz/
│       │   └── slam_visualization.rviz         # RViz 설정
│       └── urdf/                               # 로봇 모델
├── start_complete_system.sh                    # 실행 스크립트
└── README.md
```

## 🛠️ 개별 실행 (디버깅용)

전체 시스템이 아닌 개별 컴포넌트 실행:
```bash
# Gazebo만
ros2 launch my_robot gazebo_with_robot.launch.py

# SLAM만
ros2 launch my_robot slam_mapping.launch.py

# RViz만
rviz2 -d src/my_robot/rviz/slam_visualization.rviz

# Teleop만
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 📊 센서 토픽
```bash
/scan                                          # 2D LiDAR
/velodyne_points                               # 3D LiDAR
/camera_center/camera_center/image_raw         # RGB Camera
/depth_camera/depth_camera/points              # Depth Camera
/map                                           # SLAM Map
/odom                                          # Odometry
```

## 🎨 RViz 시각화 항목

- 🗺️ SLAM Map (회색 그리드)
- 🔴 2D LiDAR Scan (빨간 점)
- 🌈 3D LiDAR PointCloud (무지개 색상)
- 📷 Depth Camera PointCloud
- 🤖 Robot Model
- 📐 TF Frames

## ⚙️ 시스템 요구사항

- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **RAM**: 8GB 이상 권장
- **GPU**: NVIDIA GPU 권장 (Gazebo 성능)

## 📝 크레딧

- **World 환경**: HYOSUN123/jbnu_ros2_11-26
- **로봇 시스템**: Custom TurtleBot3-based robot
- **SLAM**: SLAM Toolbox

## 📧 문의

프로젝트 관련 문의: [GitHub Issues](https://github.com/aaabbcca/yanyan/issues)

## 📄 라이선스

MIT License
