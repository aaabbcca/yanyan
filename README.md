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

## 🎨 RViz 시각화 설정

### 최종 디스플레이 구성:
```
✓ Grid - 기본 그리드
✓ SLAM Map - 2D 누적 맵 (Alpha: 0.2, 반투명)
✓ RobotModel - 로봇 3D 모델
✓ TF - 좌표계 프레임
✓ LaserScan - 2D LiDAR (빨간색 원형)
✓ PointCloud2 - 3D LiDAR (흰색 포인트클라우드)
```

### PointCloud 설정:

**실시간만 표시:**
```yaml
Decay Time: 0
Color: 흰색 (255; 255; 255)
Color Transformer: FlatColor
Use rainbow: ☐
```

**누적 효과 (N초간):**
```yaml
Decay Time: 5 ~ 120 (초 단위)
Color: 흰색 (255; 255; 255)
Style: Spheres
Size: 0.05
```

### SLAM Map 설정:
```yaml
Alpha: 0.2 (반투명)
Draw Behind: ✓
Color Scheme: map
```

### RViz 설정 파일 위치:
```
~/yanyan/src/my_robot/rviz/final_slam_visualization.rviz
```

### 실행 방법:
```bash
# 전체 시스템 (Gazebo + SLAM + RViz)
cd ~/yanyan
./start_complete_system.sh

# RViz만 (저장된 설정으로)
source install/setup.bash
rviz2 -d src/my_robot/rviz/final_slam_visualization.rviz
```

## 💾 맵 저장하기

매핑 완료 후 맵을 저장하려면:
```bash
cd ~/yanyan
source install/setup.bash

# SLAM 맵 저장
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'gochang_dolmen_final_map'}}"
```

저장된 파일:
- `gochang_dolmen_final_map.yaml` - 맵 메타데이터
- `gochang_dolmen_final_map.pgm` - 맵 이미지

## 🎯 발표/시연용 팁

### 1. 깔끔한 시각화:
- SLAM Map Alpha를 0.2로 설정 (배경으로)
- PointCloud를 흰색 FlatColor로 설정
- Decay Time을 5-10초로 설정 (적절한 누적)

### 2. 로봇 제어:
- Teleop 창에서 w/x/a/d로 부드럽게 이동
- 천천히 움직여서 맵 품질 향상

### 3. 카메라 각도:
- 위에서 내려다보는 각도 (Orbit 뷰)
- Distance: 15-25m
- Pitch: 0.8-1.0

### 4. 스크린샷/녹화:
```bash
# 스크린샷
gnome-screenshot -w

# 화면 녹화 (설치 필요시)
sudo apt install simplescreenrecorder -y
simplescreenrecorder
```

## 📊 성능 메트릭

- YOLO 처리: ~0.05초
- BLIP 처리: ~0.8초
- SLAM 업데이트: 실시간
- 포인트클라우드: 30Hz
- LaserScan: 10Hz

