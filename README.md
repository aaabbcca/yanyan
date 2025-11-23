# KITTI 스타일 고인돌 공원 시각화

ROS2 + Gazebo + KITTI 스타일 포인트 클라우드 시각화 프로젝트

## 📋 프로젝트 개요
- 고창 고인돌 공원을 Gazebo로 시뮬레이션
- Velodyne LiDAR (116,000 points @ 10Hz)
- KITTI 데이터셋 스타일 시각화

## 🚀 빠른 시작

### 필수 패키지 설치
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan
```

### 빌드
```bash
cd ~/yanyan
colcon build
source install/setup.bash
```

### 실행

**터미널1: Gazebo**
```bash
ros2 launch my_robot gazebo_with_robot.launch.py
```

**터미널2: RViz (KITTI 스타일)**
```bash
ros2 launch my_robot kitti_visualization.launch.py
```

**터미널3: SLAM**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**터미널4: 키보드 조종**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 🎨 특징
- ✅ KITTI 스타일 포인트 클라우드
- ✅ 실시간 SLAM
- ✅ 고인돌 공원 3D 환경

## 👥 팀
- KITTI 시각화
- 환경 구축
- SLAM/Navigation
