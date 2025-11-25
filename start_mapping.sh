#!/bin/bash

echo "🗺️  고창 돌멘 공원 SLAM 매핑 시작!"
echo ""

cd ~/yanyan
source install/setup.bash

# 한 번에 모든 것 실행!
ros2 launch my_robot complete_slam_visualization.launch.py

