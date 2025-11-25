#!/bin/bash

echo "🚀 고창 돌멘 공원 완전 통합 시스템"
echo "========================================"
echo ""
echo "실행 항목:"
echo "  📦 Gazebo (World + Robot)"
echo "  🗺️  SLAM Toolbox (매핑)"
echo "  📊 RViz (시각화)"
echo "  📸 Camera View (카메라)"
echo "  🎮 Teleop (키보드 제어)"
echo ""
echo "========================================"
echo ""

cd ~/yanyan
source install/setup.bash

ros2 launch my_robot complete_system.launch.py

