#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}🗺️  SLAM 기반 고창 돌멘 공원 매핑 시작${NC}"

# ROS2 환경 설정
source install/setup.bash

# Gazebo 모델 경로
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/my_robot_hyo/hyo_model

echo -e "${YELLOW}📍 1단계: Gazebo 실행${NC}"
ros2 launch my_robot gazebo_with_robot.launch.py \
  world:=$HOME/my_robot_hyo/hyo_word/test4.world &
sleep 5

echo -e "${YELLOW}📍 2단계: SLAM Toolbox 실행${NC}"
ros2 launch my_robot slam_mapping.launch.py &
sleep 3

echo -e "${YELLOW}📍 3단계: RViz 실행${NC}"
rviz2 -d src/my_robot/rviz/slam_visualization.rviz &
sleep 2

echo -e "${GREEN}✅ 모든 시스템 실행 완료!${NC}"
echo ""
echo -e "${BLUE}🎮 조작 방법:${NC}"
echo -e "  ${YELLOW}새 터미널${NC}에서 다음 명령어 실행:"
echo -e "  ${GREEN}cd ~/yanyan && source install/setup.bash${NC}"
echo -e "  ${GREEN}ros2 run teleop_twist_keyboard teleop_twist_keyboard${NC}"
echo ""
echo -e "${BLUE}📊 맵 저장:${NC}"
echo -e "  ${GREEN}ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: dolmen_park_map}}\"${NC}"
echo ""
echo -e "${RED}⚠️  종료: Ctrl+C${NC}"

wait
