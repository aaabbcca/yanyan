#!/bin/bash

# 터미널 색상
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🏛️ 고창 돌멘 공원 가시화 시작${NC}"

# ROS2 환경 설정
source install/setup.bash

# Gazebo 모델 경로 등록
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/my_robot_hyo/hyo_model

echo -e "${YELLOW}📍 1단계: Gazebo 실행 중...${NC}"
# Gazebo 실행 (백그라운드)
ros2 launch my_robot gazebo_with_robot.launch.py \
  world:=$HOME/my_robot_hyo/hyo_word/test4.world &

GAZEBO_PID=$!
sleep 5

echo -e "${YELLOW}📍 2단계: RViz 실행 중...${NC}"
# RViz 실행
ros2 run rviz2 rviz2 -d src/my_robot/rviz/dolmen_park_visualization.rviz &

RVIZ_PID=$!
sleep 3

echo -e "${GREEN}✅ 가시화 실행 완료!${NC}"
echo -e "${YELLOW}📌 추가 도구:${NC}"
echo -e "  - 키보드 제어: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo -e "  - 이미지 뷰: ros2 run rqt_image_view rqt_image_view"

# 종료 대기
wait
