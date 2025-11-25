#!/bin/bash

echo "🎮 로봇 키보드 제어 시작!"
echo ""
echo "  w/x : 전진/후진"
echo "  a/d : 좌회전/우회전"
echo "  s   : 정지"
echo "  q/z : 속도 증가/감소"
echo ""

cd ~/yanyan
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
