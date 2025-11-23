#!/bin/bash

echo "🔍 ROS2 Environment Test"
echo "========================"

# Topic 확인
echo "📡 Topics:"
ros2 topic list | grep -E "(scan|velodyne|odom)"

# Gazebo 확인
echo ""
echo "🎮 Gazebo:"
pgrep -x "gzserver" > /dev/null && echo "✅ Running" || echo "❌ Not Running"

echo ""
echo "✅ Test Complete!"
