#!/bin/bash
# Robot 2 실행 스크립트 (Domain 2, IGN_PARTITION=robot2)

ROBOT_ID=2
DOMAIN_ID=2
IGN_PART="robot2"
ROS2_WS="$HOME/SW_project/ros2_ws"
MAP_PATH="$HOME/SW_project/ros2_ws/test.yaml"

echo "🤖 Starting Robot $ROBOT_ID"
echo "  - ROS_DOMAIN_ID: $DOMAIN_ID"
echo "  - IGN_PARTITION: $IGN_PART"
echo ""

gnome-terminal --window \
    --tab --title="R${ROBOT_ID}-rl1" -e "bash -c 'export ROS_DOMAIN_ID=$DOMAIN_ID; export IGN_PARTITION=$IGN_PART; cd $ROS2_WS; source install/setup.bash; echo rl1: Gazebo + Robot; ros2 launch agilex_scout simulate_control_gazebo.launch.py lidar_type:=3d rviz:=false gui:=false; exec bash'" \
    --tab --title="R${ROBOT_ID}-rl2" -e "bash -c 'export ROS_DOMAIN_ID=$DOMAIN_ID; cd $ROS2_WS; source install/setup.bash; echo Waiting 15s...; sleep 15; echo rl2: Navigation2; ros2 launch scout_nav2 nav2.launch.py simulation:=true; exec bash'" \
    --tab --title="R${ROBOT_ID}-rl3" -e "bash -c 'export ROS_DOMAIN_ID=$DOMAIN_ID; cd $ROS2_WS; source install/setup.bash; echo Waiting 20s...; sleep 20; echo rl3: TF publisher; ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom; exec bash'" \
    --tab --title="R${ROBOT_ID}-rl4" -e "bash -c 'export ROS_DOMAIN_ID=$DOMAIN_ID; cd $ROS2_WS; source install/setup.bash; echo Waiting 25s...; sleep 25; echo rl4: Nav2 bringup; ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=$MAP_PATH; exec bash'"

echo "✅ 4 tabs opened for Robot $ROBOT_ID (rl1~rl4 auto-run)"
