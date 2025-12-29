#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_challenge_ws/fastdds_tuning.xml

colcon build --packages-select sensor_system
source install/setup.bash

# Start packet capture in background (requires sudo)
sudo tcpdump -i lo -w network_log.pcap port 7400 or port 7401 &
TCPDUMP_PID=$!

ros2 run sensor_system central_node &
CENTRAL_PID=$!

sleep 2
ros2 run sensor_system sensor_node_1 &
ros2 run sensor_system sensor_node_2 &

wait $CENTRAL_PID
sudo kill $TCPDUMP_PID
killall sensor_node_1 sensor_node_2 2>/dev/null
gnuplot plot_results.gp
echo "Benchmark Finished. PCAP saved as network_log.pcap"