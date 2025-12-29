# ROS 2 Multi-Sensor 500Hz Performance Benchmark

This project demonstrates a high-frequency sensor system utilizing ROS 2 Humble. It is designed to evaluate middleware performance, clock synchronization accuracy, and latency overhead in a real-time context.

## Project Scope

* **Two Sensor Nodes**: Independent nodes generating and publishing timestamped sine waves at 500Hz (2ms period).
* **Central Processing Node**: A master node that subscribes to both sensor streams and records a 10-second snapshot of data.
* **Clock Synchronization**: Implementation of a master-slave clock alignment to ensure timestamps across nodes are synchronized to a single reference.
* **Latency Analysis**: Real-time calculation of end-to-end delay (Receive Time - Sent Timestamp).
* **DDS Tuning**: Optimization using FastDDS Shared Memory (SHM) transport to achieve sub-millisecond latency.

---

## Overview
```
ros2_challenge_ws/
├── fastdds_tuning.xml        # Middleware performance configuration
├── plot_results.gp           # Gnuplot script for visualization
├── run_all.sh                # Automation script for the benchmark
└── src/
    └── sensor_system/
        ├── CMakeLists.txt    # Build configuration
        ├── package.xml       # Package dependencies
        ├── src/
        │   ├── central_node.cpp    # Data recorder & sync master
        │   ├── sensor_node_1.cpp   # 500Hz Sine publisher (ID 1)
        │   └── sensor_node_2.cpp   # 500Hz Sine publisher (ID 2)
        └── msg/
            └── SensorSignal.msg    # Custom interface definition

```


## Prerequisites & Installation

### 1. ROS 2 Humble
Ensure you are running on Ubuntu 22.04 with ROS 2 Humble Desktop installed.

### 2. FastDDS Middleware
Install the RMW implementation for FastRTPS:
```bash
sudo apt update
sudo apt install ros-humble-rmw-fastrtps-cpp
```

### 3. Dependencies
Install Gnuplot for report generation and tcpdump for optional packet sniffing:
```bash
sudo apt install gnuplot tcpdump
```

## System Configuration

FastDDS Tuning (fastdds_tuning.xml)
The system uses a custom XML profile to force Shared Memory (SHM) transport, bypassing the network stack to minimize latency.

Clock Synchronization

* Central Node broadcasts a master timestamp.

* Sensor Nodes calculate their local clock offset relative to the master.

* Sensor Nodes apply this offset to all outgoing message timestamps.

## Execution Guide

### Option 1: Automated Run (Recommended)

The run_all.sh script handles building, environment setup, execution, and plotting:

```bash
chmod +x run_all.sh
./run_all.sh
```

### Option 2: Manual Run

If running separately, follow this order to ensure proper synchronization:

#### 1. Central Node

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastdds_tuning.xml
ros2 run sensor_system central_node
```

#### 2. Sensor Nodes

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastdds_tuning.xml
ros2 run sensor_system sensor_node_1
# and
ros2 run sensor_system sensor_node_2
```


## Results and Analysis

After the 10-second capture, the system generates Final_Challenge_Report.jpg.

* **Signal Integrity:** Both 5Hz sine waves should be perfectly aligned, proving successful clock synchronization.
* **Latency:** With Shared Memory enabled, the end-to-end latency should typically reside between 0.2ms and 0.9ms.

## Network Communication (Optional)

To verify the resulting network communication, run a packet sniffer while the benchmark is active:

```bash
sudo tcpdump -i lo -n udp port 7400
```



