# ROS2 Rosbag Splitter & Odometry Analyze

This project allows you to split a rosbag file into smaller chunks based on a specific time duration in a ROS2 (Humble) environment. Additionally, with the provided script, you can perform odometry analysis on your rosbag files that contain message types such as "geometry_msgs/PoseStamped", "geometry_msgs/TransformStamped", "geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/PointStamped", or "nav_msgs/Odometry".

## Used Packages
- `rclpy`
- `rosbag2_py`
- `evo`

## Installation

### 1. Install ROS2 Humble
Make sure that ROS2 Humble is installed. Follow the [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html) for installation instructions.

### 2. Install Evo Tools
To install Evo Tools, refer to the [Evo GitHub repository](https://github.com/MichaelGrupp/evo).

### 3. Set up ROS2 Workspace
You need to run this project within your own ROS2 workspace (e.g., `ros_ws`). Follow the steps below to create your workspace:

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/your_username/rosbag_analyze.git
cd ~/ros_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Rosbag Splitter

**Note:** Before running the program, ensure that the rosbag folder you want to split is located inside `~/ros_ws/src/rosbag2_tool/rosbag_analyze/rosbags`. If it is not, please move the folder to the appropriate location or modify the folder path when running the program.

#### Parameters

- `input_uri`: URI of the input rosbag file (default: `~/ros_ws/src/rosbag2_tool/rosbag_analyze/rosbags/case_bag`).
- `output_uri`: URI of the output rosbag file (default: `~/ros_ws/src/rosbag2_tool/rosbag_analyze/created_rosbags/new`).
- `split_duration`: Duration of each split (in seconds, default: 60).

#### Example Usage

##### 1.1 Run with default parameters

To run the program with default parameters, use:

```bash
ros2 run rosbag_analyze split_rosbag
```
or

```bash
ros2 launch rosbag_analyze rosbag_splitter.launch.py
```

##### 1.2 Run with updated parameters

```bash
ros2 run rosbag_analyze split_rosbag --ros-args -p input_uri:="/home/user/my_rosbags/my_rosbag" -p output_uri:="/home/user/my_results/new_rosbag"
```

or

```bash
ros2 launch rosbag_analyze rosbag_splitter.launch.py input_uri:="/home/user/my_rosbags/my_rosbag" output_uri:="/home/user/my_results/new_rosbag" split_duration:="120"
```

### 2. Odometry Analyze

To perform Trajectory, Relative Pose Error (RPE), and Absolute Pose Error (APE) analysis, move your rosbag folders into `~/ros_ws/src/rosbag2_tool/rosbag_analyze/created_rosbags`. You can examine the metrics and visualization results in the `~/ros_ws/src/rosbag2_tool/rosbag_analyze/evo_tool` directory.

**Note** Make sure that there are `metrics` and `visualization` folders in the location where you will save the analysis results.

Output Files:
- `Trajectory: Visualizations and metrics of the odometry trajectory (PNG and JSON files).`
- `RPE: Images for Relative Pose Error analysis (PNG) and result files (ZIP).`
- `APE: Images for Absolute Pose Error analysis (PNG) and result files (ZIP).`

You can run the script to analyze the odometry for each rosbag. The following command analyzes the data in the subdirectories of the specified directory:

```bash
python3 ~/ros_ws/src/rosbag2_tool/rosbag_analyze/scripts/evo_tool_analyze.py
```
**Note:** Before running the program, ensure that the rosbag folder you want to analyze is located inside `~/ros_ws/src/rosbag2_tool/rosbag_analyze/created_rosbags`. If it is not, please move the folder to the appropriate location or modify the folder path when running the program.

**Note:** You can modify the input and output paths when running the script. If you want to use the default paths, just press 'Enter'.
