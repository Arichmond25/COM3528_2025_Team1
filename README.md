# COM3528_2025_Team1

## Project Overview
This project enables a MiRo robot to detect another MiRo robot using YOLOv8, align itself with the detected MiRo, and move toward it. The system uses ROS for communication, YOLOv8 for object detection, and OpenCV for image processing.

---

## Setup Instructions

### 1. Prerequisites
- **Python Dependencies**: Install the required Python packages:
  ```bash
  pip install ultralytics opencv-python numpy rospy sensor-msgs geometry-msgs cv-bridge
  ```

## Launching the System

### 1. Build the ROS Workspace
Navigate to the workspace and build it:
```bash
cd /home/student/mdk/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Launch Files
The project includes the following launch files:


#### **`team1.launch`**
- **World Loaded**: `blue_world` world.
- **MiRo Robots**:
  - **`miro01`**: Runs the `capture_images.py` script, which captures images from MiRo's camera and saves them for further processing.
  - **`miro02`**: (Commented out in the file) Intended to run the `wander_intruder.py` script, which makes MiRo wander in the environment.
- **Purpose**: Simulates a scenario where MiRo captures images in a custom world.
- **Command**:
  ```bash
  roslaunch COM3528_2025_Team1 team1.launch

#### **`detect_miro.launch`**
- **World Loaded**: `detect_miro` world.
- **MiRo Robots**:
  - **`miro01`**: Runs the `detect_miro.py` script, which enables MiRo to detect another MiRo using YOLOv8 and move toward it.
  - **`miro02`**: Runs the `random_drive.py` script, which makes MiRo move randomly in the environment.
- **Purpose**: Simulates a scenario where one MiRo (patrol) detects and follows another MiRo (intruder).
- **Command**:
  ```bash
  roslaunch COM3528_2025_Team1 detect_miro.launch
  ```

## How It Works

1. **Detection**:
   - MiRo uses YOLOv8 to detect another MiRo in its camera frames.
   - Detected MiRo images are saved in the `detected_images` directory.

2. **Alignment**:
   - MiRo rotates until the detected MiRo is centered in its view (left camera's right side and right camera's left side).

3. **Navigation**:
   - MiRo moves forward when aligned with the detected MiRo.
   - Micro adjustments are made to maintain alignment.
