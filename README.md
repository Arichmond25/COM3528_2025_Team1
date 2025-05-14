# COM3528_2025_Team1

## Project Overview
TODO

---

## Setup Instructions

### 1. Prerequisites
- **Python Dependencies**: Install the required Python packages:
  ```bash
  pip install ultralytics 
  ```
- **Python Dependencies for running detect_miro in real life**:
  ```bash
  pip install ultralytics 
  pip install torch==2.2.0 torchvision==0.17.0
  ```

## Launching the System

### 1. Build the ROS Workspace
Navigate to the workspace and build it:
```bash
cd /home/student/mdk/catkin_ws/src
catkin build
src
cd COM3528_2025_Team1
chmod +x src/*.py
```

### 2. Run In Sim
The project includes the following launch files:


#### **`team1.launch`**
- **World Loaded**: `blue_world` world.
- **MiRo Robots**:
  - **`miro01`**: Runs the `patrol_miro.py` script, which truns the miro into the patroler.
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

### 2. Run In Real Life 
1. **Connect to miro**
2. **Run:**
```bash
  rosrun COM3528_2025_Team1 detect_miro.py
  ```

## How It Works

### 1. **Detection**
- MiRo uses the trained YOLOv8 model to detect another MiRo in its camera frames.

### 2. **Alignment**
- MiRo rotates until the detected MiRo is centered in its view (left camera's right side and right camera's left side).

### 3. **Navigation**
- MiRo moves forward when aligned with the detected MiRo.
- Micro adjustments are made to maintain alignment.

### 4. **Patrolling (`patrol_miro.py`)**
- MiRo follows a predefined patrol route consisting of forward movements and 90-degree turns.
- The patrol route is defined in the `start_patrol_steps` (for exiting the start box) and `patrol_steps` (for the main patrol loop).
- MiRo continuously scans for intruders using its left and right cameras:
  - If an intruder is detected, MiRo transitions to the **CHASE** state and moves toward the detected MiRo.
  - If MiRo loses sight of the intruder, it transitions to the **RECOVER** state and retraces its steps to return to the patrol route.
- MiRo uses a PID controller to ensure smooth and accurate movements during patrol and recovery.
- The robot's head is reset to a default position to ensure consistent camera alignment.

---

## Project Structure

The project is organized as follows:

```
COM3528_2025_Team1/
├── src/
│   ├── capture_images.py       # Script to capture images from MiRo's camera
│   ├── detect_miro.py          # Script to detect another MiRo using YOLOv8
│   ├── patrol_miro.py          # Script to make MiRo patrol a predefined route
│   ├── random_drive.py         # Script to make MiRo move randomly
│   ├── yolo_model/
│   │   └── best.pt             # YOLOv8 trained model file
│   └── detected_images/        # Directory to save detected MiRo images
├── launch/
│   ├── team1.launch            # Launch file for the `blue_world` scenario
│   ├── detect_miro.launch      # Launch file for the `detect_miro` scenario
├── sim/
│   ├── worlds/
│   │   ├── blue_world          # Custom world for the `team1.launch` file
│   │   └── detect_miro         # Custom world for the `detect_miro.launch` file
├── dataset/                    # Data set used to train the YOLOv8 model
├── README.md                   # Project documentation

```
