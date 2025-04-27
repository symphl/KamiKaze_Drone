# 🚀 Autonomous Kamikaze Drone Project

This project demonstrates a drone that detects and tracks vehicles using YOLOv8 in a Gazebo simulation, running with PX4 and MAVROS, and autonomously flies towards them.

---

## 🛠 Installation Instructions

Follow these steps **from scratch** if you are setting up on a fresh machine:

1. **Install WSL2 and Ubuntu**
    - Install WSL2 on Windows and Ubuntu 20.04 LTS.

2. **Install ROS Noetic**
    ```bash
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    ```

3. **Install MAVROS**
    ```bash
    sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
    sudo apt install geographiclib-tools
    sudo geographiclib-get-geoids egm96-5
    ```

4. **Install PX4 Autopilot**
    ```bash
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd PX4-Autopilot
    bash ./Tools/setup/ubuntu.sh --no-nuttx
    ```

5. **Install Gazebo Classic (for ROS-Gazebo simulation)**
    PX4 setup will install it automatically, but ensure gazebo-classic is linked with ROS.

6. **Clone this repository**
    ```bash
    cd ~
    git clone https://github.com/<your-username>/drone_project.git
    cd drone_project
    ```

7. **Setup your ROS workspace**
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ln -s ~/drone_project/drone_control .
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

8. **Copy the Modified Iris Model**
    - Replace your PX4 iris model with the provided `iris.sdf.jinja`:
    ```bash
    cp ~/drone_project/modified_px4/Tools/simulation/gazebo-classic/models/iris/iris.sdf.jinja ~/PX4-Autopilot/Tools/simulation/gazebo-classic/models/iris/
    ```

---

## 🏃 Running the Project

Open **5 Terminals** and follow these steps:

### Terminal 1: Launch PX4 SITL
```bash
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic_iris
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```
### Terminal 2: Launch MAVROS
```
source ~/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```
Terminal 3: Start Vehicle State Monitor Node
```
source ~/catkin_ws/devel/setup.bash
rosrun drone_control vehicle_state_monitor.py
```
Terminal 4: Start Target Tracking Controller Node
```
source ~/catkin_ws/devel/setup.bash
rosrun drone_control target_tracking_controller.py
```
Terminal 5: Visualize Processed Image using rqt
```
rqt

# Then open "Plugins" > "Visualization" > "Image View"
# Select topic: `/processed_image`
```
