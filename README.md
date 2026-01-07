# ROS2 Workspace Setup (Ubuntu 22.04 + ROS2 Humble)

This guide explains how to replicate the **ROS2 workspace** exactly as developed on the reference machine (Ubuntu 22.04, ROS 2 Humble).  
Follow each step carefully to ensure full compatibility.

---

## 🧭 1. Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone all required repositories
1️⃣ xArm ROS2 driver

```bash
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
```
📖 Follow installation instructions from the official repo:
👉 https://github.com/xArm-Developer/xarm_ros2

2️⃣ Orbbec SDK ROS2
```bash
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
```
📖 Follow setup steps described here:
👉 https://github.com/orbbec/OrbbecSDK_ROS

3️⃣ IFRA Link Attacher
```bash
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
```
📖 Refer to the instructions provided in the repo:
👉 https://github.com/IFRA-Cranfield/IFRA_LinkAttacher

4️⃣ Aliyu custom packages (LearnRoboticsWROS)

a. Core robot description and configuration, simulation in Gazebo and Moveit in custom setup
```bash
git clone https://github.com/LearnRoboticsWROS/my_xarm6_simulation.git
```

b. Main application node
```bash
git clone https://github.com/LearnRoboticsWROS/my_xarm6_app.git
```

c. Interfaces (custom messages/services)
```bash
git clone https://github.com/LearnRoboticsWROS/my_xarm6_interfaces.git
```

📂 3. Verify workspace structure

Before building, make sure you have exactly these six directories inside ~/aliyu_ws/src:

~/aliyu_ws/src/
├── IFRA_LinkAttacher
├── OrbbecSDK_ROS2
├── my_xarm6_simulation
├── my_xarm6_app
├── my_xarm6_interfaces
└── xarm_ros2

🧱 4. Build the workspace

Every terminal open must source ros2 environment: 
```bash
source /opt/ros/humble/setup.bash
```

From the workspace root:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

From thi moment on, remember to source ros2 and workspace environment everytime a terminal is opened
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
```

## 🚀 5. Test and Run

Once the workspace has been successfully built and sourced, you can test the robot simulation and motion planning.

---

### 🦾 Test 1 — Basic robot spawn & inverse kinematics (IK) control

This test verifies that the robot model, MoveIt configuration, and gripper are correctly loaded, and that the inverse kinematics service is working.

#### 🖥️ Terminal 1 — Launch the simulation environment
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```
This will:
Launch Gazebo with the robot and the gripper model.
Load the MoveIt motion planning environment.
Start RViz for visualization.
You should see the robot appear in the world with its attached gripper.

### Test 2 — Inverse Kinematics computation
Once the robot is running, open another terminal to compute IK for a target pose.

#### 🖥️ Terminal 2 — Run the IK service client
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_xarm6_app move_to_pose
```
This script will send a test pose to the inverse kinematics solver and print the resulting joint values in the terminal.

Expected result:
You should see the robot goes in target position

---

### 🦿 Test 3 — Pick & Place in “Blind Mode”

This test verifies that the pick and place logic and the object attachment mechanism (gripper link attacher) are working correctly.  
It runs a simple hard-coded motion without using vision or perception data.

Make sure the simulation from **Test 1** is still running (Gazebo + MoveIt + RViz).

#### 🖥️ Terminal 2 — Run the pick & place node
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_xarm6_app pick_place
```
What happens:
- The node will command the robot to move to a predefined “pick” position.
- The gripper will attach the target object using the LinkAttacher plugin.
- The robot will then move to a predefined “place” position and release the object.
This validates:
- Correct communication between the application node and MoveIt.
- The gripper attachment mechanism is functioning properly.
- The complete motion sequence executes without errors.
Expected behavior:
- The robot arm moves smoothly to the object, picks it up, and places it at another position in the scene.
- You should see the object attached to the gripper during motion and detached at the end.

---

### 👁️ Test 4 — Pick & Place with Vision

This test integrates the **vision pipeline** with the robot pick and place logic.  
The robot will now detect the object using a simulated RGB-D camera, compute its 3D position, and perform the pick and place automatically based on this vision data.

Make sure the simulation from **Test 1** is still running (Gazebo + MoveIt + RViz).

---

#### 🖥️ Terminal 1 — Simulation
Keep Gazebo and MoveIt running as before:
```bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```
#### 🖥️ Terminal 2 — Run the pick & place node
Run the node that identifies the object and provides its 3D position:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_xarm6_app object_position_server
```
This node:
- Detects the target object using color-based masking.
- Uses the simulated depth camera intrinsics to convert pixel coordinates (u, v, depth) → 3D coordinates (X, Y, Z).
- Publishes the position in meters with reference to the camera_optical_link frame.

#### 🤖 Terminal 3 — Vision-based pick & place node
Now run the node that uses the detected object position to plan and execute the motion:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_xarm6 pick_place_vision
```
This node:
- Requests the object pose from the vision server.
- Transforms it into the robot base frame.
- Plans the motion using MoveIt.
- Executes the pick and place trajectory using the LinkAttacher to grasp and release the object.

✅ Expected behavior
- The robot will automatically move to the detected object position (based on camera data).
- The gripper will attach the object, lift it, and place it at a defined location.
- In RViz and Gazebo you’ll see the robot reacting to the object’s actual location rather than a fixed position.

🎯 Success criteria
If the robot detects and manipulates the object autonomously based on the camera’s input, the full vision-to-action pipeline is working correctly:
- Object detection ✅
- Depth-to-3D conversion ✅
- Coordinate transformation ✅
- Motion planning and execution ✅
You now have a fully functional ROS2 pick & place application with vision.

---

## ⚙️ Test 5 — Integration with LLM (Ollama)

In this section, we integrate a **Large Language Model (LLM)** with the robot application.  
The goal is to allow natural-language control of the robot through **Ollama** running locally.

---

### 🧩 Overview

We will tackle this integration in **two main steps**:

#### 🪜 Step 1 — Direct Command Execution
The user sends a text command (prompt) such as:

> “Move the TCP to x 0.3 y 0.5 z 0.3 keeping current orientation”

Ollama interprets it, generates a **JSON plan**, and a ROS2 node translates that JSON into robot commands executed via **MoveIt**.

#### 🧠 Step 2 — Reasoning Loop (Next phase)
A ROS2 node will orchestrate reasoning loops by using the LLM as a “tool” for high-level decision making (e.g., multi-step tasks).

---

## 🪜 Step 1 — Setup and Testing

### Install Ollama

```bash
curl -fsSL https://ollama.com/install.sh | sh
```
- check:
```bash
ollama --version
```
- Get the model
```bash
ollama pull llama3
```
### Source the environemt and test

```bash
cd ~/ros2_ws/src/my_xarm6_app
source venv/bin/activate
python -m my_xarm6_app.llm.test_ollama
```
You should receive a response printed in the terminal from the llama3 model.


# Architecture Step 1
Terminal (ros2 topic pub)
        │
        ▼
/llm_command (String)
        │
        ▼
[llm_command_node] → call Ollama → generate PoseStamped → pub /target_pose
        │
        ▼
[move_to_position_from_llm] → call IK → FollowJointTrajectory
        │
        ▼
robot in Gazebo moves

- test
- Terminal 1:
```bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```

- Terminal 2:
```bash
ros2 run my_xarm6_app move_to_position_llm
```

- Terminal 3:
```bash
cd ~/my_xarm6_app
source venv/bin/activate
ros2 launch my_xarm6_app llm_command_node
```

- Terminal 4:
```bash
ros2 topic pub /llm_command std_msgs/String "{data: 'move the TCP to x 0.3 y 0.5 z 0.3 keeping current orientation'}"
```

This command will be:
- Interpreted by Ollama (llama3),
- Converted into a structured JSON with target pose,
- Published on /target_pose,
- Consumed by the motion node to compute IK and send a trajectory to the robot in simulation.
✅ Expected Result
- The terminal running llm_command_node prints the parsed JSON command.
- The robot in Gazebo moves to the requested pose.
- No error messages appear in either node.
You have now successfully integrated a local LLM with ROS2 for natural-language robotic control 🎯.

---

## 🧠 Step 2 — Reasoning Loop with LLM as Tool

In this step, we integrate the **LLM (Ollama)** as a *reasoning engine* that can understand complex, multi-step commands (e.g., “pick the red object and place it somewhere else”).  
A ROS 2 node called **llm_task_node** interprets the natural-language task, generates a structured JSON plan, and coordinates the other nodes (vision, motion, and gripper).

---

### 🧱 System Architecture

User prompt → /llm_task (std_msgs/String)
│
▼
[llm_task_node] → calls Ollama → outputs structured JSON
│
▼
[pick_place_vision_llm] → calls vision service (object_position_server)
│
▼
[move_to_position_llm] → computes IK → executes trajectory
│
▼
Robot performs task in Gazebo


---

## 🧪 Test Procedure (Step 2)

You will use **five terminals**.  
Make sure you have built the workspace and sourced the environment (`source install/setup.bash`).

---

### 🖥️ Terminal 1 — Launch simulation

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```
This starts Gazebo + MoveIt + RViz.

### 👁️ Terminal 2 — Start the vision server
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_xarm6_app object_position_server
```
This node identifies colored objects using a color mask and computes their 3D positions from the simulated depth camera, publishing them in the camera_optical_link frame.

### 🤖 Terminal 3 — Start the vision-based pick & place node (LLM-enabled)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_xarm6_app pick_place_vision_llm
```
This node receives parsed JSON tasks from the LLM and executes them by calling the vision service and the motion node (move_to_position_llm).

### 🧩 Terminal 4 — Launch the LLM task node
```bash
cd ~/ros2_ws/src/my_xarm6_app
source venv/bin/activate
ros2 run my_xarm6_app llm_task_node
```
This node:
- Subscribes to /llm_task
- Sends the text command to Ollama
- Parses the LLM response into structured JSON
- Publishes the plan so that pick_place_vision_llm can execute it

### 💬 Terminal 5 — Send natural-language commands
🧭 Example 1 — Simple motion command
```bash
ros2 topic pub -1 /llm_command std_msgs/String "{data: 'move the TCP to x 0.3 y 0.5 z 0.3 keeping current orientation'}"
```
✅ Expected behavior:
- The llm_task_node parses the command.
- The move_to_position_llm node computes IK.
- The robot moves to the target pose in Gazebo.

🎯 Example 2 — Full pick & place task
```bash
ros2 topic pub -1 /llm_task std_msgs/String "{data: 'pick the red object with the orientation of roll -3.14 pitch 0.0 yaw 0.0 and place it at x 0.3 y 0.3 z 0.3 with orientation of tcp of roll -3.14 pitch 1.0 yaw 0.0'}"
```


✅ Expected behavior:

- The llm_task_node calls Ollama and generates a JSON plan.
- The pick_place_vision_llm node requests the red object position from object_position_server.
- The robot moves to the detected position, picks the object, and places it at the target pose with the specified orientation.

You should clearly see:
- The object being detected and attached to the gripper.
- The robot moving autonomously following the LLM-interpreted task.

🏁 Success Criteria
If the robot successfully performs both the direct movement and the color-based pick & place based on natural-language inputs:
- The reasoning loop works ✅
- The vision pipeline is integrated ✅
- The motion execution pipeline (IK + FollowJointTrajectory) is validated ✅
Congratulations! 🎉
You now have a fully functional ROS 2 + Ollama LLM robotic control system capable of understanding and executing high-level natural-language tasks.


---

## 🙌 Credits

Developed by **Francesco Rizzotti**  
from [**Learn Robotics With ROS**](https://www.learn-robotics-with-ros.com)

📧 **Email:** [ros.master.ai@gmail.com](mailto:ros.master.ai@gmail.com)

Francesco is available for:
- Advanced ROS2 / MoveIt2 / Isaac Sim developments  
- Custom industrial deployments with real robots and vision system integrated 
- Vision-based application and AI-powered automation systems  

© 2025 Learn Robotics With ROS – All rights reserved.
