# Franka Research 3 MuJoCo Environment

This project provides a complete MuJoCo simulation environment for the Franka Research 3 (FR3) robot arm, built from the official Franka ROS packages with high-quality visual meshes.

## Features

- **Official FR3 Model**: Generated from official franka_ros URDF/xacro with joint_limits.yaml
- **High-Quality Visuals**: Uses official MuJoCo Franka visual meshes
- **Realistic Physics**: STL collision meshes with accurate inertial properties
- **MuJoCo Integration**: Native MJCF format optimized for simulation
- **Python API**: Easy-to-use environment class
- **Interactive Viewer**: Real-time 3D visualization with detailed meshes

## Installation

### Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

### Build environment: `franka_mujoco_env`

The packages are already installed in `franka_mujoco_env`, If you want to create a new one or don't already have the project's virtual environment, create it once:

1. Create the virtual environment
   ```bash
   python3 -m venv franka_mujoco_env
   ```

2. Activate it and upgrade `pip`
   ```bash
   source franka_mujoco_env/bin/activate
   python -m pip install --upgrade pip
   ```

3. Install Python dependencies
   ```bash
   pip install mujoco numpy scipy matplotlib ikpy transforms3d tqdm lxml rospkg catkin_pkg dm-control
   ```

   - **Required packages**: mujoco, dm-control, numpy, scipy, matplotlib, ikpy, transforms3d, tqdm, lxml, rospkg, catkin_pkg

4. Verify installation (optional)
   ```bash
   python - <<'PY'
   import mujoco, numpy
   print('MuJoCo:', mujoco.__version__)
   print('NumPy:', numpy.__version__)
   PY
   ```

### Setup

1. **Activate the virtual environment**:
   ```bash
   source franka_mujoco_env/bin/activate
   ```

2. **Navigate to the project directory**:
   ```bash
   cd franka_mujoco_project
   ```

## Files Overview

### Main Model

- `fr3con.xml` - **PRIMARY** Official FR3 model with visual meshes
- `meshdir/` - Directory containing visual meshes
- `catkin_ws/src/franka_ros/franka_description/` - Official Franka meshes and URDF sources

### Generated Models

- `fr3_with_gripper.urdf` - URDF generated from official xacro with hand:=true
- `fr3_arm_only.urdf` - URDF with hand:=false

### Demo Scripts and Environment Classes

- `movement_demo.py` - **NEW** Complete arm movement demonstration with smooth pose transitions
- `main.py` - Simple static pose demonstration
- `franka_ik_test.py` - Inverse kinematics testing with ikpy
- `joint_demo.py` - Inspect joint names/types/ranges and actuator mapping; runs a brief motion
- `cartesian_demo.py` - Print end-effector pose (site `attachment_site`) and trace a small Cartesian path
- `franka_environment_corrected.py` - Main environment class
- `franka_environment.py` - Original environment class
- `demo_franka.py` - Basic demonstration script

## Usage

### Quick Start Demos

**Animated Movement Demo** (Recommended):
```bash
python movement_demo.py
```
This demonstrates smooth arm movements through 6 different poses:
- Home position → Reach forward → Move right → Move left → Upright → Return home
- Features smooth interpolation, timing control, and console feedback

**Static Pose Demo**:
```bash
python main.py
```
Simple demonstration showing the robot in a fixed pose.

**Inverse Kinematics Demo**:
```bash
python franka_ik_test.py
```
Test inverse kinematics calculations with visualization.

### Direct Model Loading

Load the official FR3 model with high-quality visual meshes:

```bash
python - <<'PY'
import mujoco
import mujoco.viewer

# Load the official FR3 model with visual meshes
model = mujoco.MjModel.from_xml_path('fr3con.xml')
data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
PY
```

This will:
1. Load the official Franka FR3 model with gripper
2. Display high-quality visual meshes from franka_description
3. Use accurate collision geometry and joint limits from joint_limits.yaml
4. Start the interactive MuJoCo viewer with realistic rendering

### Environment Class Usage

For programmatic control, use the environment class with the official model:

```python
from franka_environment_corrected import FrankaFR3EnvironmentCorrected
import numpy as np

# Load the official FR3 model
env = FrankaFR3EnvironmentCorrected('fr3con.xml')

# Reset to home position
env.reset_to_home()

# Demonstrate joint control
env.demo_joint_control()

# Start interactive viewer
env.start_viewer()
```

## Environment API

### FrankaFR3Environment Class

#### Key Methods

- `reset_to_home()` - Reset robot to home position
- `set_joint_positions(positions)` - Set arm joint positions
- `set_gripper_position(position)` - Control gripper (0=closed, 0.04=open)
- `get_joint_positions()` - Get current joint positions
- `get_end_effector_pose()` - Get end-effector position and orientation
- `apply_joint_torques(torques)` - Apply control torques
- `step()` - Step simulation forward
- `render()` - Render scene to image
- `start_viewer()` - Launch interactive viewer

#### Example Usage

```python
from franka_environment import FrankaFR3Environment
import numpy as np

# Create environment
env = FrankaFR3Environment()

# Reset to home position
env.reset_to_home()

# Set specific joint positions
target_joints = np.array([0.5, -0.5, 0, -2.0, 0, 1.5, 0.785])
env.set_joint_positions(target_joints)

# Control gripper
env.set_gripper_position(0.02)  # Half open

# Get current state
joint_pos = env.get_joint_positions()
ee_pos, ee_rot = env.get_end_effector_pose()

# Apply control
torques = np.array([1, -1, 0, 2, 0, -1, 0])
env.apply_joint_torques(torques)
env.step()
```

## Model Specifications

### Official FR3 Model (`fr3con.xml`)

#### Joint Configuration
- **fr3_joint1-7**: 7-DOF arm joints (revolute)
- **fr3_finger_joint1-2**: 2-DOF gripper joints (prismatic, mimic joint)

#### Joint Limits (from joint_limits.yaml)
- **joint1**: [-2.3093, 2.3093] rad, effort: 87N·m
- **joint2**: [-1.5133, 1.5133] rad, effort: 87N·m  
- **joint3**: [-2.4937, 2.4937] rad, effort: 87N·m
- **joint4**: [-2.7478, -0.4461] rad, effort: 87N·m
- **joint5**: [-2.48, 2.48] rad, effort: 12N·m
- **joint6**: [0.8521, 4.2094] rad, effort: 12N·m
- **joint7**: [-2.6895, 2.6895] rad, effort: 12N·m
- **Gripper**: [0, 0.04] m opening range, effort: 100N

#### Visual Quality
- **High-fidelity meshes**: Converted from official Franka DAE visual meshes
- **Accurate geometry**: Based on CAD models from franka_description
- **Realistic materials**: Proper surface properties for rendering

#### Physics Properties
- **Collision meshes**: STL files from franka_description/meshes/collision
- **Inertial properties**: Accurate mass, center of mass, and inertia from URDF
- **Joint dynamics**: Realistic damping and friction parameters
- **Actuator limits**: Force/torque limits matching real FR3 specifications

#### File Structure
```
fr3con.xml                                # Main model file
catkin_ws/src/franka_ros/franka_description/meshes/collision/  # Collision meshes
```

## Troubleshooting

### Dependencies

If you encounter import errors, ensure all required packages are installed:

```bash
pip install mujoco dm-control numpy scipy matplotlib ikpy transforms3d
```

## Scene Configuration

The project includes `scene.xml` which provides:
- **Environment setup**: Floor, lighting, and skybox
- **Visual enhancements**: Professional lighting and materials
- **Camera positioning**: Optimal viewing angles
- **Model integration**: Includes the official FR3 model (`fr3con.xml`)

## Movement Demo Features

The `movement_demo.py` script showcases:
- **Smooth interpolation**: Ease-in/ease-out motion between poses
- **State machine**: Professional pose management and transitions
- **Configurable timing**: 3 seconds hold + 2 seconds transition
- **Real-time feedback**: Console output showing movement progress
- **High-frequency updates**: 100 Hz simulation rate for smooth visuals
- **Infinite cycling**: Continuous demonstration loop

**Pose Sequence:**
1. Home position (standard FR3 configuration)
2. Forward reach (extending toward workspace)
3. Right sweep (demonstrating workspace range)
4. Left sweep (showing full lateral motion)
5. Upright position (vertical reach demonstration)
6. Return to home (completing the cycle)

## Advanced Usage

### Integration with ROS

The environment can be integrated with ROS topics for real-robot compatibility:

```python
# Example ROS integration
import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    env.set_joint_positions(np.array(msg.position[:7]))
```

## Model Generation Process

This official FR3 model was created through the following process:

1. **URDF Generation**: Used xacro to expand official franka_description with hand:=true
   ```bash
   export ROS_PACKAGE_PATH=.../franka_ros
   xacro fr3.urdf.xacro hand:=true > fr3_with_gripper.urdf
   ```

2. **Mesh Path Resolution**: Fixed package:// paths to absolute paths for MuJoCo

3. **MJCF Export**: Loaded URDF in MuJoCo and exported canonical MJCF
   ```python
   model = mujoco.MjModel.from_xml_path('fr3_with_gripper_fixed.urdf')
   mujoco.mj_saveLastXML('fr3con.xml', model)
   ```

4. **Visual Enhancement**: 
   - Converted DAE visual meshes to STL using trimesh
   - Added visual geoms with converted meshes
   - Kept collision geoms with original STL meshes

5. **Joint Limits**: Preserved official limits from joint_limits.yaml

## Contributing

This environment was built using:
- Official Franka URDF/xacro files from franka_ros package
- Official visual and collision meshes from franka_description  
- Official joint limits from joint_limits.yaml
- MuJoCo physics engine with native MJCF format
- Python mesh processing (trimesh) for DAE→STL conversion

The model maintains full compatibility with the official Franka specifications.

## License

This project uses the official Franka Emika URDF files and follows their licensing terms.
