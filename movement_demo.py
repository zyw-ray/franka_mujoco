import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    model = mujoco.MjModel.from_xml_path('basic_scene.xml')
    data = mujoco.MjData(model)
    
    # Define keyframe poses for the arm movement demo
    poses = [
        # Home position
        [0, 0, 0, -1.57, 0, 1.57, -0.785],
        # Pose 1: Reach forward
        [0, -0.3, 0, -2.2, 0, 1.9, 0.785],
        # Pose 2: Move right
        [0.8, -0.3, 0.3, -2.0, 0.5, 1.5, 0],
        # Pose 3: Move left  
        [-0.8, -0.3, -0.3, -2.0, -0.5, 1.5, 0],
        # Pose 4: Upright position
        [0, -0.8, 0, -1.0, 0, 0.8, 0.785],
        # Back to home
        [0, 0, 0, -1.57, 0, 1.57, -0.785],
    ]
    
    # Timing parameters
    pose_duration = 3.0  # seconds to hold each pose
    transition_duration = 2.0  # seconds to transition between poses
    
    current_pose_idx = 0
    target_pose = np.array(poses[0])
    current_joint_pos = np.array(poses[0])
    
    # Initialize robot at first pose
    data.qpos[:7] = current_joint_pos
    data.ctrl[:7] = current_joint_pos
    
    start_time = time.time()
    last_transition_time = start_time
    in_transition = False
    transition_start_pose = current_joint_pos.copy()

    print("FR3 Arm Movement Demo")
    print("====================")
    print("The robot will move through several poses:")
    print("1. Home position")
    print("2. Reach forward") 
    print("3. Move right")
    print("4. Move left")
    print("5. Upright position")
    print("6. Return home")
    print("\nPress Ctrl+C to exit")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            current_time = time.time()
            elapsed = current_time - last_transition_time
            
            # State machine for pose transitions
            if not in_transition and elapsed >= pose_duration:
                # Start transition to next pose
                in_transition = True
                transition_start_pose = current_joint_pos.copy()
                current_pose_idx = (current_pose_idx + 1) % len(poses)
                target_pose = np.array(poses[current_pose_idx])
                last_transition_time = current_time
                print(f"Moving to pose {current_pose_idx + 1}")
                
            elif in_transition and elapsed >= transition_duration:
                # Finish transition
                in_transition = False
                current_joint_pos = target_pose.copy()
                last_transition_time = current_time
                print(f"Reached pose {current_pose_idx + 1}")
                
            elif in_transition:
                # Interpolate during transition
                t = elapsed / transition_duration
                # Use smooth interpolation (ease in/out)
                t_smooth = 0.5 * (1 - np.cos(np.pi * t))
                current_joint_pos = transition_start_pose + t_smooth * (target_pose - transition_start_pose)
            
            # Set joint positions
            data.ctrl[:7] = current_joint_pos
            
            # Step simulation
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)  # 100 Hz update rate

if __name__ == "__main__":
    main()




