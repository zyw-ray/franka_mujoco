import os
import mujoco
import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import transforms3d as tf
 
def main():
    # Prefer project MJCF if present to infer TCP offset
    tcp_offset = 0.1034
    if os.path.exists("fr3con.xml"):
        try:
            mj_model = mujoco.MjModel.from_xml_path("fr3con.xml")
            mj_data = mujoco.MjData(mj_model)
            mujoco.mj_forward(mj_model, mj_data)
            site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
            if site_id != -1:
                # Use Z distance from wrist to site as TCP offset estimate
                tcp_offset = float(mj_data.site_xpos[site_id][2] - mj_data.body_xpos[mj_model.body("fr3_link7")][2])
        except Exception:
            pass

    urdf_candidates = ["fr3_with_gripper.urdf", "fr3_arm_only.urdf"]
    urdf_path = next((p for p in urdf_candidates if os.path.exists(p)), None)
    if urdf_path is None:
        raise FileNotFoundError("No FR3 URDF found: expected fr3_with_gripper.urdf or fr3_arm_only.urdf")

    # Build IK chain from URDF, using base link and inferred TCP offset
    my_chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path,
        base_elements=["fr3_link0"],
        last_link_vector=[0, 0, tcp_offset],
        base_element_type="link",
    )
    ee_pos = [-0.13, 0.5, 0.1] # 末端执行器在世界坐标系下的目标位置的xyz 
    ee_euler = [3.14, 0, 1.57] # 末端执行器的目标欧拉角姿态（弧度）
    ee_orientation = tf.euler.euler2mat(*ee_euler) # 将欧拉角姿态用旋转矩阵表示
    ref_pos = None # Let ikpy use default initial guess
    
    fig, ax = plot_utils.init_3d_figure()
    solution = my_chain.inverse_kinematics(ee_pos, ee_orientation, "all", initial_position=ref_pos)
    print("IK solution (radians):", solution)
    my_chain.plot(solution, ax)
    plt.show()
 
if __name__ == "__main__":
    main()
