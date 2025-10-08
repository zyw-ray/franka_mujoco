import mujoco
import os
import numpy as np
import time


def print_joint_info(model: mujoco.MjModel) -> None:
    num_joints = model.njnt
    print(f"Total joints: {num_joints}")
    print("Index | Name              | Type     | Range (if limited)")
    print("------ | ----------------- | -------- | -------------------")

    for j in range(num_joints):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint_{j}"
        jtype = model.jnt_type[j]
        jtype_str = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(int(jtype), str(int(jtype)))
        limited = model.jnt_limited[j]
        rng = "-"
        if limited:
            rng = f"{model.jnt_range[j, 0]:.4f} {model.jnt_range[j, 1]:.4f}"
        print(f"{j:5d} | {name:<17} | {jtype_str:<8} | {rng}")


def print_actuator_mapping(model: mujoco.MjModel) -> None:
    num_act = model.nu
    print(f"\nTotal actuators: {num_act}")
    print("Index | Name              | Type       | Joint")
    print("------ | ----------------- | ---------- | -----")
    for a in range(num_act):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or f"act_{a}"
        # In compiled model, XML <position>/<velocity>/<motor> typically map to 'general' actuators
        atype_str = "general"
        j = int(model.actuator_trnid[a, 0])
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) if j != -1 else "-"
        print(f"{a:5d} | {name:<17} | {atype_str:<10} | {jname}")


def simple_motion(model: mujoco.MjModel, seconds: float = 3.0) -> None:
    data = mujoco.MjData(model)
    t0 = time.time()
    while time.time() - t0 < seconds:
        # Generate a small sinusoidal motion for the first 7 joints
        t = time.time() - t0
        target = 0.2 * np.sin(2 * np.pi * 0.5 * t) * np.ones(7)
        data.ctrl[:7] = target
        mujoco.mj_step(model, data)
    print("\nSimple motion complete.")


def main():
    # Prefer scene.xml, then basic_scene.xml, then raw model fr3con.xml
    candidates = [
        "scene.xml",
        "basic_scene.xml",
        "fr3con.xml",
    ]
    model_path = next((p for p in candidates if os.path.exists(p)), None)
    if model_path is None:
        raise FileNotFoundError("No scene.xml, basic_scene.xml, or fr3con.xml found in project root")

    model = mujoco.MjModel.from_xml_path(model_path)
    print_joint_info(model)
    print_actuator_mapping(model)
    simple_motion(model, seconds=3.0)


if __name__ == "__main__":
    main()


