import mujoco
import numpy as np
import os
import time


def get_site_id(model: mujoco.MjModel, name: str) -> int:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if sid == -1:
        raise ValueError(f"Site '{name}' not found in model")
    return sid


def get_site_pose(model: mujoco.MjModel, data: mujoco.MjData, site_id: int):
    pos = data.site_xpos[site_id].copy()
    mat = data.site_xmat[site_id].reshape(3, 3).copy()
    return pos, mat


def main():
    # Resolve model path
    candidates = ["scene.xml", "basic_scene.xml", "fr3con.xml"]
    model_path = next((p for p in candidates if os.path.exists(p)), None)
    if model_path is None:
        raise FileNotFoundError("No scene.xml, basic_scene.xml, or fr3con.xml found")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Use the attachment site on the wrist as the EE frame
    site_name = "attachment_site"
    site_id = get_site_id(model, site_name)

    # Step once to initialize kinematics
    mujoco.mj_forward(model, data)
    pos, mat = get_site_pose(model, data, site_id)
    print(f"Initial EE site '{site_name}' pose:")
    print(f"  position: {pos}")
    print(f"  xmat:\n{mat}")

    # Simple Cartesian demo: move EE in a small circle in XY while holding Z
    center = pos.copy()
    radius = 0.03
    hz = 200
    dt = 1.0 / hz
    duration = 4.0
    t0 = time.time()

    print("\nCommanding joint-space controls to trace small EE circle (approximate)...")

    while time.time() - t0 < duration:
        t = time.time() - t0
        # Very simple heuristic: sinusoidally vary first three joints to induce EE circle
        data.ctrl[0] = 0.2 * np.sin(2 * np.pi * 0.5 * t)
        data.ctrl[1] = -0.3 + 0.15 * np.sin(2 * np.pi * 0.5 * t + np.pi / 2)
        data.ctrl[2] = 0.2 * np.cos(2 * np.pi * 0.5 * t)

        mujoco.mj_step(model, data)

        if int(t * hz) % 40 == 0:  # print at ~5 Hz
            p, _ = get_site_pose(model, data, site_id)
            print(f"t={t:4.2f}s  EE pos: {p}")

        time.sleep(dt)

    print("\nCartesian demo complete.")


if __name__ == "__main__":
    main()




