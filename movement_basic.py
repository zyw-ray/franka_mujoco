import mujoco.viewer
import time
 
def main():
    model = mujoco.MjModel.from_xml_path('basic_scene.xml')
    data = mujoco.MjData(model)
    data.ctrl[:7] = [0, 0.607, 0, -2.12, -0.14, 4.52, -3.02]
 
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.02) # slower the render speed
 
if __name__ == "__main__":
    main()
