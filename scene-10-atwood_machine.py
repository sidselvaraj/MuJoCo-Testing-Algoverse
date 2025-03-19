import mujoco
import mujoco.viewer
import time

# Load model
model = mujoco.MjModel.from_xml_path("atwood_machine.xml")
data = mujoco.MjData(model)

# Create viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    for _ in range(99999):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

print("Simulation complete.")
