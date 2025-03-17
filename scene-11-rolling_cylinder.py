import mujoco
import mujoco.viewer
import time

# Load the MuJoCo model
model_path = "rolling_cylinder.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Create a viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running() and time.time() - start_time < 100:  # Run for 100 seconds
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)  # Slows down the simulation
