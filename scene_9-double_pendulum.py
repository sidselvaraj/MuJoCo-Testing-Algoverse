import mujoco
import mujoco.viewer
import numpy as np
import time  # Import time module for delay

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path("double_pendulum.xml")
data = mujoco.MjData(model)

# Set Initial Position (Ensure the Arms Start Tilted)
data.qpos[0] = np.deg2rad(30)  # First arm tilted 30 degrees
data.qpos[1] = np.deg2rad(-20)  # Second arm tilted -20 degrees

# Set Initial Velocity (Give a Push for Natural Swing)
data.qvel[0] = 2.0  # Push first arm
data.qvel[1] = -1.5  # Push second arm

# Run simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)  # Step simulation forward
        time.sleep(0.006)  # Add delay to slow down the simulation
        viewer.sync()  # Update viewer

