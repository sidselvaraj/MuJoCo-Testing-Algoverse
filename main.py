import mujoco
import mujoco.viewer

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path("slope_box.xml")
data = mujoco.MjData(model)
start_pos = data.qpos[:].copy()  # Store initial state



# Prototypes
def move_object(name, x, y, z):
    obj_id = model.body(name).id 
    #sets x y and z positions for box
    data.qpos[obj_id * 7] = x
    data.qpos[obj_id * 7 + 1] = y
    data.qpos[obj_id * 7 + 2] = z
    mujoco.mj_forward(model, data)  # Apply update

def get_position(name):
    obj_id = model.body(name).id
    return (data.qpos[obj_id * 7], data.qpos[obj_id * 7 + 1], data.qpos[obj_id * 7 + 2]), data.time


def reset_sim():
    """Reset the simulation."""
    data.qpos[:] = start_pos  # Reset position
    data.qvel[:] = 0  # Stop movement
    mujoco.mj_forward(model, data)

# testing stuff
''''
move_object("box", 0.5, 0.5, 1.0)
print(get_position("box"))
reset_sim()
'''

with mujoco.viewer.launch_passive(model, data) as viewer:
    for _ in range(80000): 
        mujoco.mj_step(model, data)
        viewer.sync()
