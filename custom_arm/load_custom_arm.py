import mujoco
import mujoco.viewer
import time

xml_path = "custom_arm.xml"

print(f"Loading custom arm model from {xml_path}...")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

print("Launching MuJoCo viewer. Press ESC to close.")
mujoco.viewer.launch(model, data)
