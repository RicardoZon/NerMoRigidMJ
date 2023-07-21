import mujoco.viewer as viewer
import mujoco

m = mujoco.MjModel.from_xml_path("../models/dynamic_4l.xml")
# m = mujoco.MjModel.from_xml_path("../models/tail_assets/tail_")
d = mujoco.MjData(m)
viewer.launch(m, d)


# viewer.launch()