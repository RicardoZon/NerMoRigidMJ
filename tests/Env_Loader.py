import mujoco.viewer as viewer		# trg_x = originPoint[0] + ratio * ovalRadius[0] *math.cos(cur_radian)
		# trg_y = originPoint[1] + ratio * ovalRadius[1] *math.sin(cur_radian)
import mujoco

# m = mujoco.MjModel.from_xml_path("../models/dynamic_4l.xml")
m = mujoco.MjModel.from_xml_path("../models/fl_single.xml")
# m = mujoco.MjModel.from_xml_path("../models/tail_assets/tail_")
d = mujoco.MjData(m)
# viewer.launch(m, d)


viewer.launch()