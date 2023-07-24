from ToSim import SimModel
from Controller import MouseController
import matplotlib.pyplot as plt
import time

# --------------------
RUN_TIME_LENGTH = 20
if __name__ == '__main__':
	fre = 0.67  # 0.5
	time_step = 0.002
	spine_angle = 0 #20
	run_steps_num = int(RUN_TIME_LENGTH / time_step)

	theMouse = SimModel("../models/dynamic_4l.xml")
	theController = MouseController(fre, time_step, spine_angle)

	for i in range(10):
		#ctrlData = 0
		ctrlData = [0.0, 1, 0.0, 1, 0.0, 1, 0.0, 1, 0,0,0,0]
		theMouse.runStep(ctrlData, time_step)
	theMouse.initializing()
	start = time.time()
	for i in range(run_steps_num):
		#print("Step --> ", i)
		tCtrlData = theController.runStep()				# No Spine
		#tCtrlData = theController.runStep_spine()		# With Spine
		ctrlData = tCtrlData
		theMouse.runStep(ctrlData, time_step)
	end = time.time()
	timeCost = end-start
	print("Time -> ", timeCost)

	dis = theMouse.drawPath()
	print("py_v --> ", dis/timeCost)
	print("sim_v --> ", dis/(run_steps_num*time_step))
	theMouse.savePath("own_125")

	#'''
	fig, axs = plt.subplots(2,2)
	subTitle = ["Fore Left Leg", "Fore Right Leg",
		"Hind Left Leg", "Hind Right Leg"]
	for i in range(4):
		pos_1 = int(i/2)
		pos_2 = int(i%2)
		print(pos_1, pos_2)
		axs[pos_1,pos_2].set_title(subTitle[i])
		axs[pos_1,pos_2].plot(theController.trgXList[i], theController.trgYList[i])
		axs[pos_1,pos_2].plot(theMouse.legRealPoint_x[i], theMouse.legRealPoint_y[i])

	plt.show()
	#'''

	'''
	plt.plot(theController.trgXList[0], theController.trgYList[0], label='Target trajectory')
	plt.plot(theMouse.legRealPoint_x[0], theMouse.legRealPoint_y[0], label='Real trajectory ')
	plt.legend()
	plt.xlabel('y-coordinate (m)')
	plt.ylabel('z-coordinate (m)')
	plt.grid()
	plt.show()
	'''