# from mujoco_py import load_model_from_path, MjSim, MjViewer
import matplotlib.pyplot as plt
import numpy as np
import math
import mujoco
import mujoco.viewer as viewer

import time
class SimModel(object):
	"""docstring for SimModel"""
	def __init__(self, modelPath, render=False):
		super(SimModel, self).__init__()
		self.model = mujoco.MjModel.from_xml_path(modelPath)
		self.data = mujoco.MjData(self.model)
		self._timestep = self.model.opt.timestep

		self.render = render
		if render:
			self.viewer = viewer.launch_passive(self.model, self.data)
			self.viewer.cam.azimuth = 0
			self.viewer.cam.lookat[0] += 0.25
			self.viewer.cam.lookat[1] += -0.5
			self.viewer.cam.distance = self.model.stat.extent * 0.5
			# self.viewer.run_speed = 2  # changed

		# self.sim_state = self.sim.get_state()
		# self.sim.set_state(self.sim_state)
		self.legPosName = [
			["router_shoulder_fl", "foot_s_fl"],
			["router_shoulder_fr", "foot_s_fr"],
			["router_hip_rl", "foot_s_rl"],
			["router_hip_rr", "foot_s_rr"]]
		self.fixPoint = "body_ss"#"neck_ss"
		self.legRealPoint_x = [[],[],[],[]]
		self.legRealPoint_y = [[],[],[],[]]
		self.movePath = [[],[],[]]

	def initializing(self):
		self.movePath = [[],[],[]]
		self.legRealPoint_x = [[],[],[],[]]
		self.legRealPoint_y = [[],[],[],[]]

	def do_simulation(self, ctrl, n_frames):
		# ------------------------------------------ #
		# ID 0, 1 left-fore leg and coil 
		# ID 2, 3 right-fore leg and coil
		# ID 4, 5 left-hide leg and coil
		# ID 6, 7 right-hide leg and coil
		# Note: For leg, it has [-1: front; 1: back]
		# Note: For fore coil, it has [-1: leg up; 1: leg down]
		# Note: For hide coil, it has [-1: leg down; 1: leg up]
		# ------------------------------------------ #
		# ID 08 is neck		(Horizontal)
		# ID 09 is head		(vertical)
		# ID 10 is spine	(Horizontal)  [-1: right, 1: left]
		# Note: range is [-1, 1]
		# ------------------------------------------ #
		self.data.ctrl[:] = ctrl
		for _ in range(n_frames):
			# step_start = time.time()
			mujoco.mj_step(self.model, self.data)
			if self.render:
				self.viewer.sync()
			# 	time_until_next_step = self._timestep - (time.time() - step_start)
			# 	print(time_until_next_step)
			# 	if time_until_next_step > 0:
			# 		time.sleep(time_until_next_step)
		'''
		tData = self.sim.data.get_site_xpos(self.fixPoint)
		for i in range(3):
			self.movePath[i].append(tData[i])
		for i in range(4):
			originPoint = self.sim.data.get_site_xpos(self.legPosName[i][0])
			currentPoint = self.sim.data.get_site_xpos(self.legPosName[i][1])
			#print(originPoint, currentPoint)
			tX = currentPoint[1]-originPoint[1]
			tY = currentPoint[2]-originPoint[2]
			self.legRealPoint_x[i].append(tX)
			self.legRealPoint_y[i].append(tY)
		'''

	def getTime(self):
		return self.data.time

	def point_distance_line(self, point,line_point1,line_point2):
		#计算向量
		vec1 = line_point1 - point
		vec2 = line_point2 - point
		distance = np.abs(np.cross(vec1,vec2)) / np.linalg.norm(line_point1-line_point2)
		return distance

	def drawPath(self):
		#print(self.movePath)
		path_X = self.movePath[0]
		path_Y = self.movePath[1]
		tL = len(path_X)

		"""
		order = 3
		tY = []
		for i in range(tL):
			tY.append(-path_Y[i])
		parameter = np.polyfit(tY, path_X, order)
		smooth_x = []
		for i in range(tL):
			tVal = 0
			for j in range(order):
				tVal = tVal + parameter[order] * tY[i] ** j
			smooth_x.append(tVal)
		dis = 0
		for i in range(tL-1):
			dX = smooth_x[i]-smooth_x[i+1]
			dY = path_Y[i]-path_Y[i+1]
			dis = dis + math.sqrt(dX*dX + dY*dY)
		print("Dis --> ", dis)
		"""

		ds = 1
		dL = int(tL/ds)
		check_x = []
		check_y = []
		print(tL)
		for i in range(dL):
			check_x.append(path_X[i*ds])
			check_y.append(path_Y[i*ds])

		check_x.append(path_X[-1])
		check_y.append(path_Y[-1])

		#dis = 50
		#for i in range(dL):
		#	dX = check_x[i]-check_x[i+1]
		#	dY = check_y[i]-check_y[i+1]
		#	dis = dis + math.sqrt(dX*dX + dY*dY)
		#dis = path_Y[0] - path_Y[-1]
		dX = path_X[0]-path_X[-1]
		dY = path_Y[0]-path_Y[-1]
		dis = math.sqrt(dX*dX + dY*dY)
		#dis = path_Y[0] - path_Y[-1]
		print("Dis --> ", dis)

		start_p = np.array([check_x[0], check_y[0]])
		end_p = np.array([check_x[-1], check_y[-1]])
		
		maxDis = 0
		for i in range(tL):
			cur_p = np.array([path_X[i], path_Y[i]])
			tDis = self.point_distance_line(cur_p, start_p, end_p)
			if tDis > maxDis:
				maxDis = tDis
		print("MaxDiff --> ", maxDis)
		plt.plot(path_X, path_Y)
		plt.plot(check_x, check_y)
		plt.grid()
		plt.show()

		return dis

	def savePath(self, flag):
		filePath = "Data/path_"+flag+".txt"
		trajectoryFile = open(filePath, 'w')
		dL = len(self.movePath[0])
		for i in range(dL):
			for j in range(3):
				trajectoryFile.write(str(self.movePath[j][i])+' ')
			trajectoryFile.write('\n')
		trajectoryFile.close()


if __name__ == '__main__':
	import os
	RUN_TIME_LENGTH = 20  # seconds
	fre = 0.67
	from Controller import MouseController
	theMouse = SimModel("../models/dynamic_4l.xml", render=True)
	dt = 0.01
	n_frames = int(dt / theMouse._timestep)
	run_steps_num = int(RUN_TIME_LENGTH / dt)
	theController = MouseController(fre, dt, 0)
	mujoco.mj_resetData(theMouse.model, theMouse.data)

	for i in range(10):  # 0.2 s
		ctrlData = [0.0, 1, 0.0, 1, 0.0, 1, 0.0, 1, 0, 0, 0, 0]
		# ctrlData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]  # Init
		theMouse.do_simulation(ctrlData, n_frames=1)
	theMouse.initializing()
	start = time.time()
	for i in range(run_steps_num):
		tCtrlData = theController.runStep()  # No Spine
		# tCtrlData = theController.runStep_spine()		# With Spine
		ctrlData = tCtrlData
		theMouse.do_simulation(ctrlData, n_frames=n_frames)
		# print(theMouse.data.time)
	end = time.time()

	timeCost = end - start
	print("Time -> ", timeCost)

	# dis = theMouse.drawPath()
	# print("py_v --> ", dis / timeCost)
	# print("sim_v --> ", dis / (run_steps_num * time_step))
	# theMouse.savePath("own_125")

	# '''
	fig, axs = plt.subplots(2, 2)
	subTitle = ["Fore Left Leg", "Fore Right Leg",
				"Hind Left Leg", "Hind Right Leg"]
	for i in range(4):
		pos_1 = int(i / 2)
		pos_2 = int(i % 2)
		print(pos_1, pos_2)
		axs[pos_1, pos_2].set_title(subTitle[i])
		axs[pos_1, pos_2].plot(theController.trgXList[i], theController.trgYList[i])
		axs[pos_1, pos_2].plot(theMouse.legRealPoint_x[i], theMouse.legRealPoint_y[i])

	plt.show()
	# '''

	'''
	plt.plot(theController.trgXList[0], theController.trgYList[0], label='Target trajectory')
	plt.plot(theMouse.legRealPoint_x[0], theMouse.legRealPoint_y[0], label='Real trajectory ')
	plt.legend()
	plt.xlabel('y-coordinate (m)')
	plt.ylabel('z-coordinate (m)')
	plt.grid()
	plt.show()
	'''



