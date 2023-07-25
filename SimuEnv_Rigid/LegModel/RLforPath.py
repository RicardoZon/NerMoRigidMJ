import math
import numpy as np

class LegPath(object):
	"""docstring for ForeLegPath"""
	def __init__(self, pathType="circle"):
		super(LegPath, self).__init__()
		'''
		self.para_FU = [[-0.005, -0.05], [0.03, 0.01]]
		self.para_FD = [[-0.005, -0.05], [0.03, 0.005]]
		self.para_HU = [[0.00, -0.05], [0.03, 0.01]]
		self.para_HD = [[0.00, -0.05], [0.03, 0.005]]
		'''
		# Default Stable
		# self.para_FU = [[-0.00, -0.045], [0.03, 0.01]]
		# self.para_FD = [[-0.00, -0.045], [0.03, 0.005]]
		# self.para_HU = [[-0.005, -0.05], [0.03, 0.01]]
		# self.para_HD = [[-0.005, -0.05], [0.03, 0.005]]

		self.para_FU = [[-0.00, -0.045], [0.03, 0.01]]
		self.para_FD = [[-0.00, -0.045], [0.03, 0.005]]
		self.para_HU = [[-0.005, -0.05], [0.03, 0.01]]
		self.para_HD = [[-0.005, -0.05], [0.03, 0.005]]


	def getOvalPathPoint(self, radian, leg_flag, halfPeriod, ratio=1.0):
		pathParameter = None
		cur_radian = 0
		if leg_flag == "F":
			if radian < halfPeriod*math.pi:
				pathParameter = self.para_FU
				cur_radian = radian/halfPeriod
			else:
				pathParameter = self.para_FD
				cur_radian = (radian)/(2-halfPeriod)
		else:
			if radian < halfPeriod*math.pi:
				pathParameter = self.para_HU
				cur_radian = radian/halfPeriod
			else:
				pathParameter = self.para_HD
				cur_radian = (radian)/(2-halfPeriod)

		originPoint = pathParameter[0]
		ovalRadius = pathParameter[1]

		trg_x = originPoint[0] + ratio * ovalRadius[0] *math.cos(cur_radian)
		trg_y = originPoint[1] + ratio * ovalRadius[1] *math.sin(cur_radian)
		# trg_x = originPoint[0] + ovalRadius[0] * math.cos(cur_radian)
		# trg_y = originPoint[1] + ovalRadius[1] * math.sin(cur_radian)
		return [trg_x, trg_y]
