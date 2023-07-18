import math
import matplotlib.pyplot as plt
import numpy as np

class MotionLength(object):
	"""docstring for MotionLength"""
	def __init__(self, ls, lh, ll):
		super(MotionLength, self).__init__()
		self.lS = ls
		self.lH = lh
		self.ll = ll

		self.period = 2/2
		sL = 1000
		self.tD = 2*math.pi / sL
		
		self.rtList = []
		self.ltList = []
		tList = []
		for i in range(sL+1):
			ct = i*self.tD
			if ct > math.pi*self.period:
				break
			tList.append(i*self.tD)
			self.rtList.append(math.cos(ct))
			self.ltList.append(self.getLt(ct))
		self.tL = len(tList)
		
		#print(max(self.rtList), " --- ", min(self.rtList))
		#print(max(self.ltList), " --- ", min(self.ltList))
	
	def getLt(self, ct):
		lt = ct/self.period - math.sin(ct/self.period)
		lt = self.ll*(1-2*lt/math.pi)
		return lt

	def getStepLength(self, alpha):
		beta = alpha
		lt = self.ll
		if beta == 0:
			return lt
		
		lM = lt*math.cos(beta) + self.lH*math.sin(beta)
		lM = lM + self.lS - self.lS/beta * math.sin(beta)
		return lM

if __name__ == '__main__':
	#ls = 48
	#lh = 39
	ls = 56
	lh = 40
	fl = 10#20
	hl = 10#30
	stepLeg = MotionLength(ls, lh, hl)
	rList = []
	for i in range(16):
		curA = (i)*5
		alpha = curA*math.pi/180
		vt = fl + stepLeg.getStepLength(alpha)
		vo = fl + hl
		rList.append([curA, vt/vo])

	print(rList)


	# 0.50Hz
	v50 = [[0, 0.03947457561561242],
	[5,  0.04495932711464506],
	[10, 0.048480501081004206],
	[15, 0.049022135293645845],
	[20, 0.049625986119386845],
	[25, 0],
	[30, 0],
	[35, 0]]
	# 0.67Hz
	v40 = [[0, 0.01],
	[5,  0],
	[10, 0],
	[15, 0],
	[20, 0],
	[25, 0],
	[30, 0],
	[35, 0]]
	# 0.80Hz
	v33 = [[0, 0.02796175327217167],
	[5,  0.033732228099566317],
	[10, 0.037163189097996686],
	[15, 0.04185797960443065],
	[20, 0.04547670852910837],
	[25, 0.04859014384417703],
	[30, 0.05183340981990166],
	[35, 0.05345241905661865]]

	# 1.00Hz
	v25 = [[0, 0.01],
	[5,  0],
	[10, 0],
	[15, 0],
	[20, 0],
	[25, 0],
	[30, 0],
	[35, 0]]
	# 1.25Hz
	v20 = [[0, 0.01],
	[5,  0],
	[10, 0],
	[15, 0],
	[20, 0],
	[25, 0],
	[30, 0],
	[35, 0]]

	print(0.04304453437317624/0.03029587021261925)
	print(rList[2])
	"""
	,
	[20,],
	[25,],
	[30,]]
	"""
	aList = []
	rCal = []

	rTest50 = []
	rTest40 = []
	rTest33 = []
	rTest25 = []
	rTest20 = []
	for i in range(len(v33)):
		aList.append(i*5)
		rCal.append(rList[i][1])
		rTest50.append(v50[i][1]/v50[0][1])
		rTest40.append(v40[i][1]/v40[0][1])
		rTest33.append(v33[i][1]/v33[0][1])
		rTest25.append(v25[i][1]/v25[0][1])
		rTest20.append(v20[i][1]/v20[0][1])

	plt.plot(aList,rCal, linewidth=3, label='Theoretical value', linestyle="--")
	plt.plot(aList,rTest50, linewidth=3, label='50Hz')
	plt.plot(aList,rTest40, linewidth=3, label='40Hz')
	plt.plot(aList,rTest33, linewidth=3, label='33Hz')
	plt.plot(aList,rTest25, linewidth=3, label='25Hz')
	plt.plot(aList,rTest20, linewidth=3, label='20Hz')

	#xTicks = np.arange(0, 5, 50)
	yTicks = np.arange(0, 2.5, 0.25)
	#plt.xticks(xTicks)
	plt.yticks(yTicks)
	plt.xticks(fontsize=20)
	plt.yticks(fontsize=20)
	plt.legend(fontsize=20)
	plt.xlabel('Amplitude of spine swing (degree)', fontsize=20)
	plt.ylabel('Speedup ratio for robot motion', fontsize=20)
	plt.grid()
	plt.show()