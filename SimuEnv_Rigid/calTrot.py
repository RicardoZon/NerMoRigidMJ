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
		beta = 2*alpha
		lt = self.ll
		if beta == 0:
			return lt
		
		lM = lt*math.cos(beta) + self.lH*math.sin(beta)
		lM = lM + self.lS - self.lS/beta * math.sin(beta)
		return lM

if __name__ == '__main__':
	ls = 40
	lh = 36
	#ls = 56
	#lh = 40
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
	v050 = [[0, 0.01734628657135246],
	[5,  0.02290776848797434],
	[10, 0.027964840442169597],
	[15, 0.032330191615207955],
	[20,  0.03285966341996365],
	[25, 0],
	[30, 0],
	[35, 0]]
	# 0.67Hz
	v067 = [[0, 0.02356543191452682],
	[5,  0.03168035789129912],
	[10, 0.03786616879871207],
	[15, 0.04381063324174377],
	[20, 0.04422652256006014],
	[25, 0],
	[30, 0],
	[35, 0]]
	# 0.80Hz
	v080 = [[0, 0.02796175327217167],
	[5,  0.033732228099566317],
	[10, 0.037163189097996686],
	[15, 0.04185797960443065],
	[20, 0.04547670852910837],
	[25, 0.04859014384417703],
	[30, 0.05183340981990166],
	[35, 0.05345241905661865]]

	# 1.00Hz
	v100 = [[0, 0.01],
	[5,  0],
	[10, 0],
	[15, 0],
	[20, 0],
	[25, 0],
	[30, 0],
	[35, 0]]
	# 1.25Hz
	v125 = [[0, 0.01],
	[5,  0],
	[10, 0],
	[15, 0],
	[20, 0],
	[25, 0],
	[30, 0],
	[35, 0]]


	aList = []
	rCal = []

	rTest050 = []
	rTest067 = []
	rTest080 = []
	rTest100 = []
	rTest125 = []
	for i in range(len(v050)):
		aList.append(i*5)
		rCal.append(rList[i][1])
		rTest050.append(v050[i][1]/v050[0][1])
		rTest067.append(v067[i][1]/v067[0][1])
		rTest080.append(v080[i][1]/v080[0][1])
		rTest100.append(v100[i][1]/v100[0][1])
		rTest125.append(v125[i][1]/v125[0][1])

	plt.plot(aList,rCal, linewidth=3, label='Theoretical value', linestyle="--")
	plt.plot(aList,rTest050, linewidth=3, label='0.50Hz')
	plt.plot(aList,rTest067, linewidth=3, label='0.67Hz')
	plt.plot(aList,rTest080, linewidth=3, label='0.80Hz')
	plt.plot(aList,rTest100, linewidth=3, label='1.00Hz')
	plt.plot(aList,rTest125, linewidth=3, label='1.25Hz')

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