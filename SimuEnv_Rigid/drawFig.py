import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import numpy as np
import math

def getPathData(flag):
	filePath = "Data/path_"+flag+".txt"
	inputFile = open(filePath,"r")
	dataList = []

	inputLines = inputFile.readlines()
	for line in inputLines:
		lineStrData = line.split(' ')
		dataNum = len(lineStrData) - 1
		lineData = []
		for i in range(dataNum):
			lineData.append(float(lineStrData[i]))
		dataList.append(lineData)

	# --------------------- #
	tL = len(dataList)
	returnList = [[],[]]
	for i in range(tL):
		returnList[0].append(dataList[i][0])
		returnList[1].append(dataList[i][1])
	# --------------------- #	
	return returnList
def getDis(tList):
	diff_x = tList[0][0]-tList[0][1]
	diff_y = tList[1][0]-tList[1][1]

	tL = math.sqrt(diff_x*diff_x + diff_y+diff_y)
	return tL

if __name__ == '__main__':
	hz20_a0 = getPathData("s_20_0")
	hz50_a0 = getPathData("s_50_0")
	hz50_a30 = getPathData("s_50_30")

	for tD in range(len(hz50_a30[0])):
		hz50_a30[0][tD] = hz50_a30[0][tD]+0.03
		hz50_a30[1][tD] = 1-hz50_a30[1][tD]
		hz50_a0[1][tD] = 1-hz50_a0[1][tD]
		hz20_a0[1][tD] = 1-hz20_a0[1][tD]

	plt.figure(figsize=(8, 4))
	#plt.plot(hz20_a0[1][100:],hz20_a0[0][100:], linewidth=3, label=r'f = 20 Hz, $\alpha = 0^\circ$')
	#plt.plot(hz50_a0[1][100:],hz50_a0[0][100:], linewidth=3, label=r'f = 50 Hz, $\alpha = 0^\circ$')
	#plt.plot(hz50_a30[1][100:],hz50_a30[0][100:], linewidth=3, label=r'f = 50 Hz, $\alpha = 30^\circ$')
	plt.plot(hz20_a0[1][100:],hz20_a0[0][100:], linewidth=3, label=r'f = 0.625 Hz, $\alpha = 0^\circ$')
	plt.plot(hz50_a0[1][100:],hz50_a0[0][100:], linewidth=3, label=r'f = 1.55 Hz, $\alpha = 0^\circ$')
	plt.plot(hz50_a30[1][100:],hz50_a30[0][100:], linewidth=3, label=r'f = 1.55 Hz, $\alpha = 30^\circ$')
	
	plt.xticks(fontsize=15)
	plt.yticks(fontsize=15)
	plt.xlabel('y-coordinate (m)', fontsize=15)
	plt.ylabel('x-coordinate (m)', fontsize=15)
	#plt.legend(fontsize=20)
	plt.legend(loc="upper right", fontsize=15)
	plt.subplots_adjust(left=0.15, bottom=0.2,right=0.96, top=0.9)
	plt.grid()
	plt.savefig('swingF.pdf',dpi=300)
	plt.show()