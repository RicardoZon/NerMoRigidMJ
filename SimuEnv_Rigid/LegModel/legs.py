import math

class LegModel(object):
	"""docstring for ForeLegM"""
	def __init__(self, leg_params):
		super(LegModel, self).__init__()
		# 0: AE 	1: AB
		# 2: BC 	3: CD
		# 4: DE 	5: EF (DE+EF ~= DF)
		# A (0,0)
		self.len = leg_params
		self.By = 0
		self.Bz = self.len[1]

	def pos_2_angle(self, Fy, Fz):
		AF = math.sqrt(Fy*Fy + Fz*Fz)
		a_FAy = math.acos(Fy/AF)
		if Fz > 0:
			a_FAy = - a_FAy
		a_FAE = self.LawOfCosines_angle(AF, self.len[0], self.len[5])
		## Need
		q1 = a_FAE - a_FAy
		#--------------------------------------
		Ey =  self.len[0]*math.cos(q1)
		Ez =  self.len[0]*math.sin(q1)

		DE_FE = self.len[4]/self.len[5]
		Dy = Ey + DE_FE*(Ey-Fy)
		Dz = Ez + DE_FE*(Ez-Fz)

		AD = math.sqrt(Dy*Dy + Dz*Dz)
		BD = math.sqrt((self.By-Dy)*(self.By-Dy) + (self.Bz-Dz)*(self.Bz-Dz))

		a_ABD = self.LawOfCosines_angle(self.len[1], BD, AD)
		a_DBC = self.LawOfCosines_angle(self.len[2], BD, self.len[3])
		a_ABC = a_ABD + a_DBC

		q2 = a_ABC - math.pi
		return [q1, q2]

	def angel_2_pos(self, q1, q2):
		Ey = self.len[0]*math.cos(q1)
		Ez = self.len[0]*math.sin(q1)

		Cy = 0 - self.len[2]*math.sin(q2)
		Cz = self.len[1] + self.len[2]*math.cos(q2)


		CE = math.sqrt((Ey-Cy)*(Ey-Cy) + (Ez-Cz)*(Ez-Cz))
		a_ECz = 0
		if Ey != Cy:
			a_ECz = math.acos((Cz-Ez)/CE) * (Ey-Cy)/abs(Ey-Cy)
		a_ECD = self.LawOfCosines_angle(CE, self.len[3], self.len[4])
		if a_ECD == -10:
			return []
		a_DCz = a_ECD + a_ECz

		'''
		Dy = Cy + self.len[3]*math.cos(a_DCz-math.pi/2)
		Dz = Cz + self.len[3]*math.sin(a_DCz-math.pi/2)
		print(math.sqrt((Cy-Dy)*(Cy-Dy)+(Cz-Dz)*(Cz-Dz)),
			' -- ', math.sqrt((Ey-Dy)*(Ey-Dy)+(Ez-Dz)*(Ez-Dz)))
		'''
		Dy = Cy + self.len[3]*math.sin(a_DCz)
		Dz = Cz - self.len[3]*math.cos(a_DCz)
		

		DE_y = Dy - Ey
		DE_z = Dz - Ez

		FE_DE = self.len[5]/self.len[4]
		Fy = Ey - FE_DE*DE_y
		Fz = Ez - FE_DE*DE_z

		AF = math.sqrt((0-Fy)*(0-Fy) + (0-Fz)*(0-Fz))
		BD = math.sqrt((self.By-Dy)*(self.By-Dy) + (self.Bz-Dz)*(self.Bz-Dz))

		a_AEF = self.LawOfCosines_angle(self.len[0], self.len[5] ,AF)
		a_BCD = self.LawOfCosines_angle(self.len[2], self.len[3] ,BD)

		PI = math.pi
		#if a_AEF < PI/6 or a_BCD < PI/6  or a_AEF > PI*5/6  or a_BCD > PI*5/6:
		if a_AEF < PI/18 or a_BCD < PI/72  or a_AEF > PI*17/18  or a_BCD > PI*71/72:
			return []

		return [[0,0], [self.By, self.Bz],[Cy, Cz],
			[Dy, Dz], [Ey, Ez], [Fy, Fz]]

	def LawOfCosines_edge(self, la, lb, angle_ab):
		lc_2 = la*la + lb*lb - 2*la*lb*math.cos(angle_ab)
		lc = math.sqrt(lc_2)
		return lc

	def LawOfCosines_angle(self, la, lb, lc):
		angle_ab_cos = (la*la + lb*lb - lc*lc)/(2*la*lb)
		#print("----> ", angle_ab_cos)
		if abs(angle_ab_cos) > 1:
			return -10
		angle_ab = math.acos(angle_ab_cos)
		return angle_ab
	