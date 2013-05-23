import pygame, sys, math, random
from pygame.locals import *

#settings
DISP_WIDTH = 800
DISP_HEIGHT = 600
BGCOLOR = (0, 0, 132)
PLANK_NUM = 5
PLANK_LEN = [100, 100, 100, 75, 75]


class IKSolver():

	def __init__(self, planknum_):
		pygame.init()
		self.planknum = planknum_
		self.FPS = 30 # frames per second setting 
		self.fpsClock = pygame.time.Clock()
		self.screen = pygame.display.set_mode((DISP_WIDTH, DISP_HEIGHT))
		self.screen.fill(BGCOLOR)
		pygame.display.set_caption('IK solver')
		self.plankImg = pygame.image.load('plank.png')
		self.plankWidth = self.plankImg.get_width()
		self.plankHeight = self.plankImg.get_height()
		self.plankScalars = [(int(self.plankWidth * l/100.0), self.plankHeight) for l in PLANK_LEN]
		#print self.plankScalars
		self.targetImg = pygame.image.load('target.png')
		self.endImg = pygame.image.load('end.png')
		self.plankAngles = [0]*planknum_
		self.worldAngles = [0]*planknum_
		self.plankPositions = [(0, 0)]*planknum_
		self.plankEnds = [(100, 0)]*planknum_
		self.goal = (-50, 50, random.randint(0, 360))

	# convert a point, (x, y) tuple, to a PyGame coordinate
	def pointDisplay(self, p):
		x, y = p
		return (DISP_WIDTH/2 + x, DISP_HEIGHT/2 - y)
	
	# convert a point on the screen, (x, y) tuple, to a point in world space
	def pointActual(self, p):
		x, y = p
		return (x - DISP_WIDTH/2, -y + DISP_HEIGHT/2)
		
	def cross(self, a, b):
		c = [a[1]*b[2] - a[2]*b[1],
			 a[2]*b[0] - a[0]*b[2],
			 a[0]*b[1] - a[1]*b[0]]

		return c
		
	def computeJacobianTranspose(self):
		endx, endy = self.plankEnds[-1]
		jt = []
		for startx, starty in self.plankPositions:
			dx = endx - startx
			dy = endy - starty
			js = self.cross((0, 0, 1), (dx, dy, 0))
			jt.append([js[0], js[1]])
		return jt
	
	def computeTargetVector(self):
		endx, endy = self.plankEnds[-1]
		targetx, targety, trash = self.goal
		return [targetx-endx, targety-endy]
		
	def computeRotationVector(self, jacobianTranspose, targetVector):
		rv = []
		for row in jacobianTranspose:
			rv.append(row[0]*targetVector[0] + row[1]*targetVector[1])
		return rv
	
	def adjustForFramerate(self, v):
		for i in range(len(v)):
			v[i] = v[i] / (120 * float(self.FPS))
		return v

	def rotatePlanks(self, angles):
		for i in range(len(self.plankAngles)):
			self.plankAngles[i] += angles[i]
		
		self.worldAngles[0] = self.plankAngles[0]
		for a in range(1, len(self.worldAngles)):
			self.worldAngles[a] = self.worldAngles[a-1] + self.plankAngles[a]
		
		theta = math.radians(self.plankAngles[0])
		x = PLANK_LEN[0] * math.cos(theta)
		y = PLANK_LEN[0] * math.sin(theta)
		self.plankEnds[0] = (x, y)
		for j in range(1, len(self.plankPositions)):
			self.plankPositions[j] = self.plankEnds[j-1]
			x0, y0 = self.plankPositions[j]
			theta += math.radians(self.plankAngles[j])
			x = x0+PLANK_LEN[j] * math.cos(theta)
			y = y0+PLANK_LEN[j] * math.sin(theta)
			self.plankEnds[j] = (x, y)
		#print self.plankPositions
		#print self.plankEnds
		#print self.plankAngles
		
	def displayPlanks(self):
		for angle, position, scalar in zip(self.worldAngles, self.plankPositions, self.plankScalars):
			stretchedPlank = pygame.transform.scale(self.plankImg, scalar)
			rotatedPlank = pygame.transform.rotate(stretchedPlank, angle)
			rotRect = rotatedPlank.get_rect()
			rotRect.center = self.pointDisplay(position)
			self.screen.blit(rotatedPlank, rotRect)
		endRect = self.endImg.get_rect()
		endRect.center = self.pointDisplay(self.plankEnds[-1])
		self.screen.blit(self.endImg, endRect)
	
	def displayTarget(self):
		x, y, a = self.goal
		rotatedTarget = pygame.transform.rotate(self.targetImg, a)
		rotRect = rotatedTarget.get_rect()
		rotRect.center = self.pointDisplay((x, y))
		self.screen.blit(rotatedTarget, rotRect)

	def run(self):
		rotationVector = [0]*self.planknum
		while True:
			self.screen.fill(BGCOLOR)
			# move the joints by how much we decided
			self.rotatePlanks(rotationVector)
			# print the target to the screen
			self.displayTarget()
			# calculate plank position in real space and display to screen
			self.displayPlanks()
			
			# compute the Jacobian Transpose
			jt = self.computeJacobianTranspose()
			# compute the target vector
			tv = self.computeTargetVector()
			# compute how far to rotate each joint
			rotationVector = self.computeRotationVector(jt, tv)
			# adjust for framerate
			rotationVector = self.adjustForFramerate(rotationVector)
			
			for event in pygame.event.get():
				if event.type == QUIT:
					return
				elif event.type == MOUSEBUTTONDOWN:
					x, y = self.pointActual( event.pos )
					a = random.randint(0, 360)
					self.goal = (x, y, a)
			
			pygame.display.update()
			self.fpsClock.tick(self.FPS)

if __name__ == '__main__':
	solver = IKSolver(PLANK_NUM)
	solver.run()
	pygame.quit()
	sys.exit()
