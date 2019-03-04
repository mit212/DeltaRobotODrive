#!/usr/bin/python
"""
Delta Robot Kinematics

Jerry Ng - jerryng@mit.edu
Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019
"""

from math import sqrt
from scipy.optimize import fsolve
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time

pi = np.pi
arctan = np.arctan
sin = np.sin
cos = np.cos

RAD2DEG = 180.0/pi
DEG2RAD = pi/180.0

# POSITIVE MOTION OF THETA MOVES ARM DOWN! This is opposite the ODrive convention!

class position(object):
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

def rotz(theta):
	return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta),  np.cos(theta), 0], [0, 0, 1]])

def roty(theta):
	return np.array([[np.cos(theta), 0, np.sin(theta)],[0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

def rotx(theta):
	return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta),  np.cos(theta)]])

class deltaSolver(object):
	def __init__(self, sb = 2*109.9852, sp = 109.9852, L = 304.8, l = 609.5144, h = 42.8475, tht0 = (0, 0, 0)):
		# 109.9852mm is 2 * 2.5" * cos(30)
		(self.currTheta1, self.currTheta2, self.currTheta3) = tht0
		self.vel1 = 0
		self.vel2 = 0
		self.vel3 = 0
		#base equilateral triangle side (sb)
		#platform equilateral triangle side (sp)
		#upper legs length (L)
		#lower legs parallelogram length (l)
		#lower legs parallelogram width (h)
		self.sb = sb #2.5 inches
		self.sp = sp
		self.L = L
		self.l = l
		self.h = h

		#planar distance from {0} to near base side (wb)
		#planar distance from {0} to a base vertex (ub)
		#planar distance from {p} to a near platform side (wp)
		#planar distance from {p} to a platform vertex (up)
		self.wb = (sqrt(3)/6) * self.sb
		self.ub = (sqrt(3)/3) * self.sb
		self.wp = (sqrt(3)/6) * self.sp
		self.up = (sqrt(3)/3) * self.sp
		
		self.a = self.wb - self.up
		self.b = self.sp/2 - (sqrt(3)/2) * self.wb
		self.c = self.wp - self.wb/2

		(xx, yy, zz)=self.FK((self.currTheta1, self.currTheta2, self.currTheta3))
		#print(xx, yy, zz)
		self.x = xx
		self.y = yy
		self.z = zz
		self.endpt = (self.x, self.y, self.z)
		(th1, th2, th3) = self.IK((self.x, self.y, self.z))
		self.thts = (th1, th2, th3)
		#print(RAD2DEG*th1, RAD2DEG*th2, RAD2DEG*th3)
		self.fig = plt.figure()

		self.plot((xx,yy,zz))
	
	def plot(self, pos = (0, 0, -500)):
		(x, y, z) = pos
		thts = self.ik(pos)

		ax = self.fig.add_subplot(111, projection='3d')
		ax.set_xlim3d(-400, 400)
		ax.set_ylim3d(-400, 400)
		ax.set_zlim3d(-1000, 400)
		plt.ion()
		plt.show()
		ax.set_xlabel('X [mm]')
		ax.set_ylabel('Y [mm]')
		ax.set_zlabel('Z [mm]')
		#Draw Origin
		ax.scatter(0,0,0, marker = '+', c = 'k')

		#Draw Base
		base1 = np.matrix([-self.sb/2,-self.wb,0]).transpose()
		base2 = np.matrix([self.sb/2,-self.wb,0]).transpose()
		base3 = np.matrix([0,self.ub,0]).transpose()
		basePts = np.hstack((base1, base2, base3, base1))
		basePts = np.array(basePts)
		ax.plot(basePts[0,:] ,basePts[1,:], basePts[2,:],c='k')
		
		#Plot Endpoint
		p = np.array([x, y, z])
		a1 = p+np.array([100,0,0])
		a2 = p+np.array([0,100,0])
		a3 = p+np.array([0,0,100])
		self.xEnd = ax.plot([p[0], a1[0]], [p[1], a1[1]], [p[2], a1[2]], c='r', marker = '<')
		self.yEnd = ax.plot([p[0], a2[0]], [p[1], a2[1]], [p[2], a2[2]], c='g', marker = '<')
		self.zEnd = ax.plot([p[0], a3[0]], [p[1], a3[1]], [p[2], a3[2]], c='b', marker = '<')

		#Plot End Platform
		p = np.array([[x, y, z]]).T
		BTp1 = p+np.array([[0, -self.up, 0]]).T
		BTp2 = p+np.array([[self.sp/2, self.up, 0]]).T
		BTp3 = p+np.array([[-self.sp/2, self.up, 0]]).T
		BTp = np.array(np.hstack((BTp1, BTp2, BTp3, BTp1)))
		self.myPts = ax.plot(BTp[0,:], BTp[1,:], BTp[2,:],c='darkviolet')

		#Plot linkages
		pt1B = np.array([[0,-self.wb,0]]).T
		pt1J = pt1B+np.array([[0, -self.L*cos(-thts[0]), self.L*sin(-thts[0])]]).T
		pt1P = BTp1
		pt2B = np.dot(rotz(2*np.pi/3), pt1B)
		pt2J = pt2B+np.dot(rotz(2*np.pi/3), np.array([[0, -self.L*cos(-thts[1]), self.L*sin(-thts[1])]]).T)
		pt2P = BTp2
		pt3B = np.dot(rotz(4*np.pi/3) , pt1B)
		pt3J = pt3B+np.dot(rotz(4*np.pi/3), np.array([[0, -self.L*cos(-thts[2]), self.L*sin(-self.thts[2])]]).T)
		pt3P = BTp3
		self.link1 = ax.plot([pt1B[0][0], pt1J[0][0], pt1P[0][0]], [pt1B[1][0], pt1J[1][0], pt1P[1][0]], [pt1B[2][0], pt1J[2][0], pt1P[2][0]], c='dimgrey')
		self.link2 = ax.plot([pt2B[0][0], pt2J[0][0], pt2P[0][0]], [pt2B[1][0], pt2J[1][0], pt2P[1][0]], [pt2B[2][0], pt2J[2][0], pt2P[2][0]], c='dimgrey')
		self.link3 = ax.plot([pt3B[0][0], pt3J[0][0], pt3P[0][0]], [pt3B[1][0], pt3J[1][0], pt3P[1][0]], [pt3B[2][0], pt3J[2][0], pt3P[2][0]], c='dimgrey')

		#Update the Figure
		self.fig.canvas.draw_idle()
		plt.pause(0.0001)

	def updatePlot(self, pos = (0, 0, -500)):
		(x, y, z) = pos
		thts = self.ik(pos)
		# Plot Endpoint
		p = np.array([x, y, z])
		a1 = p+np.array([100,0,0])
		a2 = p+np.array([0,100,0])
		a3 = p+np.array([0,0,100])
		self.updateThings(self.xEnd,[p[0], a1[0]], [p[1], a1[1]],[p[2], a1[2]])
		self.updateThings(self.yEnd,[p[0], a2[0]], [p[1], a2[1]],[p[2], a2[2]])
		self.updateThings(self.zEnd,[p[0], a3[0]], [p[1], a3[1]],[p[2], a3[2]])

		#Plot End Points
		p = np.array([[x, y, z]]).T
		BTp1 = p+np.array([[0, -self.up, 0]]).T
		BTp2 = p+np.array([[self.sp/2, self.up, 0]]).T
		BTp3 = p+np.array([[-self.sp/2, self.up, 0]]).T
		BTp = np.array(np.hstack((BTp1, BTp2, BTp3, BTp1)))
		self.updateThings(self.myPts, BTp[0,:], BTp[1,:], BTp[2,:])

		#Plot linkages
		pt1B = np.array([[0,-self.wb,0]]).T
		pt1J = pt1B+np.array([[0, -self.L*cos(-thts[0]), self.L*sin(-thts[0])]]).T
		pt1P = BTp1
		pt2B = np.dot(rotz(2*np.pi/3), pt1B)
		pt2J = pt2B+np.dot(rotz(2*np.pi/3), np.array([[0, -self.L*cos(-thts[1]), self.L*sin(-thts[1])]]).T)
		pt2P = BTp2
		pt3B = np.dot(rotz(4*np.pi/3) , pt1B)
		pt3J = pt3B+np.dot(rotz(4*np.pi/3), np.array([[0, -self.L*cos(-thts[2]), self.L*sin(-self.thts[2])]]).T)
		pt3P = BTp3
		self.updateThings(self.link1, [pt1B[0][0], pt1J[0][0], pt1P[0][0]], [pt1B[1][0], pt1J[1][0], pt1P[1][0]], [pt1B[2][0], pt1J[2][0], pt1P[2][0]])
		self.updateThings(self.link2, [pt2B[0][0], pt2J[0][0], pt2P[0][0]], [pt2B[1][0], pt2J[1][0], pt2P[1][0]], [pt2B[2][0], pt2J[2][0], pt2P[2][0]])
		self.updateThings(self.link3, [pt3B[0][0], pt3J[0][0], pt3P[0][0]], [pt3B[1][0], pt3J[1][0], pt3P[1][0]], [pt3B[2][0], pt3J[2][0], pt3P[2][0]])

		#Update the Figure
		self.fig.canvas.draw_idle()
		plt.pause(0.1)
	
	def update_lines(self, num, dataLines, lines) :
		for line, data in zip(lines, dataLines) :
			# note: there is no .set_data() for 3 dim data...
			line.set_data(data[0:2, num:num+2])
			line.set_3d_properties(data[2,num:num+2])
		return lines

	def updateThings(self, linesObj, xPts, yPts, zPts):
		linesObj[0].set_data(xPts, yPts)
		linesObj[0].set_3d_properties(zPts)
	
	def FK(self,thts):
		#	Works regardless of length unit. Angle units are in radians. 
		th1, th2, th3 = thts
		def simulEqns(inp):
			(x, y, z) = inp
			l = self.l
			L = self.L
			a = self.a
			b = self.b
			c = self.c
			eq1 = 2*z*L*sin(th1) + x*x + y*y + z*z - l*l + L*L + a*a + 2*y*a + 2*L*(y+a)*cos(th1) 
			eq2 = 2*z*L*sin(th2) + x*x + y*y + z*z - l*l + L*L + b*b + c*c + 2*x*b + 2*y*c - L*(sqrt(3)*(x+b)+y+c)*cos(th2) 
			eq3 = 2*z*L*sin(th3) + x*x + y*y + z*z - l*l + L*L + b*b + c*c - 2*x*b + 2*y*c + L*(sqrt(3)*(x-b)-y-c)*cos(th3)
			return (eq1, eq2, eq3)
		return fsolve(simulEqns,(0,0,-100))
	
	def IK(self, endPos):
		x, y, z = endPos		
		def simulEqns(inp):
			(th1, th2, th3) = inp
			l = self.l
			L = self.L
			a = self.a
			b = self.b
			c = self.c
			eq1 = 2*z*L*sin(th1) + x*x + y*y + z*z - l*l + L*L + a*a + 2*y*a + 2*L*(y+a)*cos(th1) 
			eq2 = 2*z*L*sin(th2) + x*x + y*y + z*z - l*l + L*L + b*b + c*c + 2*x*b + 2*y*c - L*(sqrt(3)*(x+b)+y+c)*cos(th2) 
			eq3 = 2*z*L*sin(th3) + x*x + y*y + z*z - l*l + L*L + b*b + c*c - 2*x*b + 2*y*c + L*(sqrt(3)*(x-b)-y-c)*cos(th3)
			return (eq1, eq2, eq3)
		return fsolve(simulEqns,(0,0,0))
	
	def ik(self,endPos):
		return self.IK(endPos)

	def fk(self,thts):
		return self.FK(thts)

	def solveTheta1(self, position):
		#Takes in an argument that is position class
		#Solves for Theta1
		E1 = 2*self.L*(position.y+self.a)
		F1 = 2*position.z*self.L
		G1 = position.x**2 + position.y**2 + position.z**2 + self.a**2 + self.L**2 + 2*position.y*self.a - self.l**2

		return self.angleSolver(E1, F1, G1, 1)

	def solveTheta2(self, position):
		E2 = -self.L*(sqrt(3) * (position.x + self.b) + position.y + self.c)
		F2 = 2*position.z*self.L
		G2 = position.x**2 + position.y**2 + position.z**2 + self.b**2 + self.c**2 + self.L**2 + 2*(position.x*self.b + position.y*self.c) - self.l**2

		return self.angleSolver(E2, F2, G2, 2)

	def solveTheta3(self, position):
		E3 = self.L * (sqrt(3) * (position.x - self.b) - position.y - self.c)
		F3 = 2*position.z*self.L
		G3 = position.x**2 + position.y**2 + position.z**2 + self.b**2 + self.c**2 + self.L**2 + 2*(-position.x*self.b + position.y*self.c) - self.l**2

		return self.angleSolver(E3, F3, G3, 3)

	def angleSolver(self, E, F, G, thetaID):
		t1 = (-F + sqrt(E**2 + F**2 - G**2))/(G - E)
		t2 = (-F - sqrt(E**2 + F**2 - G**2))/(G - E)
		thetaPossible1 = 2*arctan(t1)
		thetaPossible2 = 2*arctan(t2)

		if(thetaID == 1):
			currTheta = self.currTheta1
		
		elif(thetaID == 2):
			currTheta = self.currTheta2

		elif(thetaID == 3):
			currTheta = self.currTheta3

		#calculate the difference between the possible angles that solves the quadratic with current angle. 
		thetaDiff1 = thetaPossible1 - self.currTheta1
		thetaDiff2 = thetaPossible2 - self.currTheta2

		#return the theta that is closest to the current theta
		if(abs(thetaDiff1) < abs(thetaDiff2)):
			return thetaPossible1

		else:
			return thetaPossible2
	
	def velIKSolver(self, inputVec):
		#inputVec is a vector in meters per second. [xdot, ydot, zdot]
		a11 = self.x
		a12 = self.y+self.a + self.L*cos(self.currTheta1)
		a13 = self.z + self.L*sin(self.currTheta1)

		a21 = 2*(self.x+self.b) - sqrt(3)*self.L*cos(self.currTheta2)
		a22 = 2*(self.y + self.c) - self.L*cos(self.currTheta2)
		a23 = 2*(self.z + self.L*sin(self.currTheta2))

		a31 = 2*(self.x - self.b) + sqrt(3)*self.L*cos(self.currTheta3)
		a32 = 2*(self.y + self.c) - self.L*cos(self.currTheta3)
		a33 = 2*(self.z + self.L*sin(self.currTheta3))
		A = np.matrix([[a11, a12, a13], 
			 [a21, a22, a23], 
			 [a31, a32, a33]])
		b11 = L*((self.y + self.a)*sin(self.currTheta1) - self.z*cos(self.currTheta1))
		b22 = -L*((sqrt(3)*(self.x + self.b) + self.y + self.c)*sin(self.currTheta2) + 2*self.z*cos(self.currTheta2))
		b33 = L*((sqrt(3)*(self.x-self.b) - self.y - self.c)*sin(self.currTheta3) - 2*self.z*cos(self.currTheta3))
		B = np.matrix([[b11, 0, 0], 
			[0,b22,0], 
			[0,0, b33]])
		AB = np.dot(A,B)
		thetadot = np.dot(AB, inputVec)

		#return a vector of the angle velocities. [omega1, omega2, omega3]
		return thetadot


def testPlot():
	kin = deltaSolver()
	kin.plot()
	time.sleep(1)
	kin.updatePlot((0, 100, kin.z))
	time.sleep(1)
	kin.updatePlot((100, 100, kin.z))
	time.sleep(1)
	kin.updatePlot((100, -100, kin.z))
	time.sleep(1)
	kin.updatePlot((-100, -100, kin.z))
	time.sleep(1)
	kin.updatePlot((-100, 100, kin.z))
	time.sleep(1)
	kin.updatePlot((0, 100, kin.z))
	time.sleep(1)
	kin.updatePlot((0, 0, kin.z))
	time.sleep(1)