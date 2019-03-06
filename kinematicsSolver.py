#!/usr/bin/python
"""
Delta Robot Kinematics and Plotting

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
	def __init__(self, sb = 2*109.9852, sp = 109.9852, L = 304.8, l = 609.5144, h = 42.8475, tht0 = (0, 0, 0), swivel_limit = 20):
		# 109.9852mm is 2 * 2.5" * cos(30)

		#swivel_limit is the physical constraint of the system's current design. 
		#different ball joints have different swivel limits
		self.swivel_limit = 0.349066 #20 degrees in radians
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
		self.x = xx
		self.y = yy
		self.z = zz
		self.endpt = (self.x, self.y, self.z)
		(th1, th2, th3) = self.IK((self.x, self.y, self.z))
		self.thts = (th1, th2, th3)
		self.fig = plt.figure()

		self.plot((xx,yy,zz))
	
	def plot(self, pos = (0, 0, -500)):
		(x, y, z) = pos
		thts = self.ik(pos)

		ax = self.fig.add_subplot(111, projection='3d')
		self.ax = ax
		ax.set_xlim3d(-400, 400)
		ax.set_ylim3d(-400, 400)
		ax.set_zlim3d(-900, 100)
		plt.gca().set_aspect('equal', adjustable='box')
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
		
		#Plot Origin Axes
		p = np.array([0, 0, 0])
		a1 = p+np.array([100,0,0])
		a2 = p+np.array([0,100,0])
		a3 = p+np.array([0,0,100])
		ax.plot([p[0], a1[0]], [p[1], a1[1]], [p[2], a1[2]], c='r', marker = '<')
		ax.plot([p[0], a2[0]], [p[1], a2[1]], [p[2], a2[2]], c='g', marker = '<')
		ax.plot([p[0], a3[0]], [p[1], a3[1]], [p[2], a3[2]], c='b', marker = '<')

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

	def updatePlot(self, pos = (0, 0, -500), color = None):
		if(color == None):
			(x, y, z) = pos
			thts = self.ik(pos)

			#Plot End Platform
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
			pt3J = pt3B+np.dot(rotz(4*np.pi/3), np.array([[0, -self.L*cos(-thts[2]), self.L*sin(-thts[2])]]).T)
			pt3P = BTp3
			self.updateThings(self.link1, [pt1B[0][0], pt1J[0][0], pt1P[0][0]], [pt1B[1][0], pt1J[1][0], pt1P[1][0]], [pt1B[2][0], pt1J[2][0], pt1P[2][0]])
			self.updateThings(self.link2, [pt2B[0][0], pt2J[0][0], pt2P[0][0]], [pt2B[1][0], pt2J[1][0], pt2P[1][0]], [pt2B[2][0], pt2J[2][0], pt2P[2][0]])
			self.updateThings(self.link3, [pt3B[0][0], pt3J[0][0], pt3P[0][0]], [pt3B[1][0], pt3J[1][0], pt3P[1][0]], [pt3B[2][0], pt3J[2][0], pt3P[2][0]])

			#Update the Figure
			self.fig.canvas.draw_idle()
			plt.pause(0.1)
		else:
			self.ax.scatter(xs = pos[0], ys = pos[1],zs = pos[2], c = color)
	
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

	def check_constraints(self, motorID, endpos, theta1):
		#endpos is a tuple

		#assign lengths
		length_to_motor = self.wb
		length_to_attach = self.up

		#create an array of the end position
		xe = np.array([endpos[0], endpos[1], endpos[2]])

		#all rotation matrices along the z axis
		if(motorID == 1):
			#rotation matrix is the identity
			rotation_matrix = np.array([[1, 0, 0],[0, 1, 0 ],[0, 0, 1]])
		if(motorID == 2):
			#rotation_matrix is rotation 120 degrees
			rotation_matrix = np.array([[np.cos(2*pi/3), -np.sin(2*pi/3), 0],[np.sin(2*pi/3), np.cos(2*pi/3), 0 ],[0, 0, 1]])
		if(motorID == 3):
			#rotation matrix is rotation -120 degrees
			rotation_matrix = np.array([[np.cos(-2*pi/3), -np.sin(-2*pi/3), 0],[np.sin(-2*pi/3), np.cos(-2*pi/3), 0 ],[0, 0, 1]])

		#rotate the end effector position so that the math is easy
		xe = np.dot(rotation_matrix,xe)

		#Find the end position of the attachment
		bvec = xe - np.array([0, length_to_attach, 0])
		x0 = np.array([length_to_motor, 0, 0])
		#Solve two equations to solve for theta3
		theta2 = np.arcsin((bvec[2] - x0[2] + self.L*np.cos(theta1))/(-self.l))
		theta3 = np.arcsin((bvec[0] - x0[0])/(self.l*np.cos(theta2)))

		#theta3 corresponds to the angle of the ball joint
		if(theta3 > self.swivel_limit):
			return False

		return True



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

if __name__ == "__main__":
	testPlot()
