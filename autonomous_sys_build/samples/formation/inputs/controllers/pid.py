import math
import time
import numpy as np


class PID:
	
	def __init__(self, p, i, d):
		self.Kp = p
		self.Kd = d
		self.Ki = i
		self.error = 0   #e(t)
		self.errorM1 = 0 #e(t-1)
		self.errorM2 = 0 #e(t-2)
		self.cmd = 0     #cmd(t)
		self.cmdM = 0    #cmd(t-1) 
	def update(self, e):
		self.errorM2 = self.errorM1
		self.errorM1 = self.error
		self.error = e
		self.cmd = self.cmdM + (self.error)*self.Kp + (self.error - self.errorM1)*self.Kd + (self.error - 2*self.errorM1 + self.errorM2)*self.Ki
		self.cmdM = 0 #self.cmd
		return self.cmd
