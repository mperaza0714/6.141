#!usr/bin/env python 2

import rospy
import laplace.Laplace as LP

class SRReport:
	def __init__(self):
		self.Mp = None # Peak overshoot
		self.ess = None # Steady state error
		
		self.trise = None # Rise time (10% to 90%)
		self.tp = None # Time of peak overshoot
		self.ts = None # Settling time (<1%)


class SRMonitor:
	def __init__(self, controller = LP(), plant = LP()):
		self.K = controller
		self.g = plant
	
	def get_CL_TF(self):
		pass
	
	def generate_SR(self):
		pass
