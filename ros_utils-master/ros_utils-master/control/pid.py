#!/usr/bin/env python2

import rospy
import numpy as np

class PID:
	"""
	A signal controller that provides proportional (P), integral (I), and/or
	differential (D) error correction to any measurement, given a reference
	target.
	"""
	class Error:
		"""
		Keeps track of timestamped errors, in terms of a presenterror, previous
		error, and total accumulation of error.
		"""
		def __init__(self):
			self.pres = (0.0, 0.0) # Most recent logged error
			self.prev = self.pres # Previously logged error

			self.total = [0.0] * 100 # Accumulated error Sum(e[i] * (t[i] - t[i-1]))

		def get_de(self):
			"""Returns the """
			return self.pres[0] - self.prev[0]

		def get_dt(self):
			return self.pres[1] - self.prev[1]

		def get_total(self):
			return sum(term for term in self.total)

		def log(self, e):
			self.prev = self.pres
			self.pres = (e, rospy.Time.now().to_sec())

			self.total.pop(0)
			self.total.append(e * self.t_inc())


	def __init__(self, p, i, d):
		self.Kp = p
		self.Ki = i
		self.Kd = d
		self.reset_error()

	def reset_error(self):
		self.error = PID.Error()

	def eval(self, e):
		self.error.log(e)

		dt = self.error.get_dt()
		de = self.error.get_de()
		e = self.error.pres[0]
		total  = self.error.get_total()

		try:
			u = self.Kp * e + self.Ki * total + self.Kd * de/dt
		except:
			u = self.Kp * e + self.Ki * total + 0.0

		rospy.loginfo("PID.eval:\tu = %f deg", u_wrapped/np.pi*180.0)
		return u

class FixedPID(PID):
	class FixedError(PID.Error):
		"""
		A class representing the current, previous, and accumulated error,
		with fixed time step `dt`.
		"""
		def __init__(self, dt):
			super().__init__()
			self.dt = dt

		def get_dt(self):
			return self.dt

		def log(self, e):
			self.prev = (self.pres[0], 0)
			self.pres = (e, self.dt)

			self.total.pop(0)
			self.total.append(e * self.t_inc())

	def __init__(self, p, i, d, dt = 1.0):
		super().__init__(p, i, d)
		self.dt = dt
		self.reset_error()

	def reset_error(self):
		self.error = PID.FixedError(self.dt)
