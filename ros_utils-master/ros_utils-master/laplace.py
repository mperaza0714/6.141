#!usr/bin/env python2

"""
Defines a notation for Nth order functions by defining them in LaPlace space. Its raw value is a dictionary {n : float}
where n is an integer power of LaPlace variable s and float is a constant multiplier.

For example, a raw value of {1: 0.2, 0: 0.8, -1: -0.05} would correspond to the Laplace expression (0.2s + 0.8 - 0.05/s). In the
time domain, this would produce a sum of the derivative of the input, the input, and the integral of the input.
"""

class LaPlace:
	REL_TOL = 1e-6 # Relative tolerance
	ABS_TOL = 1e-6 # Absolute tolerance
	
	# METHODS
	
	def __init__(self, raw_value={}):
		self.value = raw_value
	
	def eval(self, s):
		return sum(k * s**n for (n, k) in self.value)
	
	# STATIC METHODS
	
	@staticmethod
	def __isclose(a, b):
		return abs(a-b) <= max(LaPlace.REL_TOL * max(abs(a), abs(b)), LaPlace.ABS_TOL)
	
	@staticmethod
	def __cleanup(lp):
		lp.value = {n: k for (n,k) in self.value.keys() if not LaPlace.__isclose(k, 0.0) }
		return lp
	
	# OPERATOR OVERLOAD
	
	def __str__(self):
		return " + ".join("{:.4}s^{:d}".format(k, n) for n, k in self.value.iteritems())
	
	def __pos__(self):
		return LaPlace(self.value)
	
	def __neg__(self):
		return LaPlace({n: -k for (n, k) in self.value.iteritems()})
	
	def __add__(self, other):
		ans = dict(self.value)
		for n, k in other.value.iteritems():
			ans[n] = ans.get(n, 0.0) + k
		return LaPlace(ans)

	def __sub__(self, other):
		return self + -other

	def __mul__(self, other):
		ans = dict()
		for n1, k1 in self.value.iteritems():
			for n2, k2 in other.value.iteritems():
				ans[n1 + n2] = ans.get(n1+n2, 0.0) + k1*k2
		return LaPlace(ans)

# 	def __truediv__(self, other):
# 		pass

# 	def __pow__(self, exp):
# 		pass

	def __eq__(self, other):
		all_n = set(list(self.value.keys()) + list(other.value.keys()))
		return all( LaPlace.__isclose(self.value.get(n, 0.0), self.other.get(n, 0.0)) for n in all_n )
		
	def __ne__(self, other):
		return not self == other
	

if __name__ == "__main__":
	pass
