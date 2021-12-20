#!usr/bin/env python2

import numpy as np

def wrap(n, lo, hi):
	"""
	Parameters:
		n - a number [float]
		lo - the desired minimum value [float]
		hi - the desired maximum value [float]
	Returns the number `n` if it is in the range [`min`, `max`] inclusive, else returns `min` or `max`.
	"""
	if lo > hi:
		return n
	
	return min(hi, max(lo, n))

def range(data, lo, hi):
	'''
	Parameters:
		data - a 1D numpy.array of numbers [array]
		lo - the desired minimum element value [float]
		hi - the desired maximum element value [float]
	Returns the filtered np.array and the original indices of its elements (filtered_array, filtered_indices).
	'''
	is_nearby = np.where(lo <= data <= hi)
	return data[is_nearby], is_nearby

def bp_report(data):
	'''
	Returns percentile quantities [min, Q1, median, Q3, max], commonly used to construct boxplots.
	'''
	return np.percentile(data, [0, 25, 50, 75, 100])

def bp_outliers(data):
	"""
	Returns 'filter.range' for values within 1.5 * IQR of the 25% and 75% percentiles (Q1 and Q3).
	"""
	_, q1, _, q3, _ = bp_report(data)
	iqr = q3 - q1 #Inter Quartile Range
	return range(data, q1 - 1.5*iqr, q3 + 1.5*iqr)
