#!/usr/bin/env python

class PID:
	def __init__(self, Ki, Kp, Kd, max_out = None, min_out = None):
		self.SetGains(Ki, Kp, Kd)
		self._max_out, self._min_out = max_out, min_out

		self._error_sum, self._prv_error = 0, 0

	def SetGains(self, Ki, Kp, Kd): 
		if (not((Ki <= 0 and Kp <= 0 and Kd <= 0)
			or (Ki >= 0 and Kp >=0 and Kd >= 0))):
			print('Warning: Gains should have the same sign for stability')

		self._Ki, self._Kp, self._Kd = Ki, Kp, Kd

	def Calculate(self, dt, setpoint, plant_state):
		error = setpoint - plant_state
		self._error_sum += error*dt

		prop_error = self._Kp*error
		intg_error = self._Ki*(self._error_sum)
		derv_error = self._Kd*(error-self._prv_error)/dt

		self._prv_error = error
		control_out = prop_error + intg_error + derv_error

		if (self._max_out is not None) and control_out >= self._max_out:
			control_out = self._max_out
		elif (self._min_out is not None) and control_out <= self._min_out:
			control_out = self._min_out

		return control_out
