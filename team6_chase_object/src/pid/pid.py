#!/usr/bin/env python

class PID:
	def __init__(self, Ki, Kp, Kd):
		self.SetGains(Ki, Kp, Kd)
		self._error_sum, self._prv_error = 0, 0

	def SetGains(self, Ki, Kp, Kd): 
		if (not(Ki <= 0 or Kp <= 0 or Kd <= 0)
			or (Ki >= 0 or Kp >=0 or Kd >= 0)):
			print('Warning: Gains should have the same sign for stability')

		self._Ki, self._Kp, self._Kd = Ki, Kp, Kd

	def Calculate(self, dt, setpoint, plant_state):
		error = setpoint - plant_state
		self._error_sum += error*dt

		prop_error = self._Kp*error
		intg_error = self._Ki*(self._error_sum)
		derv_error = self._Kd*(error-self._prv_error)/dt

		self._prv_error = error
		return prop_error + intg_error + derv_error
