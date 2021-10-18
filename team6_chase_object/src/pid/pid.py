#!/usr/bin/env python

class PID:
	def __init__(self, Kp, Ki, Kd):
		self.SetGains(Kp, Ki, Kd)

		self._error_sum, self._prv_error = 0, 0

	def SetGains(self, Kp, Ki, Kd): 
		if (not((Kp <= 0 and Ki <= 0 and Kd <= 0)
			or (Kp >= 0 and Ki >=0 and Kd >= 0))):
			print('Warning: Gains should have the same sign for stability')

		self._Kp, self._Ki, self._Kd = Kp, Ki, Kd

	def Calculate(self, dt, setpoint, plant_state):
		error = setpoint - plant_state
		self._error_sum += error*dt

		prop_error = self._Kp*error
		intg_error = self._Ki*(self._error_sum)
		derv_error = self._Kd*(error-self._prv_error)/dt

		self._prv_error = error
		control_out = prop_error + intg_error + derv_error

		return control_out

	@staticmethod
	def Saturate(output, max_out, min_out):
		if output > max_out:
			output = max_out
		elif output < min_out:
			output = min_out

		return output

