#!/usr/bin/env python

class PID:
	def __init__(self, Kp, Ki, Kd, max_out, min_out):
		self.SetGains(Kp, Ki, Kd)
		self._max_out, self._min_out = max_out, min_out

		self._prop, self._intg, self._derv = 0.0, 0.0, 0.0
		self._prv_error, self._prv_plant_state = 0.0, 0.0

	def SetGains(self, Kp, Ki, Kd):
		if (not((Kp <= 0 and Ki <= 0 and Kd <= 0)
			or (Kp >= 0 and Ki >=0 and Kd >= 0))):
			print('Warning: Gains should have the same sign for stability')

		self._Kp, self._Ki, self._Kd = Kp, Ki, Kd

	def Calculate(self, dt, setpoint, plant_state):
		error = setpoint - plant_state

		self._prop = self._Kp*error
		
		self._intg += self._Ki*error*dt
		self._intg = PID.Saturate(self._intg, self._max_out, self._min_out) # avoid windup

		# dervative on measurement (prevent dervative kick)
		self._derv = self._Kd*(plant_state-self._prv_plant_state)/dt

		self._prv_error, self._prv_plant_state = error, plant_state
		control_out = self._prop + self._intg - self._derv

		return PID.Saturate(control_out, self._max_out, self._min_out)

	@staticmethod
	def Saturate(output, max_out, min_out):
		if output > max_out:
			output = max_out
		elif output < min_out:
			output = min_out

		return output

