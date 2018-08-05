import time
import numpy as np

class PIDController:
	
	def __init__(self, kP=0.0, kI=0.0, kD=0.0, max_integral=None, integral_bound=None)
		self.kP = kP
		self.kI = kI
		self.kD = kD

		self.integral = 0.0
		self.prev_time = 0.0
		self.prev_error = 0.0

		self.max_integral = max_integral
		self.integral_bound = integral_bound

	def output(self, current, setpoint):
		cur_time = time.time()

		error = setpoint - current
		delta_time = cur_time - self.prev_time

		proportional = self.kP * error
		if self.integral_bound is not None:
			if abs(error) < self.integral_bound:
				self.integral = self.kI * (self.integral + error) * delta_time
		derivative = self.kD * (error - self.prev_error) / delta_time

		if self.max_integral is not None and abs(self.integral) > self.max_integral:
			self.integral = self.max_integral * np.sign(self.integral)


		self.prev_error = error
		self.prev_time = cur_time

		output = proportional + self.integral + derivative
		return output

	def reset(self):
		self.integral = 0.0
		self.prev_error = 0.0