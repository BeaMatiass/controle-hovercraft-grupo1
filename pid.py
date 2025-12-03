import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint, upper_limit, lower_limit):

        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.last_output = None       

    def compute_pid(self, current_distance):
        current_time = time.time()

        error = self.setpoint - current_distance

        # dt
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time
        
        self.prev_time = current_time

        # Derivative
        if dt > 0.0 and self.prev_error is not None:
            derivative_action = (error - self.prev_error) / dt
        else:
            derivative_action = 0.0
    
        # Integral
        if dt > 0.0:
            new_integral = self.integral + error * dt
        else:
            new_integral = self.integral

        unsaturated_output = (self.kp * error) + (self.ki * new_integral) + (self.kd * derivative_action)

        saturated_output = max(self.lower_limit, min(unsaturated_output, self.upper_limit))

        if saturated_output == unsaturated_output:
            self.integral = new_integral
        elif (saturated_output == self.upper_limit and error < 0) or (saturated_output == self.lower_limit and error > 0):
            self.integral = new_integral

        self.prev_error = error
        self.last_output = saturated_output
        
        return saturated_output