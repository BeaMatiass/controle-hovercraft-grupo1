import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint, upper_limit, lower_limit):

        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint                    # Desired distance in % of image pixels
        self.upper_limit = upper_limit              # Output upper limit
        self.lower_limit = lower_limit              # Output lower limit

        # Persistent state
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.last_output = None       

    def process_pid(self, current_distance):
        current_time = time.time()

        error = self.setpoint - current_distance

        # Compute dt
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time
        
        self.prev_time = current_time

        # Derivative term
        if dt > 0.0 and self.prev_error is not None:
            derivative_action = (error - self.prev_error) / dt
        else:
            derivative_action = 0.0
    
        # Integral (tentative)
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

        # Update state and return
        self.prev_error = error
        self.last_output = saturated_output
        
        return saturated_output

pid_distance = PIDController(
    kp=               0.1, 
    ki=               0.1, 
    kd=               0.1,
    setpoint=         60.0,         # desired % of pixels
    lower_limit=      0.0,   
    upper_limit=      1.0    
)
    
pid_servo = PIDController(
    kp=               0.1, 
    ki=               0.1, 
    kd=               0.1,
    setpoint=         0,            # blades straight
    lower_limit=     -1.0,   
    upper_limit=      1.0    
)