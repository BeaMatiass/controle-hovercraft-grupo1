#!/usr/bin/env python3

import rospy

class controle_pid:
    def __init__(self, kp, ki, kd, setpoint, upper_limit, lower_limit):

        # Ganhos
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Setpoint e limites de saída
        self.setpoint = setpoint
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

        # Estados Internos
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.last_output = 0.0       

    def processar_pid(self, current_value):
        
        # Tempo atual
        agora = rospy.Time.now().to_sec()
        
        # erro = referencia - medida
        error = self.setpoint - current_value

        # dt
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = agora - self.prev_time
            if dt < 0:
                dt = 0.0
        
        # Depois de calcular dt, atualiza tempo (agora -> previous)
        self.prev_time = agora

        # Derivada (calcula só se houver prev_error válido e dt > 0)
        if dt > 0.0 and self.prev_error is not None:
            derivative_action = (error - self.prev_error) / dt
        else:
            derivative_action = 0.0
    
        # Integral
        if dt > 0.0:
            new_integral = self.integral + error * dt
        else:
            new_integral = self.integral

        # Saída não saturada (valor que o PID quer mandar sem considerar limitações físicas)
        unsaturated_output = (self.kp * error) + (self.ki * new_integral) + (self.kd * derivative_action)

        # Saída saturada (força o valor insaturado para um intervalo seguro físicamente - segundo parâmetros informados antes)
        saturated_output = max(self.lower_limit, min(unsaturated_output, self.upper_limit))

        # Condicional que só atualiza integral quando apropriado
        if saturated_output == unsaturated_output: 
            # Se não for saturado (não romper limite físico) -> aceita integral atualizado
            self.integral = new_integral
        else:
            # Se for saturado, permite atualizar a integral somente se o sinal ajudar a tirar saturação
            if (saturated_output == self.upper_limit and error < 0) or (saturated_output == self.lower_limit and error > 0):
                self.integral = new_integral
            # Caso contrário, não atualiza

        # Atualiza o histórico
        self.prev_error = error
        self.last_output = saturated_output
        
        return saturated_output
    
