import time

class controle_pid:
    def __init__(self, kp, ki, kd):
        # Parâmetros PID
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Antigo dicionário
        self.integral = 0.0
        self.erro_antigo = 0.0
        self.tempo_antigo = 0.0

    def processar_pid(self, erro):
        tempo_atual = time.time()

        # Cálculo de dt
        if self.tempo_antigo == 0.0:
            dt = 0.0
        else:
            dt = tempo_atual - self.tempo_antigo
        
        self.tempo_antigo = tempo_atual
    
        # Integral
        self.integral += erro * dt

        # Derivativo
        if dt > 0:
            acao_derivativa = (erro - self.erro_antigo) / dt
        else:
            acao_derivativa = 0.0
        
        # Ação de Controle (resultado PID)
        acao_de_controle = self.kp * erro + self.ki * self.integral + self.kd * acao_derivativa

        # Atualiza erro
        self.erro_antigo = erro

        return acao_de_controle

