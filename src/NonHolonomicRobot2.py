import numpy as np
from Robot import Robot
#control con estado aumentado, sistema expandido, delta star
class NonHolonomicRobot(Robot):
    def __init__(self, robot_id: int, initial_pos: np.ndarray, u_r: float, k_gamma: float, initial_alpha: float):					 
        super().__init__(robot_id, initial_pos)
        self.u_r = u_r              # Rapidez constante [cite: 64]
        self.k_gamma = k_gamma      # Ganancia de control angular [cite: 181]
        self.alpha = initial_alpha  # Orientación actual [cite: 60]

		# --- ESTADO AUMENTADO ---
        # Inicializamos delta_star como el error inicial normalizado.
        # A partir de aquí, crecerá o decrecerá sin saltos de 2pi.
        self.delta_star = 0.0 
        self.last_target_alpha = initial_alpha
        
        
    def move(self, ascent_vector: np.ndarray, dt: float) -> None:
        """
        Implementa dinámica de uniciclo (Eq. 2 y 9).
        La velocidad lineal es fija, se controla la velocidad angular omega.
        """
        # 1. Calcular el ángulo deseado del campo guía (Eq. 7)
        # Si el vector de ascenso es casi cero, mantenemos la dirección
        if np.linalg.norm(ascent_vector) < 1e-6:
            target_alpha = self.alpha
        else:
            target_alpha = np.arctan2(ascent_vector[1], ascent_vector[0])

        
		# 2. Calcular la variación del campo guía (omega_d)
        # Estimamos cuánto ha rotado el vector objetivo desde el último frame
        d_target_alpha = np.arctan2(np.sin(target_alpha - self.last_target_alpha), 
                                    np.cos(target_alpha - self.last_target_alpha))
        omega_d = d_target_alpha / dt
        
        # 3. Actualizar el error aumentado delta_star (Eq. del sistema aumentado)
        # La derivada de delta_star es (omega_i - omega_d)
        # Usamos la ley de control: omega_i = -k_gamma * delta_star
        # Por lo tanto: delta_star_dot = -k_gamma * delta_star - omega_d
        
        # Primero calculamos omega_i basado en el error acumulado actual
        omega_i = -self.k_gamma * self.delta_star
        
        # Actualizamos delta_star mediante integración de Euler
        # Nota: delta_star no se normaliza con arctan2
        self.delta_star += (omega_i - omega_d) * dt

        # 4. Actualizar orientación física del robot
        self.alpha += omega_i * dt
        
        # 5. Actualizar posición (Cinemática de Uniciclo)
        velocity = self.u_r * np.array([np.cos(self.alpha), np.sin(self.alpha)])
        self._position += velocity * dt

        # Guardar estado para el siguiente paso
        self.last_target_alpha = target_alpha
        
     
