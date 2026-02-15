import numpy as np
from Robot import Robot

class NonHolonomicRobot(Robot):
    def __init__(self, robot_id: int, initial_pos: np.ndarray, 
                 u_r: float, k_gamma: float, initial_alpha: float):
        super().__init__(robot_id, initial_pos)
        self.u_r = u_r              # Rapidez constante [cite: 64]
        self.k_gamma = k_gamma      # Ganancia de control angular [cite: 181]
        self.alpha = initial_alpha  # Orientación actual [cite: 60]

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

        # 2. Calcular error de orientación delta (Eq. 8)
        # Usamos np.arctan2 para normalizar la diferencia en (-pi, pi]
        delta = np.arctan2(np.sin(target_alpha - self.alpha), 
                           np.cos(target_alpha - self.alpha))

        # 3. Ley de control angular (Eq. 181)
        omega = self.k_gamma * delta

        # 4. Actualizar estado del robot (Integración de Euler)
        self.alpha += omega * dt
        
        # r_dot = u_r * [cos(alpha), sin(alpha)]
        velocity = self.u_r * np.array([np.cos(self.alpha), np.sin(self.alpha)])
        self._position += velocity * dt
