import numpy as np
from Robot import Robot
class HolonomicRobot(Robot):
    def move(self, ascent_vector: np.ndarray, dt: float) -> None:
        """
        Implementa dinámica libre (Eq. 1).
        El robot puede moverse en cualquier dirección instantáneamente.
        """
        # r_dot = L_hat
        self._position += ascent_vector * dt

