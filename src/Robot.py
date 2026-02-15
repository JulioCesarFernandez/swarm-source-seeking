import numpy as np
from abc import ABC, abstractmethod

class Robot(ABC):
    def __init__(self, robot_id: int, initial_pos: np.ndarray):
        """
        Clase base abstracta para los agentes del enjambre.
        """
        self._id = robot_id
        self._position = initial_pos.astype(float) # r_i en el paper [cite: 49]
        self._measurement = 0.0                    # sigma(r_i) 

    def sense(self, field: 'ScalarField') -> None:
        """
        Realiza la medición puntual de la intensidad del campo en la posición actual.
        Corresponde a la toma de datos sigma(r_i) mencionada en el paper[cite: 23, 50].
        """
        self._measurement = field.get_intensity(self._position)

    def get_current_measurement(self) -> float:
        """Accessor para la última medida recolectada."""
        return self._measurement

    def get_current_position(self) -> np.ndarray:
        """Accessor para la posición actual del robot r_i[cite: 49]."""
        return self._position.copy()

    @abstractmethod
    def move(self, ascent_vector: np.ndarray, dt: float) -> None:
        """
        Método abstracto que define cómo el robot interpreta la dirección 
        de ascenso según su dinámica particular[cite: 57, 59].
        """
        pass
