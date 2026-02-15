import numpy as np

class ScalarField:
    def __init__(self, map_seed: int):
        """
        Representa el campo escalar de la señal.
        Basado en una función Gaussiana 2D.
        """
        # Inicializamos el generador con la semilla específica del mapa
        self.rng = np.random.default_rng(seed=map_seed)
        
        # Parámetros del campo aleatorios pero determinados por la semilla
        # Posicionamos la fuente r* en un rango de [-20, 20]
        self.source_pos = self.rng.uniform(-20, 20, size=2)
        
        # Intensidad máxima (amplitud) y desviación estándar (sigma_std)
        self.intensity_max = 100.0
        self.sigma_std = 10.0

    def get_intensity(self, pos: np.ndarray) -> float:
        """
        Calcula sigma(r) en una posición dada.
        Implementa la función Gaussiana: 
        sigma(r) = A * exp(-||r - r*||^2 / (2 * sigma_std^2)).
        """
        dist_sq = np.sum((pos - self.source_pos)**2)
        intensity = self.intensity_max * np.exp(-dist_sq / (2 * self.sigma_std**2))
        return float(intensity)
