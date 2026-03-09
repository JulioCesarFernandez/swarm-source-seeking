import numpy as np

class ScalarQuadraticField:
    def __init__(self, map_seed: int, k: float = 1.0):
        """
        Representa un campo escalar cuadrático (paraboloide invertido).
        Ideal para análisis de estabilidad lineal.
        sigma(r) = sigma_max - 0.5 * k * ||r - r*||^2
        """
        self.rng = np.random.default_rng(seed=map_seed)
        
        # Posición de la fuente r* aleatoria en rango [-20, 20]
        self.source_pos = self.rng.uniform(-20, 20, size=2)
        
        self.intensity_max = 100.0  # sigma_0
        self.k = k  # Constante de curvatura (stiffness del gradiente)

    def get_intensity(self, pos: np.ndarray) -> float:
        """
        Calcula la intensidad en la posición r.
        Nota: A diferencia de la Gaussiana, esta función puede devolver 
        valores negativos si el robot se aleja lo suficiente.
        """
        # Cálculo de la norma euclídea al cuadrado: ||r - r*||^2
        dist_sq = np.sum((pos - self.source_pos)**2)
        
        # Aplicación de la forma cuadrática
        intensity = self.intensity_max - 0.5 * self.k * dist_sq
        
        return float(intensity)
    
    def get_gradient(self, pos: np.ndarray) -> np.ndarray:
        """
        Proporciona el gradiente teórico: nabla sigma = -k * (r - r*)
        Útil para comparar con el estimador del enjambre (L_hat).
        """
        return -self.k * (pos - self.source_pos)
