import numpy as np

class SwarmController:
    def __init__(self, robots_list):
        """
        Gestiona el enjambre y calcula la dirección de ascenso.
        """
        self.robots = robots_list
        self.centroid = np.zeros(2)
        self.D = 1.0  # Radio de formación (evitar división por cero inicial)
        
        # Variables de Telemetría
        self.degeneration_index = 0.0
        self.mean_alignment_error = 0.0
        self.field_variance = 0.0

    def _compute_centroid(self) -> None:
        """Calcula el centro geométrico del enjambre (Eq. pág 2)."""
        positions = [r.get_current_position() for r in self.robots]
        self.centroid = np.mean(positions, axis=0)

    def _update_D(self) -> None:
        """Calcula D como la distancia máxima al centroide (Eq. 74)."""
        distances = [np.linalg.norm(r.get_current_position() - self.centroid) for r in self.robots]
        self.D = max(distances) if distances else 1.0

    def calculate_ascent_direction(self) -> np.ndarray:
        """
        Implementa la aproximación de la dirección de ascenso L_hat (Eq. 72).
        L_hat = (1 / (N * D^2)) * sum( sigma(r_i) * (r_i - r_c) )
        """
        self._compute_centroid()
        self._update_D()
        
        N = len(self.robots)
        ascent_vector = np.zeros(2)
        
        for robot in self.robots:
            sigma_i = robot.get_current_measurement()
            relative_pos = robot.get_current_position() - self.centroid
            ascent_vector += sigma_i * relative_pos
            
        # Normalización según la Ecuación (4) del paper
        L_hat = (1 / (N * (self.D**2))) * ascent_vector
        
        # Actualizar telemetría de varianza para el análisis
        measurements = [r.get_current_measurement() for r in self.robots]
        self.field_variance = np.var(measurements)
        
        return L_hat

    def check_non_degeneracy(self) -> bool:
        """
        Verifica que los robots no estén alineados (Eq. 55).
        Calcula si los vectores relativos expanden el espacio R^2.
        """
        if len(self.robots) < 3: return False
        x_vectors = [r.get_current_position() - self.centroid for r in self.robots]
        # Usamos el valor singular más pequeño para el índice de degeneración
        _, s, _ = np.linalg.svd(x_vectors)
        self.degeneration_index = s[-1]
        return self.degeneration_index > 1e-3
        
    def get_degeneration_index(self) -> float:
        """
        Retorna el valor de degeneración calculado.
        Un valor cercano a cero indica que el enjambre está alineado.
        """
        if len(self.robots) < 3: return 0
        x_vectors = [r.get_current_position() - self.centroid for r in self.robots]
        # Usamos el valor singular más pequeño para el índice de degeneración
        _, s, _ = np.linalg.svd(x_vectors)
        self.degeneration_index = s[-1]
        return self.degeneration_index
