import numpy as np
import matplotlib.pyplot as plt
from SwarmController import SwarmController 
from ScalarField import ScalarField
from HolonomicRobot import HolonomicRobot
from NonHolonomicRobot import NonHolonomicRobot
class Simulation:
    def __init__(self, map_seed: int, centroid_seed: int, swarm_pos_seed: int, dt: float = 0.1):
        """
        Orquestador de la simulación con control granular de aleatoriedad.
        """
        self.dt = dt
        self.map_seed = map_seed
        self.centroid_seed = centroid_seed
        self.swarm_pos_seed = swarm_pos_seed
        
        # 1. Inicializar el campo escalar (Entorno)
        self.field = ScalarField(map_seed)
        
        # 2. Swarm y Controlador
        self.robots = []
        self.controller = None

    def deploy_swarm(self, num_robots: int, robot_type='non-holonomic'):
        """
        Despliega los robots usando semillas independientes para ubicación y geometría.
        """
        # Semilla para la posición inicial del centroide
        rng_c = np.random.default_rng(seed=self.centroid_seed)
        initial_centroid = rng_c.uniform(-15, 15, size=2)
        
        # Semilla para la geometría interna del enjambre (posiciones relativas)
        rng_s = np.random.default_rng(seed=self.swarm_pos_seed)
        
        self.robots = []
        for i in range(num_robots):
            # Posición relativa r_i = r_c + x_i [cite: 52]
            relative_pos = rng_s.normal(0, 2, size=2) 
            pos_i = initial_centroid + relative_pos
            
            if robot_type == 'holonomic':
                new_robot = HolonomicRobot(i, pos_i)
            else:
                # Parámetros para uniciclo: u_r, k_gamma, alpha_inicial
                alpha_i = rng_s.uniform(-np.pi, np.pi)
                new_robot = NonHolonomicRobot(i, pos_i, u_r=2.0, k_gamma=0.5, initial_alpha=alpha_i)
            
            self.robots.append(new_robot)
        
        self.controller = SwarmController(self.robots)

    def run_step(self):
        """
        Ejecuta un paso de integración temporal.
        """
        # A. Percepción: Cada robot mide el campo [cite: 23, 50]
        for robot in self.robots:
            robot.sense(self.field)
            
        # B. Procesamiento: El controlador calcula la dirección de ascenso [cite: 72]
        ascent_vec = self.controller.calculate_ascent_direction()
        
        # C. Acción: Los robots mueven según su dinámica [cite: 57, 59]
        for robot in self.robots:
            robot.move(ascent_vec, self.dt)

    def run(self, total_time: float):
        """Ejecuta la simulación completa."""
        steps = int(total_time / self.dt)
        for _ in range(steps):
            self.run_step()
