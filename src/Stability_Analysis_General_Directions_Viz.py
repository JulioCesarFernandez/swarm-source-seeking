import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from Simulation import Simulation

# Configuración de la Simulación
SIM_TIME = 40.0
DT = 0.1
NUM_ROBOTS = 3
guardarVideo = 0
nombre_archivo_video = 'Stability_General_Directions.mp4'

# Inicializar simulación
sim = Simulation(map_seed=42, centroid_seed=3, swarm_pos_seed=101, dt=DT)
sim.deploy_swarm(num_robots=NUM_ROBOTS, robot_type='non-holonomic')

# Preparación de la figura
fig, ax = plt.subplots(figsize=(10, 7)) 
x_range = np.linspace(-30, 30, 100)
y_range = np.linspace(-30, 30, 100)
X, Y = np.meshgrid(x_range, y_range)
Z = np.array([[sim.field.get_intensity(np.array([x, y])) for x in x_range] for y in y_range])

contour = ax.contourf(X, Y, Z, levels=20, cmap='viridis', alpha=0.6)
plt.colorbar(contour, label=r'Intensidad de Señal')

robot_dots, = ax.plot([], [], 'yo', label='Robots')
centroid_dot, = ax.plot([], [], 'rx', markersize=10, label=r'Centroide')
source_dot, = ax.plot(sim.field.source_pos[0], sim.field.source_pos[1], 'r*', markersize=12, label=r'Fuente')

paths = [ax.plot([], [], lw=0.5, alpha=0.5)[0] for _ in range(NUM_ROBOTS)]
history = [[] for _ in range(NUM_ROBOTS)]

ax.set_title("Estabilidad de Lyapunov en Marco de Referencia Intrínseco")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend(loc='upper right')
ax.grid(True)

props = dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='gray')
status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=9,
                      family='monospace', verticalalignment='top', bbox=props)

def update(frame):
    sim.run_step()
    
    current_deg_index = sim.controller.get_degeneration_index()
    
    # Extraer posiciones y vectores clave
    posiciones = np.array([robot.get_current_position() for robot in sim.robots])
    centroide = sim.controller.centroid
    fuente = sim.field.source_pos
    
    # 1. Vectores del Marco Intrínseco (Frenet-Serret simplificado)
    vector_d = centroide - fuente
    D_c = np.linalg.norm(vector_d)
    D_c = max(D_c, 1e-3) # Prevención de singularidad en el origen
    
    # Vectores unitarios longitudinal y transversal
    e_par = vector_d / D_c
    e_perp = np.array([-e_par[1], e_par[0]]) # Rotación de 90 grados
    
    # 2. Matriz de Covarianza Espacial
    pos_centradas = posiciones - centroide
    cov_matrix = np.cov(pos_centradas.T)
    
    if cov_matrix.ndim == 0 or np.all(cov_matrix == 0):
        V_par, V_perp = 1e-6, 1e-6
    else:
        # Proyección de la covarianza sobre los ejes intrínsecos usando formas cuadráticas
        V_par = e_par.T @ cov_matrix @ e_par
        V_perp = e_perp.T @ cov_matrix @ e_perp
        
        # Salvaguarda contra colapso dimensional
        V_par = max(V_par, 1e-6)
    
    # 3. Extraer constantes del sistema
    try:
        u_val = sim.robots[0].u_r
        k_gamma_val = sim.robots[0].k_gamma
    except AttributeError:
        u_val = 1.0 
        k_gamma_val = 1.0 
        
    # 4. Análisis de Estabilidad Generalizado
    rho = V_perp / (V_par * D_c)
    ratio_varianzas = V_perp / V_par
    limite_critico = (k_gamma_val * D_c) / (4 * u_val)
    
    discriminante = (k_gamma_val**2) - (4 * u_val * k_gamma_val * rho)
    
    if discriminante >= 0:
        lambda_1 = (k_gamma_val + np.sqrt(discriminante)) / 2
        lambda_2 = (k_gamma_val - np.sqrt(discriminante)) / 2
        estado_estab = "NODO INESTABLE"
        str_raices = f"L_1: {lambda_1:.2f} | L_2: {lambda_2:.2f}"
    else:
        parte_real = k_gamma_val / 2
        parte_imaginaria = np.sqrt(abs(discriminante)) / 2
        estado_estab = "FOCO INESTABLE"
        str_raices = f"L_1,2: {parte_real:.2f} +/- j{parte_imaginaria:.2f}"
    
    # 5. Formato de Telemetría
    info_text = (
		f"--- PARÁMETROS DE SIMULACIÓN ---\n"
        f"Map/Centroid/Swarm Seeds: {sim.map_seed}/{sim.centroid_seed}/{sim.swarm_pos_seed}\n"
        f"Índice Degeneración:      {current_deg_index:.4f}\n"
        f"K_gamma:      {k_gamma_val:.4f}\n\n"
        f"--- MARCO INTRÍNSECO ---\n"
        f"Dist. a Fuente (Dc): {D_c:.2f} m\n"
        f"Var. Longit. (V_||): {V_par:.4f}\n"
        f"Var. Transv. (V_+):  {V_perp:.4f}\n"
        f"Ratio Geométrico:    {ratio_varianzas:.4f}\n"
        f"Límite Crítico:      {limite_critico:.4f}\n\n"
        f"--- ESTADO DEL SISTEMA ---\n"
        f"Discriminante (D):   {discriminante:.4f}\n"
        f"Autovalores:         {str_raices}\n"
        f"Régimen:             {estado_estab}"
    )
    
    status_text.set_text(info_text)
    
    # Actualizar gráficos
    all_x, all_y = [], []
    for i, robot in enumerate(sim.robots):
        pos = robot.get_current_position()
        all_x.append(pos[0])
        all_y.append(pos[1])
        history[i].append(pos)
        
        h = np.array(history[i])
        paths[i].set_data(h[:, 0], h[:, 1])
        
    robot_dots.set_data(all_x, all_y)
    centroid_dot.set_data([centroide[0]], [centroide[1]])
    
    return robot_dots, centroid_dot, *paths, status_text

ani = FuncAnimation(fig, update, frames=int(SIM_TIME/DT), interval=50, blit=True)

if guardarVideo == 1:
    try:
        print("Iniciando renderizado de video...")
        ani.save(nombre_archivo_video, writer='ffmpeg', fps=20, dpi=200)
        print("Video guardado exitosamente.")
    except Exception as e:
        print(f"Error al guardar video: {e}")

plt.show()
