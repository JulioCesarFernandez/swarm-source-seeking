import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from Simulation import Simulation

# Configuración de la Simulación
SIM_TIME = 40.0
DT = 0.1
NUM_ROBOTS = 3

guardarVideo = 0
#1:Si, 0:No
nombre_archivo_video = 'simulacion_source_seeking10-expanded-high-K.mp4'

# Inicializar con el sistema de Triple Semilla
sim = Simulation(map_seed=42, centroid_seed=3, swarm_pos_seed=101, dt=DT)
sim.deploy_swarm(num_robots=NUM_ROBOTS, robot_type='non-holonomic')

# Preparación de la figura para telemetría visual
# Aumentamos ligeramente el ancho para que quepa bien el texto técnico
fig, ax = plt.subplots(figsize=(10, 7)) 
x_range = np.linspace(-30, 30, 100)
y_range = np.linspace(-30, 30, 100)
X, Y = np.meshgrid(x_range, y_range)
Z = np.array([[sim.field.get_intensity(np.array([x, y])) for x in x_range] for y in y_range])

# Renderizar el campo escalar (Mapa de calor)
contour = ax.contourf(X, Y, Z, levels=20, cmap='viridis', alpha=0.6)
plt.colorbar(contour, label=r'Intensidad de Señal')

# Elementos gráficos
robot_dots, = ax.plot([], [], 'yo', label='Robots')
centroid_dot, = ax.plot([], [], 'rx', markersize=10, label=r'Centroide')
source_dot, = ax.plot(sim.field.source_pos[0], sim.field.source_pos[1], 'r*', markersize=12, label=r'Fuente')

paths = [ax.plot([], [], lw=0.5, alpha=0.5)[0] for _ in range(NUM_ROBOTS)]
history = [[] for _ in range(NUM_ROBOTS)]

ax.set_title("Simulación Source-Seeking y Análisis de Estabilidad de Lyapunov")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend(loc='upper right')
ax.grid(True)

# Cuadro de texto para telemetría extendida
props = dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='gray')
status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=9,
                      family='monospace', verticalalignment='top', bbox=props)

def update(frame):
    sim.run_step()
    
    # Extraer parámetros dinámicos
    current_deg_index = sim.controller.get_degeneration_index()
    
    # 1. Cálculo de V_x, V_y mediante matriz de covarianza
    posiciones = np.array([robot.get_current_position() for robot in sim.robots])
    centroide = sim.controller.centroid
    pos_centradas = posiciones - centroide
    
    # Calculamos la matriz de covarianza (transponemos para que las variables sean X e Y)
    cov_matrix = np.cov(pos_centradas.T)
    
    if cov_matrix.ndim == 0 or np.all(cov_matrix == 0):
        V_x, V_y = 1e-6, 1e-6 # Evitar divisiones por cero
    else:
        # Obtenemos los autovalores de dispersión geométrica
        eig_geom, _ = np.linalg.eigh(cov_matrix)
        # Asignamos el mayor a V_y y el menor a V_x asumiendo degeneración transversal
        V_y, V_x = np.sort(eig_geom)[::-1] 
        V_x = max(V_x, 1e-6) # Prevención de singularidad estricta
    
    # 2. Extraer constantes del controlador del primer robot
    # Nota: Asegúrate de que las propiedades u_r y k_gamma sean públicas o tengan getters en NonHolonomicRobot
    try:
        u_val = sim.robots[0].u_r
        k_gamma_val = sim.robots[0].k_gamma
    except AttributeError:
        # Valores de fallback por si están encapsulados rígidamente
        print("los valores de velocidad y k_gamma no se han podido extraer de robot")
        u_val = 1.0 
        k_gamma_val = 1.0 
        
    # Calcular distancia a la fuente (x_c en coordenadas relativas)
    dist_fuente = np.linalg.norm(centroide - sim.field.source_pos)
    x_c = max(dist_fuente, 1e-3)
    
    # 3. Análisis de Estabilidad
    rho = V_y / (V_x * x_c)
    ratio_varianzas = V_y / V_x
    limite_critico = (k_gamma_val * x_c) / (4 * u_val)
    
    discriminante = (k_gamma_val**2) - (4 * u_val * k_gamma_val * rho)
    
    if discriminante >= 0:
        lambda_1 = (k_gamma_val + np.sqrt(discriminante)) / 2
        lambda_2 = (k_gamma_val - np.sqrt(discriminante)) / 2
        estado_estab = "NODO INESTABLE (Divergencia monótona)"
        str_raices = f"L_1: {lambda_1:.2f} | L_2: {lambda_2:.2f}"
    else:
        parte_real = k_gamma_val / 2
        parte_imaginaria = np.sqrt(abs(discriminante)) / 2
        estado_estab = "FOCO INESTABLE (Oscilación divergente)"
        str_raices = f"L_1,2: {parte_real:.2f} +/- j{parte_imaginaria:.2f}"
    
    # 4. Formatear la salida de telemetría
    info_text = (
        f"--- PARÁMETROS DE SIMULACIÓN ---\n"
        f"Map/Centroid/Swarm Seeds: {sim.map_seed}/{sim.centroid_seed}/{sim.swarm_pos_seed}\n"
        f"Índice Degeneración:      {current_deg_index:.4f}\n"
        f"K_gamma:      {k_gamma_val:.4f}\n\n"
        f"--- ANÁLISIS GEOMÉTRICO ---\n"
        f"Varianza Mayor (V_y):     {V_y:.4f}\n"
        f"Varianza Menor (V_x):     {V_x:.4f}\n"
        f"Ratio Geométrico (Vy/Vx): {ratio_varianzas:.4f}\n"
        f"Límite Crítico Bifurc.:   {limite_critico:.4f}\n\n"
        f"--- ESTADO DEL SISTEMA ---\n"
        f"Autovalores:              {str_raices}\n"
        f"Régimen Actual:           {estado_estab}"
    )
    
    status_text.set_text(info_text)
    
    # Actualizar posiciones de robots y trayectorias
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
