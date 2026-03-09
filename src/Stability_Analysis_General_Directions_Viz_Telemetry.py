import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from Simulation import Simulation

import csv
import os

# ... (Sus importaciones y configuraciones previas) ...

# --- CONFIGURACIÓN DEL SISTEMA DAQ (DATA ACQUISITION) ---
log_filename = 'telemetria_estabilidad_enjambre1.csv'

# Inicializar el archivo y escribir las cabeceras de las columnas
with open(log_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        'Tiempo_s', 'Map_Seed', 'Swarm_Seed', 'Deg_Index', 'k_gamma', 'u_r',
        'D_c', 'V_par', 'V_perp', 'Ratio_Vy_Vx', 'Limite_Critico', 
        'Discriminante', 'L1_Real', 'L1_Imag', 'L2_Real', 'L2_Imag', 'Regimen'
    ])
print(f"[DAQ] Sistema de telemetría inicializado: {log_filename}")


# Configuración de la Simulación
SIM_TIME = 40.0
DT = 0.1
NUM_ROBOTS = 3
guardarVideo = 0
nombre_archivo_video = 'Stability_General_Vectors.mp4'

# Inicializar simulación
sim = Simulation(map_seed=42, centroid_seed=3, swarm_pos_seed=109, dt=DT)
sim.deploy_swarm(num_robots=NUM_ROBOTS, robot_type='non-holonomic')

# Preparación de la figura
fig, ax = plt.subplots(figsize=(10, 7)) 
x_range = np.linspace(-30, 30, 100)
y_range = np.linspace(-30, 30, 100)
X, Y = np.meshgrid(x_range, y_range)
Z = np.array([[sim.field.get_intensity(np.array([x, y])) for x in x_range] for y in y_range])

contour = ax.contourf(X, Y, Z, levels=20, cmap='viridis', alpha=0.6)
plt.colorbar(contour, label='Intensidad de Señal')

# Elementos gráficos de posición
robot_dots, = ax.plot([], [], 'yo', label='Robots')
centroid_dot, = ax.plot([], [], 'rx', markersize=10, label='Centroide')
source_dot, = ax.plot(sim.field.source_pos[0], sim.field.source_pos[1], 'r*', markersize=12, label='Fuente')


# Inicialización de Vectores (Quivers) con datos ficticios [0] en lugar de []
# Esto reserva correctamente la memoria para matrices 1D en el motor gráfico
Q_par = ax.quiver([0], [0], [0], [0], color='blue', scale=1, scale_units='xy', angles='xy', width=0.005, label=r'Eje Longit. $\mathbf{\hat{e}}_{\parallel}$')
Q_perp = ax.quiver([0], [0], [0], [0], color='green', scale=1, scale_units='xy', angles='xy', width=0.005, label=r'Eje Transv. $\mathbf{\hat{e}}_{\perp}$')
Q_L = ax.quiver([0], [0], [0], [0], color='red', scale=1, scale_units='xy', angles='xy', width=0.008, label=r'Ascenso $\mathbf{\hat{L}}$')

paths = [ax.plot([], [], lw=0.5, alpha=0.5)[0] for _ in range(NUM_ROBOTS)]
history = [[] for _ in range(NUM_ROBOTS)]

ax.set_title("Estabilidad del sistema uniciclo y Dinámica Vectorial")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend(loc='upper right', fontsize=8)
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
    
    # 1. Vectores del Marco Intrínseco
    vector_d = centroide - fuente
    D_c = np.linalg.norm(vector_d)
    D_c = max(D_c, 1e-3)
    
    e_par = vector_d / D_c
    e_perp = np.array([-e_par[1], e_par[0]])
    
    # 2. Vector de Gradiente Estimado (L_hat)
    # Asegúrese de que este método devuelva un array o lista con [Lx, Ly]
    L_hat = np.array(sim.controller.calculate_ascent_direction())
    L_norm = np.linalg.norm(L_hat)
    if L_norm > 1e-6:
        L_dir = L_hat / L_norm # Normalizamos solo para la visualización de la dirección
    else:
        L_dir = np.array([0.0, 0.0])
        
    # ... (código previo del cálculo de L_hat) ...
    
    # Actualizar gráficos de vectores (magnitud visual escalada a 5 metros para claridad)
    vis_scale = 5.0
    
    # ATENCIÓN: set_offsets requiere una matriz Nx2, por eso usamos [centroide]
    # set_UVC requiere listas o arreglos, por eso usamos corchetes [...]
    Q_par.set_offsets([centroide])
    Q_par.set_UVC([e_par[0] * vis_scale], [e_par[1] * vis_scale])
    
    Q_perp.set_offsets([centroide])
    Q_perp.set_UVC([e_perp[0] * vis_scale], [e_perp[1] * vis_scale])
    
    Q_L.set_offsets([centroide])
    Q_L.set_UVC([L_dir[0] * (vis_scale * 1.2)], [L_dir[1] * (vis_scale * 1.2)])
    
    # ... (continúa con el cálculo de Matriz de Covarianza Espacial) ...
    
    # 3. Matriz de Covarianza Espacial
    pos_centradas = posiciones - centroide
    cov_matrix = np.cov(pos_centradas.T)
    
    if cov_matrix.ndim == 0 or np.all(cov_matrix == 0):
        V_par, V_perp = 1e-6, 1e-6
    else:
        V_par = max(e_par.T @ cov_matrix @ e_par, 1e-6)
        V_perp = e_perp.T @ cov_matrix @ e_perp
        
    # 4. Extraer constantes
    try:
        u_val = sim.robots[0].u_r
        k_gamma_val = sim.robots[0].k_gamma
    except AttributeError:
        u_val = 1.0 
        k_gamma_val = 1.0 
        print("error en getter de kgamma")
    # 5. Análisis de Estabilidad
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
    
    # 5. Registro DAQ (Data Logging)
    tiempo_actual = frame * DT
    with open(log_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            f"{tiempo_actual:.2f}", sim.map_seed, sim.swarm_pos_seed, 
            f"{current_deg_index:.6f}", f"{k_gamma_val:.4f}", f"{u_val:.4f}",
            f"{D_c:.4f}", f"{V_par:.6f}", f"{V_perp:.6f}", f"{ratio_varianzas:.4f}",
            f"{limite_critico:.4f}", f"{discriminante:.4f}", 
            f"{l1_real:.4f}", f"{l1_imag:.4f}", f"{l2_real:.4f}", f"{l2_imag:.4f}", 
            estado_estab
        ])
        
    
    # 6. Formato de Telemetría
    
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
    
    # Retornar todos los artistas modificados para blitting
    return robot_dots, centroid_dot, Q_par, Q_perp, Q_L, status_text, *paths

ani = FuncAnimation(fig, update, frames=int(SIM_TIME/DT), interval=50, blit=False) # Blit en False previene bugs visuales con Quivers

if guardarVideo == 1:
    try:
        print("Iniciando renderizado de video...")
        ani.save(nombre_archivo_video, writer='ffmpeg', fps=20, dpi=200)
        print("Video guardado exitosamente.")
    except Exception as e:
        print(f"Error al guardar video: {e}")

plt.show()
