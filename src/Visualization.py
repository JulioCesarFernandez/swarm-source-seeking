import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from Simulation import Simulation

# Configuración de la Simulación
SIM_TIME = 40.0
DT = 0.1
NUM_ROBOTS = 3

# Inicializar con el sistema de Triple Semilla
sim = Simulation(map_seed=42, centroid_seed=3, swarm_pos_seed=101, dt=DT)
guardarVideo = 0
#1:Si, 0:No
nombre_archivo_video = 'simulacion_source_seeking7-BuenaConvergencia.mp4'

sim.deploy_swarm(num_robots=NUM_ROBOTS, robot_type='non-holonomic')

# Preparación de la figura para telemetría visual
fig, ax = plt.subplots(figsize=(8, 7))
x_range = np.linspace(-30, 30, 100)
y_range = np.linspace(-30, 30, 100)
X, Y = np.meshgrid(x_range, y_range)
Z = np.array([[sim.field.get_intensity(np.array([x, y])) for x in x_range] for y in y_range])

# Renderizar el campo escalar (Mapa de calor)
contour = ax.contourf(X, Y, Z, levels=20, cmap='viridis', alpha=0.6)
plt.colorbar(contour, label=r'Intensidad de Señal $\sigma$')
# Elementos gráficos
robot_dots, = ax.plot([], [], 'yo', label='Robots')
#centroid_dot, = ax.plot([], [], 'rx', markersize=10, label='Centroide $\vec{r}_c$')
#source_dot, = ax.plot(sim.field.source_pos[0], sim.field.source_pos[1], 'r*', markersize=12, label='Fuente $\vec{r}^*$')

# Busca estas líneas y añade la 'r' al principio del string:
centroid_dot, = ax.plot([], [], 'rx', markersize=10, label=r'Centroide $\vec{r}_c$')
source_dot, = ax.plot(sim.field.source_pos[0], sim.field.source_pos[1], 'r*', markersize=12, label=r'Fuente $\vec{r}^*$')

paths = [ax.plot([], [], lw=0.5, alpha=0.5)[0] for _ in range(NUM_ROBOTS)]
history = [[] for _ in range(NUM_ROBOTS)]

ax.set_title("Simulación Source-Seeking: Robot Swarm")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend()
ax.grid(True)


# 1. Preparar el string con la información de las semillas
# info_text = (
    # f"Configuración de Semillas:\n"
    # f"Map Seed: {sim.map_seed}\n"     # Define el terreno/fuente
    # f"Centroid Seed: {sim.centroid_seed}\n" # Define inicio del enjambre
    # f"Swarm Seed: {sim.swarm_pos_seed}"     # Define geometría inicial
# )

# 2. Añadir el cuadro de texto al gráfico
# transform=ax.transAxes: Usa coordenadas relativas (0,0 es esq. inf. izq, 1,1 es esq. sup. der)
# (0.02, 0.98): Lo coloca en la esquina superior izquierda con un pequeño margen
props = dict(boxstyle='round', facecolor='white', alpha=0.8)
status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=10,
                      verticalalignment='top', bbox=props)
                      
#ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10,
#        verticalalignment='top', bbox=props)


def update(frame):
    sim.run_step()
    
    # 1. Coger el valor de la degeneracion
    current_deg_index = sim.controller.get_degeneration_index()
    
    # 2. Reconstruir el string combinando las semillas estáticas y el índice dinámico
    info_text = (
        f"Configuración de Semillas:\n"
        f"Map Seed: {sim.map_seed}\n"
        f"Centroid Seed: {sim.centroid_seed}\n"
        f"Swarm Seed: {sim.swarm_pos_seed}\n"
        f"--------------------------\n"
        f"Degeneration Index: {current_deg_index:.5f}" # Monitoreo en tiempo real
    )
    
    status_text.set_text(info_text)
    
    # Actualizar posiciones de robots y trayectorias
    all_x, all_y = [], []
    for i, robot in enumerate(sim.robots):
        pos = robot.get_current_position()
        all_x.append(pos[0])
        all_y.append(pos[1])
        history[i].append(pos)
        
        # Dibujar estela de trayectoria
        h = np.array(history[i])
        paths[i].set_data(h[:, 0], h[:, 1])
        
    robot_dots.set_data(all_x, all_y)
    centroid_dot.set_data([sim.controller.centroid[0]], [sim.controller.centroid[1]])
    
    return robot_dots, centroid_dot, *paths, status_text


# ... (Todo tu código anterior de inicialización y función update) ...

# Crear la animación (asegúrate de guardar la referencia en la variable 'ani')
ani = FuncAnimation(fig, update, frames=int(SIM_TIME/DT), interval=50, blit=True)

# --- OPCIÓN A: Guardar como Video MP4 (Profesional) ---
# Requiere FFmpeg instalado. Ideal para análisis cuadro a cuadro.
if guardarVideo == 1:
	try:
		print("Iniciando renderizado de video...")
		ani.save(nombre_archivo_video, writer='ffmpeg', fps=20, dpi=200)
		print("Video guardado exitosamente como 'simulacion_source_seeking.mp4'")
	except Exception as e:
		print(f"Error al guardar video (¿Tienes FFmpeg instalado?): {e}")

# --- OPCIÓN B: Guardar como GIF (Portátil) ---
# No requiere FFmpeg (usa Pillow), pero genera archivos más pesados.
# ani.save('simulacion.gif', writer='pillow', fps=15)

# Mostrar la ventana (Opcional si solo quieres guardar)
plt.show()
