# Documentación de Ingeniería: Simulación de Source-Seeking con Enjambres de Robots

**Referencia Principal:** *Source-Seeking Problem with Robot Swarms* (Acuaviva, Garcia de Marina, Jimenez, 2024).

## 1. Resumen del Proyecto

Este proyecto implementa una simulación numérica en Python para validar un algoritmo de localización de fuentes escalares (Source-Seeking) utilizando un enjambre de robots. A diferencia de los métodos tradicionales basados en la estimación del gradiente, este enfoque utiliza una **dirección de ascenso aproximada** () que permite al enjambre operar con una geometría flexible y resiliente.

El sistema soporta dos tipos de dinámica vehicular:

1. 
**Dinámica Libre (Holonómica):** Control directo de velocidad.


2. 
**Dinámica de Uniciclo (No-Holonómica):** Velocidad lineal constante con control de orientación angular.



---

## 2. Fundamentos Teóricos

### 2.1. El Problema de Localización

El objetivo es encontrar el máximo global  de un campo escalar . Los robots no conocen la posición de la fuente ni el mapa del campo; solo pueden medir la intensidad  en su ubicación actual.

### 2.2. Dirección de Ascenso ()

En lugar de calcular el gradiente explícito, el controlador calcula un vector de consenso ponderado:

Donde  es el centroide y  es el radio máximo de la formación. La convergencia está garantizada siempre que la geometría del enjambre sea **no-degenerada** (los robots no deben estar alineados).

### 2.3. Control de Uniciclo y Campo Guía

Para los robots con restricciones no-holonómicas (velocidad constante ), se utiliza el vector de ascenso normalizado como un **Campo Guía** . La ley de control para la velocidad angular  minimiza el error de orientación :

Esto alinea asintóticamente al robot con la dirección de ascenso.

---

## 3. Arquitectura del Software

La implementación sigue una arquitectura modular orientada a objetos para garantizar la separación de la física (entorno), la percepción (robots) y el control (enjambre).

### 3.1. Estructura de Clases

* **`Simulation`**: Orquestador principal. Implementa un sistema de **Triple Semilla** (`map_seed`, `centroid_seed`, `swarm_pos_seed`) para garantizar la reproducibilidad determinista de los experimentos.
* **`ScalarField`**: Modela el entorno físico (función Gaussiana) generado aleatoriamente según la semilla.
* **`Robot` (Abstracta)**: Define la interfaz de sensores (`sense`) y actuadores (`move`).
* **`HolonomicRobot`**: Implementa .
* **`NonHolonomicRobot`**: Implementa la cinemática de uniciclo y la integración de Euler para la orientación .


* **`SwarmController`**: "Cerebro" del sistema. Calcula  basándose exclusivamente en telemetría local de los robots. Incluye monitoreo del **Índice de Degeneración** (SVD) para validar la salud de la formación.

---

## 4. Parámetros de Simulación y Ajuste de Ingeniería

Para la validación visual, se han ajustado los parámetros teóricos para facilitar la observación de las trayectorias transitorias.

### 4.1. Ajuste de la Ganancia k_gamma en Simulation.py

El paper sugiere teóricamente valores donde  para una convergencia rápida. Sin embargo, en esta implementación hemos realizado el siguiente ajuste crítico:

* **Valor:**  (Reducido).
* **Justificación Técnica:** Al reducir la ganancia angular, "amortiguamos" la capacidad de giro del robot. Esto provoca radios de giro más amplios y una respuesta más lenta ante cambios en el campo guía.
* 
**Efecto Visual:** Esto permite apreciar mejor las trayectorias espirales y el comportamiento de **órbita (ciclo límite)** alrededor del máximo, tal como se predice en la Figura 5 del paper. Un  muy alto haría que el comportamiento fuese casi indistinguible del holonómico.



### 4.2. Configuración General

* **Tiempo de Simulación ():** 40.0 segundos.
* **Paso de Integración ():** 0.1 segundos.
* **Número de Robots ():** 5.
* **Velocidad Lineal ():** 2.0 m/s.

---


## 5. Incorporación del Sistema Extendido para el Error Angular ($\delta_{i,*}$)

Para garantizar la estabilidad global del controlador no-holónomo, la simulación implementa el **sistema aumentado** que aborda la naturaleza topológica del espacio angular.

### 5.1. El Problema de la Discontinuidad en $S^1$
El error de orientación estándar $\delta_i \in (-\pi, \pi]$ presenta una discontinuidad en $\pm \pi$. Un controlador proporcional simple generaría saltos bruscos en la velocidad angular al cruzar este umbral.

### 5.2. Solución: Variable "Desenrollada" ($\delta_{i,*}$)
Se introduce una variable de estado aumentada $\delta_{i,*}$ que vive en la recta real $\mathbb{R}$. Esta variable rastrea el ángulo acumulado, permitiendo que la ley de control sea continua:

$$\omega_i = -k_{\gamma}\delta_{i,*} + A_i$$

Donde $A_i$ es un parámetro de pre-alimentación (feedforward). En esta simulación, la dinámica del error se rige por:

$$\dot{\delta}_{i,*} = \omega_i - \omega_d = -k_{\gamma}\delta_{i,*} + (A_i - \omega_d)$$

Bajo este esquema, incluso si $\omega_d$ (la rotación del campo) es desconocida, una ganancia $k_{\gamma}$ suficientemente alta garantiza que el error permanezca acotado y el robot siga la dirección de ascenso sin oscilaciones por discontinuidad.

### 5.3. Observaciones Empíricas sobre la Convergencia
Comprobamos que converge mejor pero se queda lejos del maximo y exige kappa grande
Durante las pruebas en simulación, se comprobó que este sistema extendido mejora notablemente la suavidad de las trayectorias, evitando el "chattering" (castañeteo) direccional. Sin embargo, **exige una ganancia $k_{\gamma}$ (kappa) considerablemente grande** para asegurar la estabilidad. Como efecto colateral de esta alta ganancia, la inercia rotacional domina el movimiento, provocando que el enjambre entre en órbita prematuramente y se estabilice en un ciclo límite estacionario que **se queda lejos del máximo real $\vec{r}^*$** del campo.
Cambios incluidos en NonHolonimicRobot2.py
---

## 6. Análisis Avanzado de Estabilidad y Telemetría

Para validar rigurosamente las condiciones del Teorema de Poincaré-Bendixson y la aparición del ciclo límite bajo degeneración geométrica, se ha implementado una suite de instrumentación analítica separada del núcleo de la simulación.

### 6.1. Marco de Referencia Intrínseco y Covarianza Espacial

El análisis de estabilidad se ha desacoplado de los ejes cartesianos globales ($X, Y$). El sistema computa en tiempo real un marco intrínseco basado en el vector relativo a la fuente:

* **Eje Longitudinal ($\mathbf{\hat{e}}_{\parallel}$):** Dirección de alineación ideal hacia/desde el gradiente.
* **Eje Transversal ($\mathbf{\hat{e}}_{\perp}$):** Dirección ortogonal donde se desarrollan las oscilaciones inestables.

Mediante la proyección de la matriz de covarianza del enjambre sobre estos ejes, el simulador extrae la **Varianza Longitudinal ($V_{\parallel}$)** y la **Varianza Transversal ($V_{\perp}$)** para cualquier ángulo de ataque, garantizando que el análisis sea invariante a la rotación.

### 6.2. Monitoreo de Bifurcaciones (Estabilidad de Lyapunov)

El motor gráfico calcula en cada iteración la matriz Jacobiana linealizada del sistema de control no-holonómico. Monitoriza la relación geométrica crítica $\rho = \frac{V_{\perp}}{V_{\parallel} D_c}$ y el discriminante del sistema $\Delta$:

$$\Delta = k_{\gamma}^2 - 4 u k_{\gamma} \rho$$

Dependiendo del signo de $\Delta$, la telemetría clasifica el régimen del enjambre en tiempo real:

* **Nodo Inestable ($\Delta \ge 0$):** Raíces reales. El enjambre intenta una corrección de fase monótona (giro brusco).
* **Foco Inestable ($\Delta < 0$):** Raíces complejas conjugadas. Inyección de oscilaciones laterales divergentes que, al saturar la ley de control, originan el avance inverso estable.

### 6.3. Visualización Vectorial

Se ha incorporado la renderización dinámica de campos vectoriales en la interfaz visual:

* Vectores del marco intrínseco anclados al centroide del enjambre.
* Vector de ascenso estimado $\hat{L}$ (Consenso), permitiendo observar visualmente el fenómeno de "filtrado espacial" espacial cuando el enjambre se aplasta transversalmente y $\hat{L}$ se vuelve ortogonal a la fuente.

### 6.4. Sistema de Adquisición de Datos (DAQ)

Para el análisis post-simulación (Root Locus, diagramas de fase), se ha integrado un módulo DAQ que registra una serie temporal en formato CSV (`telemetria_estabilidad_enjambre.csv`). Este archivo almacena con alta precisión ($f_s = 1/dt$):

* Posiciones teóricas y métricas de degeneración.
* Evolución de las varianzas ($V_{\parallel}$, $V_{\perp}$) y la distancia a la fuente ($D_c$).
* Partes reales e imaginarias de los autovalores $\lambda_{1,2}$ en cada instante de tiempo.

### 6.5. Nuevos Módulos de Ejecución

La arquitectura ahora soporta múltiples escenarios de visualización para distintos propósitos de ingeniería:

* `Stability_Analysis_Viz.py`: Análisis de estabilidad asumiendo avance en el eje X global.
* `Stability_Analysis_General_Directions_Viz.py`: Análisis robusto con marco de referencia intrínseco y renderizado vectorial.
* `Stability_Analysis_General_Directions_Viz2.py`: Análisis robusto con marco de referencia intrínseco y renderizado vectorial.
* `Stability_Analysis_General_Directions_Viz_Telemetry.py`: Escenario completo con visualización avanzada y registro DAQ activo en CSV.


## Resultados y Visualización

El script `Visualization.py` genera una animación en tiempo real que muestra:

1. **Mapa de Calor:** Representación del campo escalar .
2. **Trayectorias:** Estelas históricas de cada robot para visualizar la evolución de la formación.
3. **Telemetría en Pantalla:**
* Configuración de semillas activas.
* **Índice de Degeneración:** Monitoreo en tiempo real de la calidad geométrica del enjambre.



### Comportamiento Observado

Con , los robots exhiben una fase inicial de alineación, seguida de una convergencia hacia . Al llegar a la vecindad de la fuente, debido a la velocidad constante , los robots no se detienen, sino que entran en una órbita estable, validando las predicciones teóricas sobre sistemas no-holonómicos en campos escalares.

---

## Instrucciones de Ejecución

1. Asegurar que `Simulation.py` y `Visualization.py` estén en el mismo directorio.
2. Instalar dependencias:
```bash
pip install numpy matplotlib

```


3. (Opcional) Instalar FFmpeg para exportación de video.
4. Ejecutar la visualización:
```bash
python Visualization.py

```
