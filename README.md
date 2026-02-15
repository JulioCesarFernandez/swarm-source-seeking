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

## 5. Incorporacion del sistema extendido para el error angular



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
