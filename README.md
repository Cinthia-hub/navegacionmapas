# Descripción del Proyecto

El objetivo de este proyecto es implementar un sistema completo de navegación autónoma para un robot móvil Pioneer P3-DX dentro del simulador Webots.

A partir de un mapa de ocupación previamente generado, el robot debe ser capaz de planificar una ruta libre de colisiones desde su posición actual hasta una posición objetivo especificada por el usuario. Para ello se utiliza el algoritmo A* como planificador global de trayectorias.

Una vez obtenida la ruta, el robot emplea un controlador de seguimiento basado en Pure Pursuit y un controlador PID para desplazarse de manera suave y estable hasta el destino. Además, se incorporan mecanismos reactivos de evasión utilizando los sensores ultrasónicos del robot para aumentar la seguridad durante la navegación.

---

# Características del Entorno de Simulación

El escenario desarrollado en Webots cumple con las siguientes especificaciones:

* Área de trabajo de 15 × 15 m².
* Robot Pioneer P3-DX.
* Cinco o más obstáculos distribuidos dentro del entorno.
* Obstáculos de diámetro aproximado de 0.5 m y altura de 1.5 m.
* Muros perimetrales de 1.5 m de altura.
* Sistema de posicionamiento mediante GPS e IMU.
* Sensores ultrasónicos para detección de obstáculos cercanos.

---

# Objetivos

## Objetivo General

Desarrollar un sistema de navegación autónoma capaz de planificar y seguir rutas libres de colisiones dentro de un entorno conocido.

## Objetivos Específicos

1. Utilizar un mapa de ocupación previamente construido.
2. Implementar el algoritmo A* para planificación de rutas.
3. Inflar obstáculos para considerar las dimensiones del robot.
4. Generar una trayectoria segura entre origen y destino.
5. Implementar seguimiento de trayectoria mediante Pure Pursuit.
6. Utilizar control PID para corregir errores de orientación.
7. Incorporar evasión reactiva mediante sensores ultrasónicos.
8. Visualizar la ruta planificada y la trayectoria recorrida.

---

# Arquitectura del Sistema

La solución desarrollada se divide en tres módulos principales: generación del mapa, planeación de trayectorias y navegación autónoma.

---

## 1. Generación del Mapa de Ocupación

Archivo:

```text
generate_map.py
```

Este módulo genera una representación discreta del entorno mediante una rejilla de ocupación (Occupancy Grid).

A partir de las dimensiones conocidas de la arena y de la ubicación de los obstáculos, el programa construye un mapa binario donde cada celda representa una región libre u ocupada.

Funciones principales:

* Creación de la rejilla de ocupación.
* Modelado de muros perimetrales.
* Modelado de obstáculos cilíndricos.
* Inflado de obstáculos para considerar el tamaño físico del robot.
* Generación de archivos para navegación.

Archivos generados:

```text
maps/
├── occupancy_grid.npy
├── occupancy_grid_raw.npy
├── occupancy_grid_meta.json
└── occupancy_grid_preview.png
```

La resolución utilizada es:

```python
RESOLUTION = 0.05 m/celda
```

obteniendo una rejilla de aproximadamente:

```python
300 × 300 celdas
```

para representar el entorno completo de 15 × 15 m².

---

## 2. Planeación de Trayectorias mediante A*

Archivo:

```text
astarmod.py
```

Este módulo implementa el algoritmo A* para calcular una ruta libre de colisiones entre una posición inicial y una posición objetivo.

Características implementadas:

* Movimientos ortogonales y diagonales.
* Heurística octil para optimizar la búsqueda.
* Prevención de corner cutting.
* Inflado de obstáculos para aumentar la seguridad.
* Conversión entre coordenadas del mundo y coordenadas de rejilla.

La planeación se realiza sobre el mapa de ocupación generado previamente.

El resultado consiste en una secuencia ordenada de puntos que representan la trayectoria óptima hasta la meta.

---

## 3. Navegación y Seguimiento de Trayectoria

Archivo:

```text
navigation_controller.py
```

Este módulo ejecuta la navegación autónoma del robot Pioneer P3-DX.

Responsabilidades principales:

1. Cargar el mapa de ocupación.
2. Obtener la posición actual mediante GPS e IMU.
3. Definir la posición objetivo.
4. Solicitar una trayectoria al algoritmo A*.
5. Seguir la ruta calculada mediante un controlador de seguimiento.
6. Corregir errores de orientación durante el desplazamiento.
7. Alcanzar la meta evitando colisiones.

Durante la navegación el robot actualiza continuamente su posición y ajusta su movimiento para mantenerse sobre la trayectoria planificada.

---

# Flujo General del Sistema

El funcionamiento completo del proyecto puede resumirse en las siguientes etapas:

1. Generación del mapa de ocupación mediante `generate_map.py`.
2. Carga del mapa por parte de `navigation_controller.py`.
3. Planeación de la ruta utilizando las funciones implementadas en `astarmod.py`.
4. Conversión de la ruta desde coordenadas de rejilla a coordenadas del mundo.
5. Seguimiento de la trayectoria por el robot Pioneer P3-DX.
6. Llegada a la posición objetivo.

```text
generate_map.py
        ↓
occupancy_grid.npy
        ↓
astarmod.py (A*)
        ↓
navigation_controller.py
        ↓
Robot Pioneer P3-DX
```
