# Descripción del Proyecto

Este proyecto tiene como objetivo implementar un sistema de exploración autónoma para un robot móvil Pioneer P3-DX dentro del simulador Webots.

Durante la exploración, el robot debe desplazarse de manera segura evitando colisiones con obstáculos y muros, mientras construye simultáneamente un mapa de ocupación del entorno utilizando información obtenida mediante un sensor Lidar.

Una característica importante del sistema desarrollado es que el tamaño del mapa es inicialmente desconocido para el robot. Por esta razón, se implementó una estructura de mapa capaz de crecer dinámicamente conforme se exploran nuevas regiones del entorno.

---

# Características del Entorno de Simulación

El escenario utilizado cumple con las siguientes especificaciones:

* Área de trabajo comprendida entre 5×5 m² y 25×25 m².
* Robot móvil Pioneer P3-DX.
* Diez o más obstáculos distribuidos dentro del escenario.
* Obstáculos con diámetro mínimo de 0.5 m y altura de 1.5 m.
* Muros perimetrales de 1.5 m de altura.
* Sensor Lidar para percepción del entorno.
* GPS y brújula para estimación de pose.

---

# Objetivos

## Objetivo General

Desarrollar un sistema capaz de explorar autónomamente un entorno desconocido mientras construye un mapa de ocupación utilizando información proveniente de un sensor Lidar.

## Objetivos Específicos

1. Implementar navegación autónoma sin colisiones.
2. Diseñar una estrategia de exploración que permita cubrir la mayor superficie posible del entorno.
3. Generar un mapa de ocupación determinístico.
4. Utilizar información proveniente del sensor Lidar para actualizar el mapa.
5. Permitir el crecimiento dinámico del mapa cuando la exploración alcance sus límites.
6. Visualizar en tiempo real el mapa generado.

---

# Herramientas Utilizadas

## Software

* Webots
* Python 3

## Bibliotecas

* NumPy
* Matplotlib
* Math
* Random
* Time

---

# Representación del Mapa

El entorno se representa mediante una rejilla de ocupación (Occupancy Grid).

Cada celda del mapa puede encontrarse en uno de tres estados:

| Estado      | Valor |
| ----------- | ----- |
| Desconocido | -1    |
| Libre       | 0     |
| Ocupado     | 1     |

```python
UNKNOWN = -1
FREE = 0
OCC = 1
```

La resolución utilizada es:

```python
RESOLUTION = 0.07 m/celda
```

El mapa inicia con un tamaño supuesto de:

```python
INIT_SIZE = 50
```

equivalente aproximadamente a un área inicial de 5×5 m².

---

# Crecimiento Dinámico del Mapa

Como el robot desconoce previamente el tamaño real del entorno, se implementó una estrategia de expansión dinámica.

Cuando una observación del Lidar cae fuera de los límites actuales del mapa:

1. Se detecta qué borde fue alcanzado.
2. Se agregan nuevas filas o columnas únicamente en la dirección necesaria.
3. Se actualizan automáticamente los desplazamientos del sistema de coordenadas.

Esto permite:

* Reducir el uso de memoria.
* Mantener un mapa compacto.
* Adaptarse a entornos de tamaño desconocido.

---

# Transformación de Coordenadas

Para relacionar el sistema de coordenadas global con el mapa de ocupación se utiliza la función:

```python
world_to_map(x, y)
```

La transformación considera:

* Resolución espacial.
* Desplazamiento del origen del mapa.
* Índices de filas y columnas.

De esta manera es posible convertir cualquier posición del entorno en una celda de la rejilla.

---

# Construcción del Mapa

## Adquisición de Datos

El sensor Lidar proporciona mediciones de distancia en múltiples direcciones alrededor del robot.

Para cada rayo se obtiene:

* Distancia medida.
* Ángulo relativo.
* Posición del punto de impacto.

---

## Ray Tracing

Se utiliza el algoritmo de Bresenham para recorrer las celdas atravesadas por cada rayo.

```python
bresenham(i0, j0, i1, j1)
```

Las celdas atravesadas son marcadas como:

* Libres (FREE)

mientras que la celda correspondiente al punto de impacto se marca como:

* Ocupada (OCC)

---

## Actualización Determinística

Para incrementar la robustez frente a lecturas erróneas:

* Se lleva un contador de observaciones libres.
* Se lleva un contador de observaciones ocupadas.

Una celda es declarada ocupada únicamente cuando existe suficiente evidencia acumulada.

Esto permite reducir falsos positivos en el mapa final.

---

# Estrategia de Exploración

La exploración implementada combina tres comportamientos:

## 1. Avance Directo

Cuando no existen obstáculos cercanos, el robot avanza hacia adelante a velocidad elevada.

---

## 2. Evasión Reactiva

Cuando el Lidar detecta obstáculos frente al robot:

* Se reduce la velocidad.
* Se selecciona la dirección más despejada.
* Se ejecuta una maniobra de giro controlada.

Esto evita colisiones contra obstáculos y muros.

---

## 3. Exploración Aleatoria

Periódicamente se introducen cambios de dirección aleatorios.

Estos giros permiten:

* Evitar ciclos repetitivos.
* Explorar nuevas regiones.
* Incrementar la cobertura total del entorno.

---

# Visualización

El mapa generado se muestra simultáneamente mediante:

## Display de Webots

Representación rápida dentro del simulador.

Colores utilizados:

* Gris: desconocido.
* Blanco: libre.
* Negro: ocupado.
* Rojo: posición del robot.

## Matplotlib

Visualización detallada en tiempo real del crecimiento y actualización del mapa.

---

# Condiciones de Finalización

La simulación termina cuando ocurre alguna de las siguientes condiciones:

## Tiempo Máximo

```python
180 segundos
```

---

## Estancamiento del Mapa

Si durante varias iteraciones consecutivas el número de nuevas celdas conocidas es muy pequeño, se considera que el robot ya no está explorando regiones nuevas.

En este caso:

* Se detiene la exploración.
* Se guarda el mapa generado.

---

# Archivos Generados

Durante la ejecución se almacena:

```text
grid.npy
```

que contiene la matriz final del mapa de ocupación para análisis posterior.

---

# Ejecución

1. Abrir el proyecto en Webots.
2. Cargar el mundo correspondiente.
3. Asociar el controlador Python al robot Pioneer P3-DX.
4. Iniciar la simulación.
5. Observar la exploración y construcción del mapa en tiempo real.
6. Revisar el archivo generado al finalizar la ejecución.
