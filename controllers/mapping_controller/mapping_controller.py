"""
mapping_controller.py
=====================
Controlador de mapeo con rejilla de ocupación (Occupancy Grid) para Pioneer p3dx en Webots.
Usa el sensor LiDAR (SickLms291) para construir el mapa y lo guarda en disco.

Coordenadas Webots (NUE):
  X → Derecha
  Z → Adelante (frente del robot cuando rotation=0)
  Y → Arriba

El mapa se guarda como:
  - occupancy_grid.npy  (array numpy)
  - occupancy_grid.png  (imagen para visualización)
  - occupancy_grid_meta.json (resolución, origen, tamaño)
"""

from controller import Robot, Lidar, GPS, InertialUnit
import numpy as np
import math
import json
import os

# ── Parámetros del mapa ────────────────────────────────────────────────────────
ARENA_SIZE   = 15.0        # metros (arena cuadrada 15×15)
RESOLUTION   = 0.1         # metros por celda
GRID_SIZE    = int(ARENA_SIZE / RESOLUTION)   # 150 × 150 celdas
ORIGIN       = -ARENA_SIZE / 2               # esquina inferior-izquierda (−7.5 m)

# Umbrales log-odds
LOG_ODD_OCC  =  0.85
LOG_ODD_FREE = -0.40
LOG_ODD_MAX  =  5.0
LOG_ODD_MIN  = -5.0

# ── Parámetros de movimiento ───────────────────────────────────────────────────
MAX_SPEED        = 3.0     # rad/s ruedas
TURN_SPEED       = 1.5
OBSTACLE_DIST    = 0.6     # m — distancia mínima frontal antes de girar
SIDE_DIST        = 0.4     # m — distancia lateral

SAVE_PATH = os.path.join(os.path.dirname(__file__), "maps")


# ── Utilidades ─────────────────────────────────────────────────────────────────
def world_to_grid(wx, wz):
    """Convierte coordenadas de mundo (m) a índices de celda."""
    col = int((wx - ORIGIN) / RESOLUTION)
    row = int((wz - ORIGIN) / RESOLUTION)
    col = max(0, min(GRID_SIZE - 1, col))
    row = max(0, min(GRID_SIZE - 1, row))
    return row, col


def bresenham(r0, c0, r1, c1):
    """Genera las celdas entre dos puntos (rayo)."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


def save_map(log_odds_grid):
    """Guarda el mapa en disco (npy + png + json)."""
    os.makedirs(SAVE_PATH, exist_ok=True)

    # Array binario: 0=libre, 1=ocupado, 0.5=desconocido
    prob = 1.0 - 1.0 / (1.0 + np.exp(log_odds_grid))
    binary = np.where(prob > 0.6, 1, np.where(prob < 0.4, 0, 0.5))

    np.save(os.path.join(SAVE_PATH, "occupancy_grid.npy"), binary)

    # Guardar PNG (blanco=libre, negro=ocupado, gris=desconocido)
    try:
        from PIL import Image
        img_array = np.zeros((GRID_SIZE, GRID_SIZE, 3), dtype=np.uint8)
        img_array[binary == 0]   = [255, 255, 255]   # libre
        img_array[binary == 1]   = [0,   0,   0]     # ocupado
        img_array[binary == 0.5] = [128, 128, 128]   # desconocido
        # Voltear para que eje Z apunte hacia arriba en imagen
        img = Image.fromarray(np.flipud(img_array))
        img.save(os.path.join(SAVE_PATH, "occupancy_grid.png"))
        print("[Mapping] PNG guardado.")
    except ImportError:
        print("[Mapping] PIL no disponible; solo se guarda .npy")

    meta = {
        "resolution": RESOLUTION,
        "grid_size":  GRID_SIZE,
        "origin":     ORIGIN,
        "arena_size": ARENA_SIZE,
    }
    with open(os.path.join(SAVE_PATH, "occupancy_grid_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    print(f"[Mapping] Mapa guardado en {SAVE_PATH}")


# ── Controlador principal ──────────────────────────────────────────────────────
def run():
    robot    = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Actuadores
    left_motor  = robot.getDevice("left wheel")
    right_motor = robot.getDevice("right wheel")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    # Sensores de posición (encoders) — para odometría simple
    left_enc  = robot.getDevice("left wheel sensor")
    right_enc = robot.getDevice("right wheel sensor")
    left_enc.enable(timestep)
    right_enc.enable(timestep)

    # GPS e IMU para pose
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)

    # LiDAR
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # Inicializar mapa log-odds
    log_odds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    step_count     = 0
    save_interval  = 200          # guardar mapa cada N pasos
    total_steps    = 6000         # parar mapeo después de estos pasos (~3 min a 32ms)
    wall_follow    = False
    turn_dir       = 1            # 1=izquierda, −1=derecha

    print("[Mapping] Iniciando mapeo de rejilla de ocupación…")
    print(f"[Mapping] Arena {ARENA_SIZE}m × {ARENA_SIZE}m | Resolución {RESOLUTION}m | Grid {GRID_SIZE}×{GRID_SIZE}")

    while robot.step(timestep) != -1 and step_count < total_steps:

        # ── Pose del robot ──────────────────────────────────────────────────
        gps_vals  = gps.getValues()          # [X, Y, Z] en NUE
        rpy       = imu.getRollPitchYaw()
        robot_x   = gps_vals[0]
        robot_z   = gps_vals[2]
        robot_yaw = rpy[1]                   # yaw en NUE = pitch en RPY

        # ── Actualizar mapa con LiDAR ───────────────────────────────────────
        ranges     = lidar.getRangeImage()
        num_rays   = len(ranges)
        fov        = lidar.getFov()
        angle_step = fov / num_rays
        angle_min  = -fov / 2.0

        rr, rc = world_to_grid(robot_x, robot_z)

        for i, dist in enumerate(ranges):
            angle = angle_min + i * angle_step + robot_yaw

            # Celdas libres a lo largo del rayo
            if dist > lidar.getMinRange() and not math.isinf(dist):
                hit_x = robot_x + dist * math.sin(angle)
                hit_z = robot_z + dist * math.cos(angle)
                hr, hc = world_to_grid(hit_x, hit_z)
            else:
                max_range = lidar.getMaxRange()
                hit_x = robot_x + max_range * math.sin(angle)
                hit_z = robot_z + max_range * math.cos(angle)
                hr, hc = world_to_grid(hit_x, hit_z)

            free_cells = bresenham(rr, rc, hr, hc)
            for fr, fc in free_cells[:-1]:
                if 0 <= fr < GRID_SIZE and 0 <= fc < GRID_SIZE:
                    log_odds[fr, fc] = max(LOG_ODD_MIN,
                                           log_odds[fr, fc] + LOG_ODD_FREE)

            # Celda ocupada (hit)
            if not math.isinf(dist) and dist > lidar.getMinRange():
                if 0 <= hr < GRID_SIZE and 0 <= hc < GRID_SIZE:
                    log_odds[hr, hc] = min(LOG_ODD_MAX,
                                           log_odds[hr, hc] + LOG_ODD_OCC)

        # ── Comportamiento de navegación reactiva (random walk) ─────────────
        # Leer rayos frontales (centro ±15°) y laterales
        half = num_rays // 2
        front_slice = ranges[half - 8 : half + 8]
        left_slice  = ranges[half + 8 : half + 24]
        right_slice = ranges[half - 24 : half - 8]

        def min_range(sl):
            vals = [v for v in sl if not math.isinf(v) and v > 0.05]
            return min(vals) if vals else float("inf")

        d_front = min_range(front_slice)
        d_left  = min_range(left_slice)
        d_right = min_range(right_slice)

        if d_front < OBSTACLE_DIST:
            # Girar — elegir dirección con más espacio
            turn_dir = 1 if d_left > d_right else -1
            left_motor.setVelocity(-turn_dir * TURN_SPEED)
            right_motor.setVelocity( turn_dir * TURN_SPEED)
        else:
            # Avanzar con ligeras correcciones laterales
            correction = 0.0
            if d_left < SIDE_DIST:
                correction = -0.5
            elif d_right < SIDE_DIST:
                correction = 0.5
            left_motor.setVelocity(MAX_SPEED + correction)
            right_motor.setVelocity(MAX_SPEED - correction)

        # ── Guardar mapa periódicamente ─────────────────────────────────────
        if step_count % save_interval == 0:
            save_map(log_odds)
            print(f"[Mapping] Paso {step_count}/{total_steps} | "
                  f"Robot ({robot_x:.2f}, {robot_z:.2f}) | "
                  f"Frontal: {d_front:.2f}m")

        step_count += 1

    # Guardar mapa final
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    save_map(log_odds)
    print("[Mapping] ¡Mapeo completado!")


if __name__ == "__main__":
    run()
