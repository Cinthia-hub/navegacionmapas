"""
visualize_map.py
================
Script de visualización (ejecutar fuera de Webots):
  - Carga el mapa de rejilla de ocupación
  - Planifica una ruta con A* entre dos puntos
  - Muestra el mapa con la ruta superpuesta

Uso:
    python visualize_map.py [start_x start_z goal_x goal_z]

Ejemplo:
    python visualize_map.py 0 0 5 5
"""

import os
import sys
import json
import numpy as np
import math

# Ruta al mapa
MAP_DIR = os.path.join(os.path.dirname(__file__),
                       "controllers", "mapping_controller", "maps")

# Puntos por defecto
START = (0.0, 0.0)
GOAL  = (5.0, 5.0)


def load_map(map_dir):
    grid_path = os.path.join(map_dir, "occupancy_grid.npy")
    meta_path = os.path.join(map_dir, "occupancy_grid_meta.json")
    grid = np.load(grid_path)
    with open(meta_path) as f:
        meta = json.load(f)
    return grid, meta


def visualize(grid, meta, waypoints, start, goal):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("matplotlib no disponible. Instala con: pip install matplotlib")
        return

    origin     = meta["origin"]
    resolution = meta["resolution"]
    gs         = grid.shape[0]

    # Crear imagen RGB
    img = np.ones((gs, gs, 3))
    img[grid == 0]   = [1.0, 1.0, 1.0]   # libre — blanco
    img[grid == 0.5] = [0.7, 0.7, 0.7]   # desconocido — gris
    img[grid >= 0.6] = [0.1, 0.1, 0.1]   # ocupado — negro

    fig, ax = plt.subplots(figsize=(9, 9))
    extent = [origin, origin + gs * resolution,
              origin, origin + gs * resolution]
    ax.imshow(np.flipud(img), extent=extent, origin="lower")

    # Ruta A*
    if waypoints:
        xs, zs = zip(*waypoints)
        ax.plot(xs, zs, "b-o", linewidth=2, markersize=4, label="Ruta A*")

    # Inicio y objetivo
    ax.plot(*start, "gs", markersize=12, label=f"Inicio {start}")
    ax.plot(*goal,  "r*", markersize=14, label=f"Objetivo {goal}")

    # Obstáculos conocidos
    obs = [(-4,0),(4,0),(0,4),(0,-4),(2.5,2.5),(-2.5,-2.5)]
    for ox, oz in obs:
        circle = plt.Circle((ox, oz), 0.25, color="orange", alpha=0.5)
        ax.add_patch(circle)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Mapa de Rejilla de Ocupación + Ruta A*")
    ax.legend(loc="upper right")
    ax.set_xlim(origin, -origin)
    ax.set_ylim(origin, -origin)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(MAP_DIR, "map_with_path.png"), dpi=150)
    print(f"[Viz] Imagen guardada en {MAP_DIR}/map_with_path.png")
    plt.show()


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) >= 4:
        start = (float(args[0]), float(args[1]))
        goal  = (float(args[2]), float(args[3]))
    else:
        start, goal = START, GOAL

    print(f"[Viz] Cargando mapa desde {MAP_DIR}…")
    try:
        grid, meta = load_map(MAP_DIR)
    except FileNotFoundError:
        print("[Viz] Mapa no encontrado. Ejecuta mapping_controller primero.")
        sys.exit(1)

    sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                    "controllers", "navigation_controller"))
    from astarmod import plan

    waypoints = plan(
        binary_grid    = grid,
        start_world    = start,
        goal_world     = goal,
        origin         = meta["origin"],
        resolution     = meta["resolution"],
        robot_radius_m = 0.40,
        do_smooth      = True,
    )

    print(f"[Viz] Waypoints: {len(waypoints)}")
    visualize(grid, meta, waypoints, start, goal)
