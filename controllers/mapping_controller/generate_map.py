#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_map.py
===============
Genera el mapa de rejilla de ocupación SINTÉTICO desde las posiciones
conocidas del mundo arena_15x15.wbt.

Ejecutar UNA SOLA VEZ antes de correr navigation_controller:
    python generate_map.py

Guarda:
    maps/occupancy_grid.npy
    maps/occupancy_grid_meta.json
    maps/occupancy_grid_preview.png
"""

import os
import json
import numpy as np
from scipy.ndimage import binary_dilation

# ── Parámetros del mundo ───────────────────────────────────────────────────────
ARENA_SIZE   = 15.0          # metros
RESOLUTION   = 0.05          # m/celda  → grid 300×300 (más detalle)
WALL_THICK   = 0.12          # grosor de muros (m)
CYL_RADIUS   = 0.25          # radio de cada cilindro (m)
INFLATE_R    = 0.40          # margen de seguridad del robot (m)

# Posiciones (X, Z) de los 6 cilindros en el mundo ENU
# Obstáculos cada 2m: ejemplo para (x,z) en pares en rango [-6,6] (con 0 incluido)
CYLINDERS = [
    (x, z)
    for x in range(-6, 7, 3)  # coincide con world.wbt: obstáculos cada 3 m
    for z in range(-6, 7, 3)
    if not (x == 0 and z == 0)  # (opcional) no poner uno en el centro
]

# ── Derivados ──────────────────────────────────────────────────────────────────
ORIGIN    = -ARENA_SIZE / 2          # −7.5
GRID_SIZE = int(ARENA_SIZE / RESOLUTION)   # 300

OUT_DIR = os.path.join(os.path.dirname(__file__), "maps")


def world_to_grid(wx, wz):
    c = int((wx - ORIGIN) / RESOLUTION)
    r = int((wz - ORIGIN) / RESOLUTION)
    return (max(0, min(GRID_SIZE-1, r)),
            max(0, min(GRID_SIZE-1, c)))


def fill_circle(grid, cx, cz, radius):
    """Marca todas las celdas dentro del círculo como ocupadas."""
    r_cells = int(np.ceil(radius / RESOLUTION)) + 1
    cr, cc  = world_to_grid(cx, cz)
    for dr in range(-r_cells, r_cells+1):
        for dc in range(-r_cells, r_cells+1):
            dist = np.hypot(dr * RESOLUTION, dc * RESOLUTION)
            if dist <= radius:
                nr, nc = cr+dr, cc+dc
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                    grid[nr, nc] = 1


def inflate(grid, radius_m):
    r_cells = max(1, int(np.ceil(radius_m / RESOLUTION)))
    y, x    = np.ogrid[-r_cells:r_cells+1, -r_cells:r_cells+1]
    struct  = (x**2 + y**2 <= r_cells**2)
    occ     = binary_dilation(grid >= 1, structure=struct)
    return occ.astype(np.float32)


def build():
    os.makedirs(OUT_DIR, exist_ok=True)

    raw = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    # ── Muros perimetrales ─────────────────────────────────────────────────
    wall_cells = max(1, int(np.ceil(WALL_THICK / RESOLUTION)))
    raw[:wall_cells,  :] = 1   # sur
    raw[-wall_cells:, :] = 1   # norte
    raw[:,  :wall_cells] = 1   # oeste
    raw[:, -wall_cells:] = 1   # este

    # ── Cilindros ──────────────────────────────────────────────────────────
    for cx, cz in CYLINDERS:
        fill_circle(raw, cx, cz, CYL_RADIUS)

    # ── Guardar mapa "real" (sin inflación) ────────────────────────────────
    np.save(os.path.join(OUT_DIR, "occupancy_grid_raw.npy"), raw)

    # ── Inflar para navegación ─────────────────────────────────────────────
    inflated = inflate(raw, INFLATE_R)
    np.save(os.path.join(OUT_DIR, "occupancy_grid.npy"), inflated)

    meta = {
        "resolution" : RESOLUTION,
        "grid_size"  : GRID_SIZE,
        "origin"     : ORIGIN,
        "arena_size" : ARENA_SIZE,
        "inflate_r"  : INFLATE_R,
        "cylinders"  : CYLINDERS,
    }
    with open(os.path.join(OUT_DIR, "occupancy_grid_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # ── Preview ────────────────────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt
        from scipy.ndimage import zoom

        scale  = 2.0
        vis_in = zoom(inflated, scale, order=1)   # bilineal
        vis_rw = zoom(raw,      scale, order=0)   # nearest para bordes nítidos

        img = np.ones((*vis_in.shape, 3))
        img[vis_in >= 1] = [0.20, 0.20, 0.20]    # inflado → gris oscuro
        img[vis_rw >= 1] = [0.05, 0.05, 0.05]    # real → negro

        ext = [ORIGIN, -ORIGIN, ORIGIN, -ORIGIN]
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(np.flipud(img), extent=ext, origin="lower")

        for cx, cz in CYLINDERS:
            circle = plt.Circle((cx, cz), CYL_RADIUS,
                                 color="red", fill=False, lw=1.5, label="_")
        ax.set_title(f"Mapa sintético  {GRID_SIZE}×{GRID_SIZE}  res={RESOLUTION}m")
        ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)")
        ax.grid(True, alpha=0.25)
        plt.tight_layout()
        out_png = os.path.join(OUT_DIR, "occupancy_grid_preview.png")
        plt.savefig(out_png, dpi=150)
        print(f"[GenMap] Preview guardado → {out_png}")
        plt.show()
    except ImportError:
        print("[GenMap] matplotlib no disponible, se omite preview")

    print(f"[GenMap] ✓  Mapa guardado en {OUT_DIR}")
    print(f"          Grid: {GRID_SIZE}×{GRID_SIZE}  |  res: {RESOLUTION} m/cel")
    print(f"          Cilindros: {len(CYLINDERS)}  |  Inflación: {INFLATE_R} m")


if __name__ == "__main__":
    build()
