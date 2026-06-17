#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
astarmod.py — A* para navegación con mapa de rejilla de ocupación
Basado en el código original de jpiramirez / Nicholas Swift
"""

from warnings import warn
import heapq
import numpy as np
import math


class Node:
    def __init__(self, parent=None, position=None):
        self.parent   = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]


def astar(maze, start, end, allow_diagonal_movement=True):
    """
    Devuelve lista de (row, col) desde start hasta end.
    maze: lista 2D o ndarray. 0 = libre, != 0 = obstáculo.
    """
    start_node = Node(None, start)
    end_node   = Node(None, end)

    open_heap = []
    closed_set = set()
    g_score = {start_node.position: 0.0}
    counter = 0

    def heuristic(pos):
        dx = abs(pos[0] - end_node.position[0])
        dy = abs(pos[1] - end_node.position[1])
        if allow_diagonal_movement:
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
        return dx + dy

    start_node.h = heuristic(start_node.position)
    start_node.f = start_node.h
    heapq.heappush(open_heap, (start_node.f, counter, start_node))

    outer_iterations = 0
    max_iterations   = len(maze) * len(maze[0])

    adjacent_squares = ((0,-1),(0,1),(-1,0),(1,0))
    if allow_diagonal_movement:
        adjacent_squares = (
            (0,-1),(0,1),(-1,0),(1,0),
            (-1,-1),(-1,1),(1,-1),(1,1)
        )

    while open_heap:
        outer_iterations += 1

        if outer_iterations > max_iterations:
            warn("A*: demasiadas iteraciones, ruta parcial devuelta")
            return return_path(current_node)

        _, _, current_node = heapq.heappop(open_heap)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == end_node:
            return return_path(current_node)

        children = []
        for new_pos in adjacent_squares:
            r = current_node.position[0] + new_pos[0]
            c = current_node.position[1] + new_pos[1]

            if r < 0 or r >= len(maze) or c < 0 or c >= len(maze[0]):
                continue
            if maze[r][c] != 0:
                continue

            # Evita "corner cutting" en diagonales: si los vecinos ortogonales
            # que forman la esquina están ocupados, no permitas ese paso.
            if new_pos[0] != 0 and new_pos[1] != 0:
                if maze[current_node.position[0] + new_pos[0]][current_node.position[1]] != 0:
                    continue
                if maze[current_node.position[0]][current_node.position[1] + new_pos[1]] != 0:
                    continue

            children.append(Node(current_node, (r, c)))

        for child in children:
            if child.position in closed_set:
                continue

            # Costo: 1 para 4-vecinos, sqrt(2) para diagonales
            dr = abs(child.position[0] - current_node.position[0])
            dc = abs(child.position[1] - current_node.position[1])
            step_cost = math.sqrt(2) if (dr + dc == 2) else 1.0

            tentative_g = current_node.g + step_cost
            if tentative_g >= g_score.get(child.position, float("inf")):
                continue

            child.g = tentative_g
            child.h = heuristic(child.position)
            child.f = child.g + child.h
            g_score[child.position] = tentative_g
            counter += 1
            heapq.heappush(open_heap, (child.f, counter, child))

    return []   # sin ruta


# ── Utilidades ─────────────────────────────────────────────────────────────────

def path2cells(path):
    """Desempaqueta path en (rows, cols) para indexar o graficar."""
    rows = [p[0] for p in path]
    cols = [p[1] for p in path]
    return rows, cols


def inflate_obstacles(grid, radius_cells=3):
    """
    Dilata las celdas ocupadas (>=1) por radius_cells.
    Devuelve un mapa 2D con 0=libre / 1=bloqueado.
    """
    from scipy.ndimage import binary_dilation
    occupied = np.array(grid) >= 1
    if radius_cells > 0:
        y, x = np.ogrid[-radius_cells:radius_cells+1,
                        -radius_cells:radius_cells+1]
        struct = (x**2 + y**2 <= radius_cells**2)
        occupied = binary_dilation(occupied, structure=struct)
    return occupied.astype(int).tolist()


def world_to_grid(wx, wz, origin, resolution):
    """Coordenadas de mundo (m) → (row, col) en la rejilla."""
    col = int((wx - origin) / resolution)
    row = int((wz - origin) / resolution)
    return row, col


def grid_to_world(row, col, origin, resolution):
    """(row, col) → coordenadas de mundo (x, z) en metros."""
    wx = origin + col * resolution + resolution / 2
    wz = origin + row * resolution + resolution / 2
    return wx, wz


def plan(grid, start_world, goal_world, origin, resolution,
         robot_radius_m=0.35, allow_diagonal=True):
    """
    Planifica ruta de start_world a goal_world.

    Parámetros
    ----------
    grid           : ndarray/lista 2D (0=libre, 1=ocupado)
    start_world    : (x, z) en metros
    goal_world     : (x, z) en metros
    origin         : coordenada de la esquina inferior-izquierda del mapa (m)
    resolution     : metros por celda
    robot_radius_m : radio de inflación de obstáculos (m)
    allow_diagonal : permitir movimientos diagonales en A*

    Retorna
    -------
    Lista de (x, z) en metros, o [] si no hay ruta.
    """
    radius_cells = max(1, int(math.ceil(robot_radius_m / resolution)))
    inflated     = inflate_obstacles(grid, radius_cells)

    start_gc = world_to_grid(*start_world, origin, resolution)
    goal_gc  = world_to_grid(*goal_world,  origin, resolution)

    path_cells = astar(inflated, start_gc, goal_gc, allow_diagonal)

    if not path_cells:
        return []

    return [grid_to_world(r, c, origin, resolution) for r, c in path_cells]


# ── Test rápido ────────────────────────────────────────────────────────────────

def test():
    maze = [
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
    ]
    path = astar(maze, (0,0), (5,6), allow_diagonal_movement=True)
    print("Path:", path)
    print("Cells:", path2cells(path))

# test()
