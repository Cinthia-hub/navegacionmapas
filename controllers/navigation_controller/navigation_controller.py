"""
Autora: Cithia Camila Bravo Marmolejo
NUA: 148241
Materia: Robótica Móvil
Docente: Juan Pablo Ignacio Ramírez Paredes
Semestre: Enero-Junio 2026

Proyecto 3
Navegación A* con seguimiento de ruta, evasión de obstáculos y visualización.
"""

import os, sys, math, json, threading, random
import numpy as np
from scipy.ndimage import binary_dilation

# ══════════════════════════════════════════════════
#  PARÁMETROS
# ══════════════════════════════════════════════════
def _random_goal(margin=1.2, arena=6.5, cyl_r=0.9):
    """Genera (x, y) aleatorio libre de obstáculos."""
    obstacles = [
        (x, y)
        for x in range(-6, 7, 3)
        for y in range(-6, 7, 3)
        if not (x == 0 and y == 0)
    ]
    rng = random.Random()   # usa semilla del tiempo → diferente cada ejecución
    for _ in range(500):
        gx = rng.uniform(-arena, arena)
        gy = rng.uniform(-arena, arena)
        # evitar punto de inicio (centro)
        if math.hypot(gx, gy) < 1.0:
            continue
        # evitar cilindros
        if any(math.hypot(gx-ox, gy-oy) < cyl_r for ox, oy in obstacles):
            continue
        return round(gx, 2), round(gy, 2)
    return 5.0, 5.0   # fallback

GOAL_X, GOAL_Y = _random_goal()
MAP_DIR      = os.path.join(os.path.dirname(__file__), "maps")

# Parámetros principales de navegación.
MAX_SPEED    =  5.0
LOOKAHEAD    =  0.8
GOAL_THR     =  0.25
KP, KI, KD  =  4.0, 0.0, 0.8

# Distancia sonar de emergencia.
EMERG_DIST   =  0.30

VIZ_HZ       =  10
REPLAN_STEPS =  150

sys.path.insert(0, os.path.dirname(__file__))
from astarmod import plan
from controller import Robot

# 24 obstáculos exactos del world.wbt.
OBSTACLES_XY = [
    (x, y)
    for x in range(-6, 7, 3)
    for y in range(-6, 7, 3)
    if not (x == 0 and y == 0)
]

# Conversión mundo ↔ grid y manejo de ángulos.
def w2g(wx, wy, origin, res, gs):
    r = int((wy - origin) / res)
    c = int((wx - origin) / res)
    return max(0, min(gs-1, r)), max(0, min(gs-1, c))

def adiff(a, b):
    d = a - b
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

def sonar_valid(v):
    """El Pioneer devuelve 0.0 cuando no hay eco. Ignorar esos valores."""
    return v > 0.05


# ── PID ─────────────────────────────────────────────────────────────
# Control de rumbo suave para evitar correcciones bruscas.
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._ie = self._pe = 0.0
    def reset(self): self._ie = self._pe = 0.0
    def update(self, e, dt):
        self._ie = max(-3.0, min(3.0, self._ie + e * dt))
        d = (e - self._pe) / max(dt, 1e-6)
        self._pe = e
        return self.kp*e + self.ki*self._ie + self.kd*d


# ── Pure Pursuit ─────────────────────────────────────────────────────
# Toma el waypoint más cercano dentro del radio lookahead.
def lookahead_pt(wps, rx, ry, lh):
    for i in range(len(wps)-1, -1, -1):
        if math.hypot(wps[i][0]-rx, wps[i][1]-ry) <= lh:
            return i, wps[i][0], wps[i][1]
    return 0, wps[0][0], wps[0][1]


# ── Mapa ─────────────────────────────────────────────────────────────
# Construye un mapa sintético inflado para navegación segura.
def build_map():
    ARENA=15.0; RES=0.05; ORIGIN=-7.5; GS=300
    CYL_R=0.25; ROBOT_R=0.40

    raw = np.zeros((GS, GS), dtype=np.float32)
    W = int(math.ceil(0.10 / RES))
    raw[:W,:]=1; raw[-W:,:]=1; raw[:,:W]=1; raw[:,-W:]=1

    CR = int(math.ceil(CYL_R / RES)) + 1
    for cx, cy in OBSTACLES_XY:
        r0, c0 = w2g(cx, cy, ORIGIN, RES, GS)
        for dr in range(-CR, CR+1):
            for dc in range(-CR, CR+1):
                if dr**2+dc**2 <= CR**2:
                    nr, nc = r0+dr, c0+dc
                    if 0 <= nr < GS and 0 <= nc < GS:
                        raw[nr, nc] = 1

    IR = int(math.ceil(ROBOT_R / RES))
    y, x = np.ogrid[-IR:IR+1, -IR:IR+1]
    inflated = binary_dilation(raw>=1, structure=(x**2+y**2<=IR**2)).astype(np.float32)

    os.makedirs(MAP_DIR, exist_ok=True)
    np.save(os.path.join(MAP_DIR, "occupancy_grid.npy"),     inflated)
    np.save(os.path.join(MAP_DIR, "occupancy_grid_raw.npy"), raw)
    meta = {"resolution":RES, "origin":ORIGIN, "grid_size":GS, "arena_size":ARENA}
    with open(os.path.join(MAP_DIR, "occupancy_grid_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[Nav] Mapa construido {GS}×{GS}  {len(OBSTACLES_XY)} obstáculos")
    return inflated, meta


# ══════════════════════════════════════════════════
#  Estado compartido
# ══════════════════════════════════════════════════
# Comparte pose, ruta, rastro y estado de finalización entre hilos.
class State:
    def __init__(self):
        self.lock      = threading.Lock()
        self.rx = self.ry = self.yaw = 0.0
        self.waypoints = []
        self.wp_idx    = 0
        self.trail     = []
        self.done      = False
        self.dirty     = False

    def write(self, rx, ry, yaw, wps, wp_idx, done=False):
        with self.lock:
            self.rx, self.ry, self.yaw = rx, ry, yaw
            self.waypoints = wps
            self.wp_idx    = wp_idx
            self.trail.append((rx, ry))
            self.done  = done
            self.dirty = True

    def read(self):
        with self.lock:
            self.dirty = False
            return (self.rx, self.ry, self.yaw,
                    list(self.waypoints), self.wp_idx,
                    list(self.trail), self.done)


# ══════════════════════════════════════════════════
#  Hilo Webots
# ══════════════════════════════════════════════════
# Aquí se leen sensores, se calcula la ruta y se mandan velocidades.
def webots_loop(state: State, grid, meta, ready: threading.Event):
    origin = meta["origin"]
    res    = meta["resolution"]
    gs     = grid.shape[0]

    robot = Robot()
    ts    = int(robot.getBasicTimeStep())
    dt    = ts / 1000.0

    lm = robot.getDevice("left wheel")
    rm = robot.getDevice("right wheel")
    lm.setPosition(float("inf")); lm.setVelocity(0)
    rm.setPosition(float("inf")); rm.setVelocity(0)

    gps = robot.getDevice("gps");           gps.enable(ts)
    imu = robot.getDevice("inertial unit"); imu.enable(ts)

    # Habilitar los 16 sonares del Pioneer.
    sonars = []
    for i in range(16):
        s = robot.getDevice(f"so{i}")
        if s:
            s.enable(ts)
            sonars.append(s)

    pid     = PID(KP, KI, KD)
    visited = np.zeros((gs, gs), dtype=bool)

    def replan(rx, ry, block_visited=True):
        aug = grid.copy()
        if block_visited:
            gg  = w2g(GOAL_X,  GOAL_Y,  origin, res, gs)
            gs0 = w2g(rx, ry,  origin, res, gs)
            for r, c in np.argwhere(visited):
                if (r,c) != gg and (r,c) != gs0:
                    aug[r, c] = 1.0
        return plan(grid=aug,
                    start_world=(rx, ry),
                    goal_world=(GOAL_X, GOAL_Y),
                    origin=origin, resolution=res,
                    robot_radius_m=0.0,
                    allow_diagonal=True)

    # Primera lectura para inicializar la navegación.
    robot.step(ts)
    gv     = gps.getValues()
    sx, sy = gv[0], gv[1]
    print(f"[Nav] Inicio ({sx:.2f},{sy:.2f})  Objetivo ({GOAL_X},{GOAL_Y})")

    waypoints = replan(sx, sy, block_visited=False)
    if not waypoints:
        print("[Nav] Sin ruta. Abortando."); ready.set(); return
    print(f"[Nav] {len(waypoints)} waypoints")

    state.write(sx, sy, 0.0, waypoints, 0)
    ready.set()

    wp_idx    = 0
    step_n    = 0
    viz_every = max(1, int(1.0 / (VIZ_HZ * dt)))
    emerg_cnt = 0
    last_pos  = (sx, sy)
    stuck_cnt = 0
    stuck_int = int(3.0 / dt)   # pasos equivalentes a 3 segundos

    while robot.step(ts) != -1:
        gv  = gps.getValues()
        rx  = gv[0]
        ry  = gv[1]                       # Y es el eje norte-sur
        yaw = imu.getRollPitchYaw()[2]    # yaw=0 → mira +X  (confirmado con diagnostico2)

        # marcar visitado
        vr, vc = w2g(rx, ry, origin, res, gs)
        visited[vr, vc] = True

        # ¿llegamos?
        dist_goal = math.hypot(GOAL_X-rx, GOAL_Y-ry)
        if dist_goal < GOAL_THR:
            print(f"[Nav] ¡Objetivo alcanzado! dist={dist_goal:.3f}m")
            lm.setVelocity(0); rm.setVelocity(0)
            state.write(rx, ry, yaw, waypoints, wp_idx, done=True)
            break

        # Detector de atasco cada 3 s.
        if step_n % stuck_int == 0 and step_n > 0:
            moved = math.hypot(rx-last_pos[0], ry-last_pos[1])
            if moved < 0.08:
                stuck_cnt += 1
                print(f"[Nav] Atasco #{stuck_cnt}, replanificando…")
                nw = replan(rx, ry)
                if nw: waypoints = nw; wp_idx = 0; pid.reset()
            else:
                stuck_cnt = 0
            last_pos = (rx, ry)

        # Replanificación periódica para refrescar el camino.
        if step_n % REPLAN_STEPS == 0 and step_n > 0:
            nw = replan(rx, ry)
            if nw: waypoints = nw; wp_idx = 0; pid.reset()

        # Consumir waypoints ya alcanzados.
        while wp_idx < len(waypoints)-1:
            if math.hypot(waypoints[wp_idx][0]-rx,
                          waypoints[wp_idx][1]-ry) < LOOKAHEAD * 0.5:
                wp_idx += 1
            else:
                break

        # Pure Pursuit → ángulo objetivo.
        _, tx, ty   = lookahead_pt(waypoints[wp_idx:], rx, ry, LOOKAHEAD)
        target_ang  = math.atan2(ty - ry, tx - rx)   # yaw=0→+X: atan2 estándar
        heading_err = adiff(target_ang, yaw)

        # Control PID del rumbo.
        omega  = pid.update(heading_err, dt)
        omega  = max(-MAX_SPEED, min(MAX_SPEED, omega))
        v_base = MAX_SPEED * min(1.0, dist_goal / 1.5)

        vl = v_base - omega
        vr = v_base + omega
        vm = max(abs(vl), abs(vr))
        if vm > MAX_SPEED:
            vl *= MAX_SPEED / vm
            vr *= MAX_SPEED / vm

        # ── evasión sonar ─────────────────────────────────────────────
        # so3=frontal-izq, so4=frontal-der, so1-so2=lateral-izq, so5-so6=lateral-der.
        # Ignorar valores 0.0 (= sin eco = lejos).
        if sonars:
            def sd(i):
                v = sonars[i].getValue() if i < len(sonars) else 0.0
                return v if sonar_valid(v) else 9999.0

            d_front  = min(sd(3), sd(4))
            d_fleft  = min(sd(1), sd(2))
            d_fright = min(sd(5), sd(6))

            if d_front < EMERG_DIST:
                emerg_cnt += 1
                # Girar hacia el lado con más espacio.
                if d_fleft >= d_fright:
                    vl, vr = -MAX_SPEED*0.4,  MAX_SPEED*0.4
                else:
                    vl, vr =  MAX_SPEED*0.4, -MAX_SPEED*0.4
                if emerg_cnt > 50:
                    nw = replan(rx, ry)
                    if nw: waypoints = nw; wp_idx = 0; pid.reset()
                    emerg_cnt = 0
            else:
                emerg_cnt = 0
                # Correcciones suaves laterales.
                if d_fleft < EMERG_DIST * 1.5 and d_fleft < d_fright:
                    vl += 0.3; vr -= 0.3
                elif d_fright < EMERG_DIST * 1.5:
                    vl -= 0.3; vr += 0.3

        lm.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, vl)))
        rm.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, vr)))

        if step_n % viz_every == 0:
            state.write(rx, ry, yaw, waypoints, wp_idx)

        if step_n % 60 == 0:
            print(f"[Nav] t={robot.getTime():.1f}s  "
                  f"pos=({rx:.2f},{ry:.2f})  yaw={math.degrees(yaw):.1f}°  "
                  f"→({tx:.2f},{ty:.2f})  herr={math.degrees(heading_err):.1f}°  "
                  f"dist={dist_goal:.2f}m  wp={wp_idx}/{len(waypoints)}")
        step_n += 1


# ══════════════════════════════════════════════════
#  Ventana matplotlib — hilo PRINCIPAL
# ══════════════════════════════════════════════════
# Dibuja mapa, rastro, ruta restante y pose actual.
def run_viz(state: State, grid, meta, ready: threading.Event):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.animation import FuncAnimation

    ready.wait()

    origin = meta["origin"]
    res    = meta["resolution"]
    gs     = grid.shape[0]
    arena  = meta["arena_size"]
    ext    = [origin, origin+arena, origin, origin+arena]

    # Cargar el mapa raw si existe, para mostrar los obstáculos reales.
    raw_path = os.path.join(MAP_DIR, "occupancy_grid_raw.npy")
    raw = np.load(raw_path) if os.path.exists(raw_path) else grid

    # Construir imagen base estilo "imagen de referencia".
    # Fondo morado oscuro, obstáculos cyan, como en la foto.
    base = np.zeros((gs, gs, 3), dtype=np.float32)
    base[:, :]        = [0.22, 0.05, 0.30]
    base[grid >= 0.5] = [0.08, 0.40, 0.44]
    base[raw  >= 0.5] = [0.22, 0.70, 0.72]

    fig, ax = plt.subplots(figsize=(7, 7))
    fig.patch.set_facecolor("#140820")
    ax.set_facecolor("#140820")
    plt.tight_layout(pad=1.8)

    ax.imshow(np.flipud(base), extent=ext, origin="lower",
              interpolation="nearest", zorder=0)

    # Inicio y objetivo fijos.
    with state.lock:
        sx = state.trail[0][0] if state.trail else 0.0
        sy = state.trail[0][1] if state.trail else 0.0

    ax.plot(sx, sy, 's', color="lime", ms=10, zorder=5,
            markeredgecolor='white', mew=0.8, label="Inicio")
    ax.plot(GOAL_X, GOAL_Y, '*', color="red", ms=15, zorder=5,
            markeredgecolor='white', mew=0.8, label="Objetivo")

    # Elementos dinámicos.
    trail_ln, = ax.plot([], [], '-', color=(0.0, 0.90, 0.90),
                        lw=1.8, alpha=0.75, zorder=2, label="Rastro")
    path_ln,  = ax.plot([], [], '-', color=(1.0, 0.85, 0.0),
                        lw=2.0, zorder=3, label="Ruta A*")
    robot_pt, = ax.plot([], [], 'o', color="white", ms=9,
                        zorder=6, label="Robot")
    qv = ax.quiver(sx, sy, 0.4, 0, angles="xy", scale_units="xy", scale=1,
                   color="white", width=0.012, zorder=7)

    info = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top",
                   fontsize=8, color="white",
                   bbox=dict(boxstyle="round,pad=0.3",
                             fc="#140820", alpha=0.80, ec="#555"))

    patches = [
        mpatches.Patch(color=(0.22,0.70,0.72), label="Obstáculo"),
        mpatches.Patch(color=(0.08,0.40,0.44), label="Zona segura"),
        mpatches.Patch(color=(0.0, 0.90,0.90), label="Rastro"),
        mpatches.Patch(color=(1.0, 0.85,0.00), label="Ruta A*"),
        mpatches.Patch(color="white",           label="Robot"),
        mpatches.Patch(color="lime",            label="Inicio"),
        mpatches.Patch(color="red",             label="Objetivo"),
    ]
    ax.legend(handles=patches, loc="lower right", fontsize=7,
              framealpha=0.6, facecolor="#140820", labelcolor="white")
    ax.set_xlabel("X (m)", color="white", fontsize=9)
    ax.set_ylabel("Y (m)", color="white", fontsize=9)
    ax.tick_params(colors="white")
    for sp in ax.spines.values(): sp.set_edgecolor("#555")
    ax.set_xlim(origin, origin+arena)
    ax.set_ylim(origin, origin+arena)
    ax.grid(True, alpha=0.15, lw=0.5, color="white")
    ax.set_title("Navegación A* — Pioneer p3dx", color="white", fontsize=10)

    def frame(_):
        if not state.dirty:
            return trail_ln, path_ln, robot_pt, qv, info

        rx, ry, yaw, wps, wp_idx, trail, done = state.read()

        if len(trail) > 1:
            trail_ln.set_data([p[0] for p in trail],
                              [p[1] for p in trail])

        rem = wps[wp_idx:]
        path_ln.set_data(
            [p[0] for p in rem] if rem else [],
            [p[1] for p in rem] if rem else []
        )

        robot_pt.set_data([rx], [ry])

        # Flecha de dirección del robot: yaw=0→+X.
        al = 0.55
        qv.set_offsets([[rx, ry]])
        qv.set_UVC(math.cos(yaw)*al, math.sin(yaw)*al)

        dist_g = math.hypot(GOAL_X-rx, GOAL_Y-ry)
        info.set_text(f"Pos : ({rx:.2f}, {ry:.2f})\n"
                      f"Yaw : {math.degrees(yaw):.1f}°\n"
                      f"Dist: {dist_g:.2f} m  WP:{wp_idx}/{len(wps)}")

        if done:
            ax.set_title("Navegación A* — ✓ OBJETIVO ALCANZADO",
                         color="lime", fontsize=10)

        return trail_ln, path_ln, robot_pt, qv, info

    ani = FuncAnimation(fig, frame, interval=int(1000/VIZ_HZ),
                        blit=False, cache_frame_data=False)
    plt.show(block=True)

    os.makedirs(MAP_DIR, exist_ok=True)
    fig.savefig(os.path.join(MAP_DIR, "nav_result.png"), dpi=130,
                bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[Viz] Guardado → {MAP_DIR}/nav_result.png")


# ══════════════════════════════════════════════════
#  Entrada principal
# ══════════════════════════════════════════════════
def run():
    print(f"[Nav] 🎯 Objetivo aleatorio: ({GOAL_X}, {GOAL_Y})")
    grid, meta = build_map()
    state      = State()
    ready      = threading.Event()

    wb = threading.Thread(target=webots_loop,
                          args=(state, grid, meta, ready),
                          daemon=True)
    wb.start()
    run_viz(state, grid, meta, ready)
    wb.join(timeout=2)


if __name__ == "__main__":
    run()