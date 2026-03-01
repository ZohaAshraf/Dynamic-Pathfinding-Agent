"""
Dynamic Pathfinding Agent with Real-Time Replanning
=====================================================
University Assignment – Informed Search Algorithms
Author : [Your Name]
Student ID : [Your ID]
Course : Artificial Intelligence
Date   : 2025

Description:
    Grid-based pathfinding visualiser that implements A* and Greedy
    Best-First Search with step-by-step animation, interactive map
    editing, and a dynamic obstacle / replan mode.

Dependencies:
    pip install pygame

Run:
    python pathfinding_agent.py
"""

import pygame
import heapq
import random
import math
import time
import sys

# ─────────────────────────────────────────────────────────────────────
#   CONSTANTS – window layout
# ─────────────────────────────────────────────────────────────────────

SCREEN_W  = 1200
SCREEN_H  = 720
PANEL_W   = 280        # right-hand control / metrics panel
GRID_W    = SCREEN_W - PANEL_W
GRID_H    = SCREEN_H
FPS       = 60         # target frame rate


# ─────────────────────────────────────────────────────────────────────
#   COLOURS  – professional deep-slate theme
# ─────────────────────────────────────────────────────────────────────

BG_COLOUR        = (15,  23,  42)   # deep slate  #0f172a
PANEL_BG         = (10,  15,  30)
GRID_LINE        = (40,  55,  80)
EMPTY_CELL       = (30,  42,  60)
START_COLOUR     = (16,  185, 129)  # emerald green  #10b981
GOAL_COLOUR      = (59,  130, 246)  # royal blue     #3b82f6
WALL_COLOUR      = (31,  41,  55)   # dark charcoal  #1f2937
FRONTIER_COLOUR  = (245, 158, 11)   # amber          #f59e0b
VISITED_COLOUR   = (239, 68,  68)   # muted red      #ef4444
PATH_COLOUR      = (34,  197, 94)   # bright lime    #22c55e
AGENT_COLOUR     = (250, 250, 250)
TEXT_COLOUR      = (220, 230, 245)
ACCENT_COLOUR    = (99,  179, 237)
DIM_TEXT         = (100, 120, 150)
BTN_NORMAL       = (31,  52,  80)
BTN_HOVER        = (50,  80, 120)
BTN_ON           = (16,  130, 100)
BTN_DANGER       = (120, 30,  30)
DIVIDER          = (40,  55,  80)


# ─────────────────────────────────────────────────────────────────────
#   NODE CLASS
# ─────────────────────────────────────────────────────────────────────

class Node:
    """
    Represents one cell in the grid.

    row, col  : grid coordinates
    is_wall   : True if this cell is an obstacle
    g         : cost from start (used by A*)
    h         : heuristic estimate to goal
    f         : g + h  (evaluation function)
    parent    : predecessor node on the current path
    """

    def __init__(self, row, col):
        self.row     = row
        self.col     = col
        self.is_wall = False
        self.g       = float('inf')
        self.h       = 0.0
        self.f       = float('inf')
        self.parent  = None

    # heapq needs this to compare two Node objects
    def __lt__(self, other):
        return self.f < other.f

    def reset(self):
        """Clear search state so we can reuse the node in a fresh search."""
        self.g      = float('inf')
        self.h      = 0.0
        self.f      = float('inf')
        self.parent = None


# ─────────────────────────────────────────────────────────────────────
#   GRID CLASS
# ─────────────────────────────────────────────────────────────────────

class Grid:
    """
    Manages the 2-D array of Node objects.

    Provides helpers for:
      - Building / resetting the grid
      - Placing / removing walls
      - Getting valid neighbours (4-directional)
      - Random obstacle generation
      - Dynamic obstacle spawning
    """

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

        # Build 2-D list of nodes
        self.nodes = [
            [Node(r, c) for c in range(cols)]
            for r in range(rows)
        ]

        # Default start = top-left, goal = bottom-right
        self.start_pos = (0, 0)
        self.goal_pos  = (rows - 1, cols - 1)

    def node(self, row, col):
        """Return the Node at (row, col), or None if out of bounds."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.nodes[row][col]
        return None

    def start_node(self):
        r, c = self.start_pos
        return self.nodes[r][c]

    def goal_node(self):
        r, c = self.goal_pos
        return self.nodes[r][c]

    def get_neighbours(self, n):
        """
        Return passable 4-directional neighbours.
        No diagonals – only Up / Down / Left / Right.
        """
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        result = []
        for dr, dc in directions:
            nb = self.node(n.row + dr, n.col + dc)
            if nb is not None and not nb.is_wall:
                result.append(nb)
        return result

    def clear_walls(self):
        for row in self.nodes:
            for n in row:
                n.is_wall = False

    def generate_random_map(self, density=0.30):
        """
        Place walls randomly at the given density (0.0 – 1.0).
        Start and goal cells are always kept clear.
        """
        self.clear_walls()
        for row in self.nodes:
            for n in row:
                pos = (n.row, n.col)
                if pos == self.start_pos or pos == self.goal_pos:
                    continue
                if random.random() < density:
                    n.is_wall = True

    def reset_search_costs(self):
        """Reset g, h, f, parent on every node before a fresh search."""
        for row in self.nodes:
            for n in row:
                n.reset()

    def spawn_random_obstacle(self, agent_pos):
        """
        Place one new wall on a random empty cell that is NOT the
        start, goal, or agent's current position.
        Returns (row, col) of the new wall, or None.
        """
        candidates = []
        for row in self.nodes:
            for n in row:
                pos = (n.row, n.col)
                if n.is_wall:
                    continue
                if pos in (self.start_pos, self.goal_pos, agent_pos):
                    continue
                candidates.append(n)

        if not candidates:
            return None

        chosen = random.choice(candidates)
        chosen.is_wall = True
        return (chosen.row, chosen.col)


# ─────────────────────────────────────────────────────────────────────
#   HEURISTIC FUNCTIONS
# ─────────────────────────────────────────────────────────────────────

def manhattan(node, goal):
    """h(n) = |row_n - row_goal| + |col_n - col_goal|"""
    return abs(node.row - goal.row) + abs(node.col - goal.col)


def euclidean(node, goal):
    """h(n) = sqrt((row_n - row_goal)^2 + (col_n - col_goal)^2)"""
    dr = node.row - goal.row
    dc = node.col - goal.col
    return math.sqrt(dr * dr + dc * dc)


def get_heuristic_fn(name):
    return euclidean if name == "Euclidean" else manhattan


# ─────────────────────────────────────────────────────────────────────
#   SEARCH ALGORITHMS
# ─────────────────────────────────────────────────────────────────────

def astar(grid, heuristic_fn):
    """
    A* Search  –  f(n) = g(n) + h(n)

    Finds the optimal (shortest) path from start to goal.
    Explores the node with the lowest combined real + estimated cost.

    Returns:
        path   : list of Nodes start -> goal  (empty = no path)
        closed : set of fully-explored Nodes
        frames : list of (frontier_snapshot, closed_snapshot) for animation
    """
    grid.reset_search_costs()
    start = grid.start_node()
    goal  = grid.goal_node()

    start.g = 0
    start.h = heuristic_fn(start, goal)
    start.f = start.g + start.h

    open_heap = []
    heapq.heappush(open_heap, (start.f, start))
    open_set  = {start}
    closed    = set()
    frames    = []

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue  # stale heap entry

        open_set.discard(current)
        closed.add(current)
        frames.append((frozenset(open_set), frozenset(closed)))

        if current is goal:
            return _build_path(goal), closed, frames

        for nb in grid.get_neighbours(current):
            if nb in closed:
                continue
            new_g = current.g + 1          # uniform edge cost = 1
            if new_g < nb.g:
                nb.g      = new_g
                nb.h      = heuristic_fn(nb, goal)
                nb.f      = nb.g + nb.h
                nb.parent = current
                if nb not in open_set:
                    heapq.heappush(open_heap, (nb.f, nb))
                    open_set.add(nb)

    return [], closed, frames   # no path found


def greedy_bfs(grid, heuristic_fn):
    """
    Greedy Best-First Search  –  f(n) = h(n)

    Faster than A* but NOT guaranteed to find the shortest path.
    Only uses the heuristic; ignores actual path cost.

    Same return signature as astar().
    """
    grid.reset_search_costs()
    start = grid.start_node()
    goal  = grid.goal_node()

    start.h = heuristic_fn(start, goal)
    start.f = start.h
    start.g = 0

    open_heap = []
    heapq.heappush(open_heap, (start.f, start))
    open_set = {start}
    closed   = set()
    frames   = []

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        open_set.discard(current)
        closed.add(current)
        frames.append((frozenset(open_set), frozenset(closed)))

        if current is goal:
            return _build_path(goal), closed, frames

        for nb in grid.get_neighbours(current):
            if nb in closed:
                continue
            if nb not in open_set:
                nb.h      = heuristic_fn(nb, goal)
                nb.f      = nb.h          # GBFS: only heuristic
                nb.g      = current.g + 1
                nb.parent = current
                heapq.heappush(open_heap, (nb.f, nb))
                open_set.add(nb)

    return [], closed, frames


def _build_path(goal_node):
    """Trace parent pointers back to start and return start -> goal list."""
    path = []
    cur  = goal_node
    while cur is not None:
        path.append(cur)
        cur = cur.parent
    path.reverse()
    return path


# ─────────────────────────────────────────────────────────────────────
#   BUTTON HELPER
# ─────────────────────────────────────────────────────────────────────

class Button:
    """Simple rectangular button with optional toggle and danger styles."""

    def __init__(self, x, y, w, h, label, toggle=False, danger=False):
        self.rect   = pygame.Rect(x, y, w, h)
        self.label  = label
        self.toggle = toggle
        self.danger = danger
        self.active = False

    def draw(self, surface, font):
        hovering = self.rect.collidepoint(pygame.mouse.get_pos())

        if self.danger:
            bg = (160, 40, 40) if hovering else BTN_DANGER
        elif self.toggle and self.active:
            bg = BTN_ON
        elif hovering:
            bg = BTN_HOVER
        else:
            bg = BTN_NORMAL

        pygame.draw.rect(surface, bg, self.rect, border_radius=6)
        border_col = ACCENT_COLOUR if (self.toggle and self.active) else DIVIDER
        pygame.draw.rect(surface, border_col, self.rect, 1, border_radius=6)

        text_surf = font.render(self.label, True, TEXT_COLOUR)
        surface.blit(text_surf, text_surf.get_rect(center=self.rect.center))

    def handle_event(self, event):
        """Returns True if this button was clicked in the given event."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                if self.toggle:
                    self.active = not self.active
                return True
        return False


# ─────────────────────────────────────────────────────────────────────
#   SETUP SCREEN – runs before main window to collect grid params
# ─────────────────────────────────────────────────────────────────────

def run_setup_screen():
    """
    Simple setup dialog: user enters rows, cols, obstacle density.
    Returns (rows, cols, density).
    Calls sys.exit() if the window is closed.
    """
    pygame.init()
    screen = pygame.display.set_mode((480, 340))
    pygame.display.set_caption("Pathfinding Agent – Setup")
    clock = pygame.time.Clock()

    font_big  = pygame.font.SysFont("Segoe UI", 22, bold=True)
    font_med  = pygame.font.SysFont("Segoe UI", 16)
    font_hint = pygame.font.SysFont("Segoe UI", 13)

    fields = {
        "rows":    {"label": "Rows (5 - 50):",      "value": "20",
                    "rect": pygame.Rect(230, 110, 120, 34)},
        "cols":    {"label": "Columns (5 - 80):",   "value": "30",
                    "rect": pygame.Rect(230, 160, 120, 34)},
        "density": {"label": "Obstacle % (0 - 70):", "value": "30",
                    "rect": pygame.Rect(230, 210, 120, 34)},
    }
    active_field = None
    start_btn    = pygame.Rect(160, 270, 160, 40)

    running = True
    while running:
        screen.fill(BG_COLOUR)

        # Title
        title = font_big.render("Dynamic Pathfinding Agent", True, ACCENT_COLOUR)
        screen.blit(title, title.get_rect(centerx=240, y=26))
        sub = font_hint.render("Configure the grid then click START", True, DIM_TEXT)
        screen.blit(sub, sub.get_rect(centerx=240, y=60))

        # Fields
        for key, fd in fields.items():
            lbl  = font_med.render(fd["label"], True, TEXT_COLOUR)
            screen.blit(lbl, (26, fd["rect"].y + 8))
            bg   = BTN_HOVER if active_field == key else BTN_NORMAL
            pygame.draw.rect(screen, bg, fd["rect"], border_radius=5)
            border = ACCENT_COLOUR if active_field == key else DIVIDER
            pygame.draw.rect(screen, border, fd["rect"], 1, border_radius=5)
            val = font_med.render(fd["value"], True, TEXT_COLOUR)
            screen.blit(val, (fd["rect"].x + 8, fd["rect"].y + 8))

        # START button
        hover = start_btn.collidepoint(pygame.mouse.get_pos())
        pygame.draw.rect(screen, BTN_ON if hover else BTN_NORMAL, start_btn, border_radius=8)
        pygame.draw.rect(screen, ACCENT_COLOUR, start_btn, 1, border_radius=8)
        sl = font_big.render("START", True, TEXT_COLOUR)
        screen.blit(sl, sl.get_rect(center=start_btn.center))

        pygame.display.flip()
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                active_field = None
                for key, fd in fields.items():
                    if fd["rect"].collidepoint(event.pos):
                        active_field = key
                if start_btn.collidepoint(event.pos):
                    running = False

            if event.type == pygame.KEYDOWN and active_field is not None:
                fd = fields[active_field]
                if event.key == pygame.K_BACKSPACE:
                    fd["value"] = fd["value"][:-1]
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    running = False
                elif event.key == pygame.K_TAB:
                    keys = list(fields.keys())
                    idx  = keys.index(active_field)
                    active_field = keys[(idx + 1) % len(keys)]
                elif event.unicode in "0123456789" and len(fd["value"]) < 3:
                    fd["value"] += event.unicode

    # Parse values safely
    try:
        rows = max(5, min(50, int(fields["rows"]["value"])))
    except ValueError:
        rows = 20
    try:
        cols = max(5, min(80, int(fields["cols"]["value"])))
    except ValueError:
        cols = 30
    try:
        density = max(0.0, min(0.70, int(fields["density"]["value"]) / 100.0))
    except ValueError:
        density = 0.30

    return rows, cols, density


# ─────────────────────────────────────────────────────────────────────
#   MAIN APPLICATION CLASS
# ─────────────────────────────────────────────────────────────────────

class App:
    """
    Main application – owns the pygame window and the game loop.

    Responsibilities:
      - Render grid and panel every frame
      - Handle all pygame events (including QUIT)
      - Drive search animation frame by frame
      - Manage agent movement and dynamic replanning
      - Display live metrics
    """

    def __init__(self, rows, cols, density):
        # pygame was already initialised in the setup screen
        pygame.display.set_caption("Dynamic Pathfinding Agent")
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        self.clock  = pygame.time.Clock()

        self.font_title  = pygame.font.SysFont("Segoe UI", 18, bold=True)
        self.font_body   = pygame.font.SysFont("Segoe UI", 14)
        self.font_small  = pygame.font.SysFont("Segoe UI", 12)
        self.font_metric = pygame.font.SysFont("Segoe UI", 13)

        # Grid
        self.rows    = rows
        self.cols    = cols
        self.density = density
        self.grid    = Grid(rows, cols)
        self.grid.generate_random_map(density)

        # Algorithm / heuristic
        self.algorithm = "A*"
        self.heuristic = "Manhattan"
        self.dyn_mode  = False

        # Search results
        self.path      = []
        self.visited   = set()
        self.frames    = []
        self.frame_idx = 0

        # Agent walking
        self.agent_idx        = 0
        self.last_agent_tick  = 0
        self.agent_move_delay = 120   # ms between agent steps

        # Animation timing
        self.last_frame_tick = 0
        self.frame_delay     = 25    # ms between search frames

        # State machine: "idle" | "animating" | "walking" | "done" | "no_path"
        self.state = "idle"

        # Metrics
        self.nodes_visited = 0
        self.path_cost     = 0
        self.exec_time_ms  = 0.0

        # Dynamic mode
        self.dyn_prob = 0.04   # 4 % chance per agent step

        self._build_panel_buttons()

    # ── Panel layout ─────────────────────────────────────────────────

    def _build_panel_buttons(self):
        px  = GRID_W + 14
        bw  = PANEL_W - 28
        bh  = 32
        gap = 8

        def btn(label, y, toggle=False, danger=False):
            return Button(px, y, bw, bh, label, toggle=toggle, danger=danger)

        y = 52

        self.btn_gen    = btn("Generate Map",        y);              y += bh + gap
        self.btn_clear  = btn("Clear Walls",         y, danger=True); y += bh + gap * 2

        self.btn_astar  = btn("Algorithm: A*",       y, toggle=True); y += bh + gap
        self.btn_gbfs   = btn("Algorithm: GBFS",     y, toggle=True); y += bh + gap * 2
        self.btn_astar.active = True

        self.btn_manh   = btn("Heuristic: Manhattan", y, toggle=True); y += bh + gap
        self.btn_eucl   = btn("Heuristic: Euclidean", y, toggle=True); y += bh + gap * 2
        self.btn_manh.active = True

        self.btn_dyn    = btn("Dynamic Mode: OFF",   y, toggle=True); y += bh + gap * 2
        self.btn_start  = btn("Play  Search",        y);              y += bh + gap
        self.btn_reset  = btn("Reset",               y, danger=True)

    # ── Coordinate helpers ───────────────────────────────────────────

    def _cell_size(self):
        return min(GRID_W // self.cols, GRID_H // self.rows)

    def _cell_rect(self, row, col):
        s = self._cell_size()
        return pygame.Rect(col * s, row * s, s - 1, s - 1)

    def _mouse_to_cell(self, mx, my):
        if mx >= GRID_W:
            return None
        s   = self._cell_size()
        col = mx // s
        row = my // s
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return (row, col)
        return None

    # ── Animation snapshot helpers ───────────────────────────────────

    def _current_frontier(self):
        if 0 < self.frame_idx <= len(self.frames):
            return self.frames[self.frame_idx - 1][0]
        return frozenset()

    def _current_visited_snap(self):
        if 0 < self.frame_idx <= len(self.frames):
            return self.frames[self.frame_idx - 1][1]
        return frozenset()

    # ── Drawing ──────────────────────────────────────────────────────

    def draw_grid(self):
        s          = self._cell_size()
        start_node = self.grid.start_node()
        goal_node  = self.grid.goal_node()
        frontier   = self._current_frontier()
        vis_snap   = self._current_visited_snap()

        for r in range(self.rows):
            for c in range(self.cols):
                node = self.grid.nodes[r][c]
                rect = self._cell_rect(r, c)

                if node is start_node:
                    colour = START_COLOUR
                elif node is goal_node:
                    colour = GOAL_COLOUR
                elif node.is_wall:
                    colour = WALL_COLOUR
                elif node in self.path and self.state in ("walking", "done"):
                    colour = PATH_COLOUR
                elif node in vis_snap:
                    colour = VISITED_COLOUR
                elif node in frontier:
                    colour = FRONTIER_COLOUR
                else:
                    colour = EMPTY_CELL

                pygame.draw.rect(self.screen, colour, rect)

        # Draw agent circle
        if self.state in ("walking", "done") and self.path:
            a    = self.path[self.agent_idx]
            rect = self._cell_rect(a.row, a.col)
            cx, cy = rect.center
            r = max(4, s // 3)
            pygame.draw.circle(self.screen, AGENT_COLOUR, (cx, cy), r)
            pygame.draw.circle(self.screen, START_COLOUR, (cx, cy), r, 2)

    def draw_panel(self):
        pygame.draw.rect(self.screen, PANEL_BG, pygame.Rect(GRID_W, 0, PANEL_W, SCREEN_H))
        pygame.draw.line(self.screen, ACCENT_COLOUR, (GRID_W, 0), (GRID_W, SCREEN_H), 2)

        # Title
        t = self.font_title.render("Pathfinding Agent", True, ACCENT_COLOUR)
        self.screen.blit(t, (GRID_W + 14, 14))

        for btn in self._all_buttons():
            btn.draw(self.screen, self.font_body)

        # Divider before metrics
        div_y = self.btn_reset.rect.bottom + 16
        pygame.draw.line(self.screen, DIVIDER,
                         (GRID_W + 14, div_y), (SCREEN_W - 14, div_y))

        my = div_y + 10
        hdr = self.font_title.render("Metrics", True, ACCENT_COLOUR)
        self.screen.blit(hdr, (GRID_W + 14, my))
        my += 26

        rows = [
            ("Algorithm",     self.algorithm),
            ("Heuristic",     self.heuristic),
            ("Dynamic Mode",  "ON" if self.dyn_mode else "OFF"),
            ("Grid Size",     f"{self.rows} x {self.cols}"),
            ("",              ""),
            ("Nodes Visited", str(self.nodes_visited)),
            ("Path Cost",     str(self.path_cost)),
            ("Exec Time",     f"{self.exec_time_ms:.1f} ms"),
            ("",              ""),
            ("Status",        self.state.upper()),
        ]

        for label, value in rows:
            if label == "":
                my += 6
                continue
            ls = self.font_metric.render(label + ":", True, DIM_TEXT)
            vs = self.font_metric.render(value,       True, TEXT_COLOUR)
            self.screen.blit(ls, (GRID_W + 14, my))
            self.screen.blit(vs, (GRID_W + 145, my))
            my += 19

        # Legend
        my += 8
        pygame.draw.line(self.screen, DIVIDER,
                         (GRID_W + 14, my), (SCREEN_W - 14, my))
        my += 10
        self.screen.blit(self.font_title.render("Legend", True, ACCENT_COLOUR),
                         (GRID_W + 14, my))
        my += 24

        legend = [
            (START_COLOUR,    "Start"),
            (GOAL_COLOUR,     "Goal"),
            (WALL_COLOUR,     "Obstacle"),
            (FRONTIER_COLOUR, "Frontier"),
            (VISITED_COLOUR,  "Visited"),
            (PATH_COLOUR,     "Path"),
        ]
        for col, lbl in legend:
            pygame.draw.rect(self.screen, col,
                             (GRID_W + 14, my + 3, 14, 14), border_radius=2)
            self.screen.blit(
                self.font_small.render(lbl, True, TEXT_COLOUR),
                (GRID_W + 34, my + 2)
            )
            my += 20

        # Hint text at bottom
        hints = ["L-Click : place wall", "R-Click : remove wall"]
        hy = SCREEN_H - 14 - len(hints) * 17
        for h in hints:
            self.screen.blit(
                self.font_small.render(h, True, DIM_TEXT),
                (GRID_W + 14, hy)
            )
            hy += 17

    def _all_buttons(self):
        return [
            self.btn_gen,  self.btn_clear,
            self.btn_astar, self.btn_gbfs,
            self.btn_manh,  self.btn_eucl,
            self.btn_dyn,
            self.btn_start, self.btn_reset,
        ]

    # ── Event handling ────────────────────────────────────────────────

    def handle_events(self):
        mouse_btns = pygame.mouse.get_pressed()

        for event in pygame.event.get():
            # Always handle QUIT first – this is critical for responsiveness
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                if event.key == pygame.K_r:
                    self._do_reset()
                if event.key == pygame.K_RETURN and self.state == "idle":
                    self._do_start_search()

            if self.btn_gen.handle_event(event):
                self._do_reset()
                self.grid.generate_random_map(self.density)

            elif self.btn_clear.handle_event(event):
                self._do_reset()
                self.grid.clear_walls()

            elif self.btn_astar.handle_event(event):
                self.algorithm = "A*"
                self.btn_astar.active = True
                self.btn_gbfs.active  = False

            elif self.btn_gbfs.handle_event(event):
                self.algorithm = "GBFS"
                self.btn_gbfs.active  = True
                self.btn_astar.active = False

            elif self.btn_manh.handle_event(event):
                self.heuristic = "Manhattan"
                self.btn_manh.active = True
                self.btn_eucl.active = False

            elif self.btn_eucl.handle_event(event):
                self.heuristic = "Euclidean"
                self.btn_eucl.active = True
                self.btn_manh.active = False

            elif self.btn_dyn.handle_event(event):
                self.dyn_mode = self.btn_dyn.active
                self.btn_dyn.label = (
                    "Dynamic Mode: ON" if self.dyn_mode else "Dynamic Mode: OFF"
                )

            elif self.btn_start.handle_event(event):
                self._do_start_search()

            elif self.btn_reset.handle_event(event):
                self._do_reset()

        # Wall drawing via mouse drag (only in idle state)
        if self.state == "idle":
            mx, my = pygame.mouse.get_pos()
            cell   = self._mouse_to_cell(mx, my)
            if cell:
                row, col = cell
                n = self.grid.node(row, col)
                if n:
                    is_start = (row, col) == self.grid.start_pos
                    is_goal  = (row, col) == self.grid.goal_pos
                    if not is_start and not is_goal:
                        if mouse_btns[0]:
                            n.is_wall = True
                        elif mouse_btns[2]:
                            n.is_wall = False

    # ── Search control ────────────────────────────────────────────────

    def _do_start_search(self):
        """Run selected algorithm and start animation."""
        if self.state != "idle":
            return

        self._do_reset(keep_grid=True)

        h_fn = get_heuristic_fn(self.heuristic)

        t0 = time.perf_counter()
        if self.algorithm == "A*":
            self.path, self.visited, self.frames = astar(self.grid, h_fn)
        else:
            self.path, self.visited, self.frames = greedy_bfs(self.grid, h_fn)
        t1 = time.perf_counter()

        self.exec_time_ms  = (t1 - t0) * 1000
        self.nodes_visited = len(self.visited)
        self.path_cost     = max(0, len(self.path) - 1)

        self.frame_idx       = 0
        self.agent_idx       = 0
        self.last_frame_tick = pygame.time.get_ticks()
        self.last_agent_tick = pygame.time.get_ticks()

        if self.frames or self.path:
            self.state = "animating"
        else:
            self.state = "no_path"

    def _do_reset(self, keep_grid=False):
        """Return to idle state and clear all search data."""
        self.state         = "idle"
        self.path          = []
        self.visited       = set()
        self.frames        = []
        self.frame_idx     = 0
        self.agent_idx     = 0
        self.nodes_visited = 0
        self.path_cost     = 0
        self.exec_time_ms  = 0.0

    def _replan(self):
        """
        Re-run the algorithm from the agent's current position.
        Called when a new obstacle blocks the planned route.
        """
        agent_node = self.path[self.agent_idx]

        # Temporarily move start to current agent cell
        original_start      = self.grid.start_pos
        self.grid.start_pos = (agent_node.row, agent_node.col)
        self.grid.reset_search_costs()

        h_fn = get_heuristic_fn(self.heuristic)

        t0 = time.perf_counter()
        if self.algorithm == "A*":
            new_path, new_vis, new_frames = astar(self.grid, h_fn)
        else:
            new_path, new_vis, new_frames = greedy_bfs(self.grid, h_fn)
        t1 = time.perf_counter()

        self.grid.start_pos = original_start  # restore

        if new_path:
            self.path          = new_path
            self.visited       = new_vis
            self.frames        = new_frames
            self.agent_idx     = 0
            self.exec_time_ms  = (t1 - t0) * 1000
            self.nodes_visited = len(new_vis)
            self.path_cost     = len(new_path) - 1
            self.state         = "walking"
        else:
            self.state = "no_path"

    def _path_blocked(self):
        """Return True if any remaining path node is now a wall."""
        for i in range(self.agent_idx, len(self.path)):
            if self.path[i].is_wall:
                return True
        return False

    # ── Update ────────────────────────────────────────────────────────

    def update(self):
        now = pygame.time.get_ticks()

        if self.state == "animating":
            # Advance one search-frame at a time for smooth animation
            if now - self.last_frame_tick >= self.frame_delay:
                self.last_frame_tick = now
                if self.frame_idx < len(self.frames):
                    self.frame_idx += 1
                else:
                    # Search animation finished; start walking
                    self.state = "walking" if self.path else "no_path"

        elif self.state == "walking":
            if now - self.last_agent_tick >= self.agent_move_delay:
                self.last_agent_tick = now

                # Dynamic mode: maybe add an obstacle
                if self.dyn_mode:
                    if random.random() < self.dyn_prob:
                        agent_pos = (self.path[self.agent_idx].row,
                                     self.path[self.agent_idx].col)
                        self.grid.spawn_random_obstacle(agent_pos)

                        if self._path_blocked():
                            self._replan()
                            return  # skip normal step this tick

                # Advance agent
                if self.agent_idx < len(self.path) - 1:
                    self.agent_idx += 1
                else:
                    self.state = "done"

    # ── Main loop ─────────────────────────────────────────────────────

    def run(self):
        """
        Main game loop – runs at FPS, never blocks.
        QUIT event is always handled to ensure clean window closure.
        """
        while True:
            self.handle_events()  # process input (including QUIT)
            self.update()         # advance simulation

            self.screen.fill(BG_COLOUR)
            self.draw_grid()
            self.draw_panel()

            pygame.display.flip()
            self.clock.tick(FPS)


# ─────────────────────────────────────────────────────────────────────
#   ENTRY POINT
# ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    rows, cols, density = run_setup_screen()   # Step 1: collect settings
    app = App(rows, cols, density)              # Step 2: launch main app
    app.run()
