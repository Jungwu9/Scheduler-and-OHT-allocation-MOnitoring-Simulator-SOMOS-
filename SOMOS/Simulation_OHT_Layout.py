from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import pandas as pd
import salabim as sim


@dataclass
class NodeState:
    occupied_by: Optional[int] = None
    queue_len: int = 0
    is_hotspot: bool = False
    hotspot_rank: int = 0
    gate: Optional[sim.Resource] = None


@dataclass
class Node:
    name: str
    x: float
    y: float
    kind: str = "station"
    cell_value: str = ""
    rc: Tuple[int, int] = (-1, -1)
    state: NodeState = field(default_factory=NodeState)
    res: Optional[sim.Resource] = None
    has_machine: bool = False
    machine_no: Optional[int] = None
    machine_dir: Optional[int] = None


@dataclass
class EdgeState:
    moving_count: int = 0
    waiting_count: int = 0


@dataclass
class Edge:
    eid: str
    u: str
    v: str
    kind: str
    length: float
    travel_time: float
    state: EdgeState = field(default_factory=EdgeState)
    res: Optional[sim.Resource] = None


class OHTLayoutBuilder:
    """
    Vertex-based tile-puzzle layout builder.

    Every tile (1~12) has the same structure:
        V_start  ->  V_end   (one direct edge, no intermediate node)

    The vertex node itself is a station.
    Adjacent cells sharing the same vertex are connected automatically.

    Vertex names:
        NW = top-left,     NE = top-right
        SW = bottom-left,  SE = bottom-right
    """

    STRAIGHT_CODES   = {"1", "2", "3", "4"}
    ARC_CODES        = {"5", "6", "7", "8", "9", "10", "11", "12"}
    JUNCTION_CODES   = {"13", "14", "15", "16", "17", "18", "19", "20"}
    RAIL_CODES       = STRAIGHT_CODES | ARC_CODES | JUNCTION_CODES
    BARRIER_CODES    = {"#", "##", "###"}
    BARRIER_SIZE_MAP = {"#": 1.10, "##": 0.68, "###": 0.05}

    # ------------------------------------------------------------------
    # TILE_CORNERS : (start_corner, end_corner)
    #   arrow tail = start,  arrow head = end
    # ------------------------------------------------------------------
    TILE_CORNERS = {
        # straight
        "1":  ("NW", "NE"),   # top edge ->
        "2":  ("SW", "NW"),   # left edge ^
        "3":  ("NE", "SE"),   # right edge v
        "4":  ("SE", "SW"),   # bottom edge <-
        # diagonal, upward arc (5~8)
        "5":  ("NW", "SE"),   # top-left -> bottom-right, convex up
        "6":  ("SE", "NW"),   # bottom-right -> top-left, convex up
        "7":  ("NE", "SW"),   # top-right -> bottom-left, convex up
        "8":  ("SW", "NE"),   # bottom-left -> top-right, convex up
        # diagonal, downward arc (9~12)
        "9":  ("NW", "SE"),   # top-left -> bottom-right, convex down
        "10": ("SE", "NW"),   # bottom-right -> top-left, convex down
        "11": ("SW", "NE"),  # bottom-left -> top-right, convex down
        "12": ("NE", "SW"),   # top-right -> bottom-left, convex down
    }

    # corner -> (delta_vertex_row, delta_vertex_col)
    # vertex of cell (r, c) = V(r + dvr, c + dvc)
    CORNER_DV = {
        "NW": (0, 0),
        "NE": (0, 1),
        "SW": (1, 0),
        "SE": (1, 1),
    }

    # ------------------------------------------------------------------
    # Arc tile quadratic Bezier control point (CP)
    #   convex up (5~8) : NW-SE diagonal -> NE,  NE-SW diagonal -> NW
    #   convex down (9~12): NW-SE diagonal -> SW,  NE-SW diagonal -> SE
    # ------------------------------------------------------------------
    ARC_CP_CORNER = {
        "5":  "NE",  "6":  "NE",   # NW-SE diagonal, convex up
        "7":  "NW",  "8":  "NW",   # NE-SW diagonal, convex up
        "9":  "SW",  "10": "SW",   # NW-SE diagonal, convex down
        "11": "SE",  "12": "SE",   # NE-SW diagonal, convex down
    }

    ARC_SAMPLES = 8   # number of curve intermediate points

    # ------------------------------------------------------------------
    # Junction tile definition: (shared_corner, straight_other, arc_other, arc_cp, mode)
    #   odd (13,15,17,19) = FORK  : exits from shared into two paths
    #   even (14,16,18,20) = MERGE : two paths merge into shared
    #
    #   Row 1 — top straight + diagonal arc (upper bow):
    #     13 FORK  NW : NW→NE(top→) + NW→SE(arc upper CP=NE)  = Tile1+Tile5
    #     14 MERGE NW : NE→NW(top←) + SE→NW(arc upper CP=NE)  = reverse of 13
    #     15 FORK  NE : NE→NW(top←) + NE→SW(arc upper CP=NW)  = Tile7+(NE→NW)
    #     16 MERGE NE : NW→NE(top→) + SW→NE(arc upper CP=NW)  = reverse of 15
    #
    #   Row 2 — bottom straight (bot) + diagonal arc:
    #     17 FORK  SW : SW→SE(bot→) + SW→NE(arc upper CP=NW)  = Tile4rev+Tile8
    #     18 MERGE SW : SE→SW(bot←) + NE→SW(arc upper CP=NW)  = reverse of 17
    #     19 FORK  SW : SW→SE(bot→) + SW→NE(arc lower CP=SE)  = Tile4rev+Tile12
    #     20 MERGE SW : SE→SW(bot←) + NE→SW(arc lower CP=SE)  = reverse of 19
    # ------------------------------------------------------------------
    TILE_JUNCTION_DEF = {
        "13": ("NW", "NE", "SE", "NE", "fork"),
        "14": ("NW", "NE", "SE", "NE", "merge"),
        "15": ("NE", "NW", "SW", "NW", "fork"),
        "16": ("NE", "NW", "SW", "NW", "merge"),
        "17": ("SE", "SW", "NW", "SW", "merge"),
        "18": ("SE", "SW", "NW", "SW", "fork"),
        "19": ("SW", "SE", "NE", "SE", "fork"),
        "20": ("SW", "SE", "NE", "SE", "merge"),
    }

    # ------------------------------------------------------------------

    def __init__(self, csv_path: str, oht_config):
        self.csv_path   = csv_path
        self.oht_config = oht_config

        self.nodes:     Dict[str, Node] = {}
        self.edges:     Dict[str, Edge] = {}
        self.adj:       Dict[str, List[str]] = {}
        self.uv_to_eid: Dict[Tuple[str, str], str] = {}

        self.grid = None
        self.rows = 0
        self.cols = 0

        self.main_node_name:      Optional[str] = None
        self.station_nodes:       List[str] = []
        self.rail_nodes:          List[str] = []
        self.control_entry_nodes: List[str] = []
        self.barrier_cells:       List[Tuple[int, int, str, float]] = []

        self.col_w:   List[float] = []
        self.row_h:   List[float] = []
        self.x_edges: List[float] = []
        self.y_edges: List[float] = []

        self._edge_idx    = 1
        self._guide_idx   = 0   # arc guide node counter
        # vertex cache: (vr, vc) -> node_name
        self._vertex_cache: Dict[Tuple[int, int], str] = {}

    # ==================================================================
    # Public
    # ==================================================================

    def build(self):
        self._load_csv()
        self._build_variable_axes()
        self._build_tiles()
        self._build_adjacency_from_edges()
        self._mark_hotspots()
        self._attach_resources()
        self._build_control_entry_nodes()

    # ==================================================================
    # CSV
    # ==================================================================

    def _load_csv(self):
        df = pd.read_csv(self.csv_path, header=None, keep_default_na=False)
        self.grid = df.values
        self.rows, self.cols = self.grid.shape

    def _cell_value(self, r: int, c: int) -> str:
        v = self.grid[r][c]
        return "" if pd.isna(v) else str(v).strip()

    def _cell_base_code(self, r: int, c: int) -> Optional[str]:
        s = self._cell_value(r, c)
        if not s or s in self.BARRIER_CODES:
            return None
        m = re.match(r"^\s*(\d+)", s)
        return m.group(1) if m else None

    # ==================================================================
    # variable axes
    # ==================================================================

    def _token_span(self, tok: str) -> float:
        cs = float(self.oht_config.cell_size)
        if tok in self.BARRIER_CODES:
            return self.BARRIER_SIZE_MAP[tok] * cs
        return 0.22 * cs if tok == "" else cs

    def _build_variable_axes(self):
        def _max_span(vals):
            spans = []
            for v in vals:
                if v in self.BARRIER_CODES:
                    spans.append(self._token_span(v))
                else:
                    m = re.search(r"(#+)\s*$", v)
                    b = m.group(1) if m else ""
                    if b in self.BARRIER_CODES:
                        spans.append(self._token_span(b))
                    else:
                        spans.append(self._token_span("") if v == ""
                                     else float(self.oht_config.cell_size))
            return max(spans) if spans else float(self.oht_config.cell_size)

        self.col_w = [
            _max_span([self._cell_value(r, c) for r in range(self.rows)])
            for c in range(self.cols)
        ]
        self.row_h = [
            _max_span([self._cell_value(r, c) for c in range(self.cols)])
            for r in range(self.rows)
        ]

        self.x_edges = [0.0]
        for w in self.col_w:
            self.x_edges.append(self.x_edges[-1] + w)

        self.y_edges = [0.0]
        for h in self.row_h:
            self.y_edges.append(self.y_edges[-1] + h)

    # ==================================================================
    # vertex node
    # ==================================================================

    def _get_or_create_vertex(self, vr: int, vc: int) -> str:
        """
        Return the station node for grid vertex V(vr, vc) (create if absent).
        The same (vr, vc) is always the same node -> automatic shared connection.
        """
        key = (vr, vc)
        if key in self._vertex_cache:
            return self._vertex_cache[key]

        name = f"V_{vr}_{vc}"
        x    = self.x_edges[vc]
        y    = -self.y_edges[vr]
        node = Node(name=name, x=x, y=y, kind="station",
                    rc=(vr - 1, vc - 1))
        self.nodes[name] = node
        self.station_nodes.append(name)
        self.rail_nodes.append(name)
        self._vertex_cache[key] = name
        return name

    def _corner_vertex(self, r: int, c: int, corner: str) -> str:
        """Node name of cell (r, c)'s corner vertex."""
        dvr, dvc = self.CORNER_DV[corner]
        return self._get_or_create_vertex(r + dvr, c + dvc)

    # ==================================================================
    # graph helpers
    # ==================================================================

    def _add_edge(self, u: str, v: str, kind: str):
        if (u, v) in self.uv_to_eid:
            return
        a, b = self.nodes[u], self.nodes[v]
        dist = math.hypot(b.x - a.x, b.y - a.y)

        if kind == "curve":
            time_per_unit = float(getattr(self.oht_config, "curve_time_per_unit", self.oht_config.base_time_per_unit))
        else:
            time_per_unit = float(getattr(self.oht_config, "base_time_per_unit", 1.0))

        tt   = max(0.05, dist * time_per_unit)
        eid  = f"E{self._edge_idx:05d}"
        self._edge_idx += 1
        self.edges[eid] = Edge(eid=eid, u=u, v=v, kind=kind,
                               length=dist, travel_time=tt)
        self.uv_to_eid[(u, v)] = eid

    # ==================================================================
    # arc curve interpolation
    # ==================================================================

    def _corner_xy(self, r: int, c: int, corner: str):
        dvr, dvc = self.CORNER_DV[corner]
        return self.x_edges[c + dvc], -self.y_edges[r + dvr]

    def _quad_bezier_pts(self, p0, cp, p2, n):
        pts = []
        for i in range(1, n):
            t = i / n; mt = 1.0 - t
            x = mt*mt*p0[0] + 2*mt*t*cp[0] + t*t*p2[0]
            y = mt*mt*p0[1] + 2*mt*t*cp[1] + t*t*p2[1]
            pts.append((x, y))
        return pts

    # ==================================================================
    # tile build
    # ==================================================================

    def _add_arc_path(self, v_from, v_to, p0, cp, p2, tok, r, c):
        """Connect vertex v_from -> v_to with a Bezier curve (inserting guide nodes)."""
        n    = getattr(self.oht_config, "arc_samples", self.ARC_SAMPLES)
        path = [v_from]
        for px, py in self._quad_bezier_pts(p0, cp, p2, n):
            self._guide_idx += 1
            gname = f"ARC_{self._guide_idx:06d}"
            self.nodes[gname] = Node(name=gname, x=px, y=py, kind="guide",
                                     cell_value=tok, rc=(r, c))
            path.append(gname)
        path.append(v_to)
        for a, b in zip(path[:-1], path[1:]):
            self._add_edge(a, b, kind="curve")

    def _build_tile(self, r: int, c: int, tok: str):
        # ── Junction tile (13~20): FORK or MERGE ────────────────────────
        if tok in self.JUNCTION_CODES:
            shared_c, str_other_c, arc_other_c, arc_cp_c, mode = self.TILE_JUNCTION_DEF[tok]
            v_shared    = self._corner_vertex(r, c, shared_c)
            v_str_other = self._corner_vertex(r, c, str_other_c)
            v_arc_other = self._corner_vertex(r, c, arc_other_c)
            cp_xy       = self._corner_xy(r, c, arc_cp_c)

            if mode == "fork":
                # shared → str_other (straight)
                self._add_edge(v_shared, v_str_other, kind="straight")
                # shared → arc_other (arc)
                p0 = (self.nodes[v_shared   ].x, self.nodes[v_shared   ].y)
                p2 = (self.nodes[v_arc_other].x, self.nodes[v_arc_other].y)
                self._add_arc_path(v_shared, v_arc_other, p0, cp_xy, p2, tok, r, c)
            else:  # merge
                # str_other → shared (straight)
                self._add_edge(v_str_other, v_shared, kind="straight")
                # arc_other → shared (arc)
                p0 = (self.nodes[v_arc_other].x, self.nodes[v_arc_other].y)
                p2 = (self.nodes[v_shared   ].x, self.nodes[v_shared   ].y)
                self._add_arc_path(v_arc_other, v_shared, p0, cp_xy, p2, tok, r, c)
            return

        # ── straight/arc tile (1~12) ─────────────────────────────────────────
        if tok not in self.TILE_CORNERS:
            return
        start_c, end_c = self.TILE_CORNERS[tok]
        v_start = self._corner_vertex(r, c, start_c)
        v_end   = self._corner_vertex(r, c, end_c)

        if tok in self.STRAIGHT_CODES:
            # straight: one direct edge between vertices
            self._add_edge(v_start, v_end, kind="straight")
        else:
            # arc: insert Bezier curve guide nodes
            p0    = (self.nodes[v_start].x, self.nodes[v_start].y)
            p2    = (self.nodes[v_end  ].x, self.nodes[v_end  ].y)
            cp_xy = self._corner_xy(r, c, self.ARC_CP_CORNER[tok])
            self._add_arc_path(v_start, v_end, p0, cp_xy, p2, tok, r, c)

    def _build_tiles(self):
        for r in range(self.rows):
            for c in range(self.cols):
                raw  = self._cell_value(r, c)
                base = self._cell_base_code(r, c)
                if raw in self.BARRIER_CODES:
                    self.barrier_cells.append(
                        (r, c, raw, float(self.BARRIER_SIZE_MAP[raw]))
                    )
                    continue
                if base in self.RAIL_CODES:
                    self._build_tile(r, c, base)

    # ==================================================================
    # adjacency / hotspot / resource
    # ==================================================================

    def _build_adjacency_from_edges(self):
        self.adj = {name: [] for name in self.nodes}
        for eid in sorted(self.edges):
            e = self.edges[eid]
            self.adj[e.u].append(e.v)

    def _mark_hotspots(self):
        indeg  = {nm: 0 for nm in self.nodes}
        outdeg = {nm: 0 for nm in self.nodes}
        for e in self.edges.values():
            outdeg[e.u] += 1
            indeg[e.v]  += 1
        for nm, node in self.nodes.items():
            if node.kind != "station":
                node.state.is_hotspot  = False
                node.state.hotspot_rank = 0
                continue
            deg = indeg.get(nm, 0) + outdeg.get(nm, 0)
            rank = 0
            if indeg.get(nm, 0) >= 2 or outdeg.get(nm, 0) >= 2:
                rank = 1
            if deg >= 4:
                rank = 2
            node.state.is_hotspot  = rank > 0
            node.state.hotspot_rank = rank

    def _attach_resources(self):
        for node in self.nodes.values():
            node.res = sim.Resource(name=f"node_{node.name}", capacity=1)
            if node.state.is_hotspot:
                node.state.gate = sim.Resource(
                    name=f"gate_{node.name}", capacity=1)
        for edge in self.edges.values():
            edge.res = sim.Resource(
                name=f"edge_{edge.eid}",
                capacity=self.oht_config.edge_capacity)

    def _build_control_entry_nodes(self):
        if not self.station_nodes:
            self.control_entry_nodes = []
            return
        min_x = min(self.nodes[nm].x for nm in self.station_nodes)
        cands = [nm for nm in self.station_nodes
                 if abs(self.nodes[nm].x - min_x) < 1e-9]
        cands.sort(key=lambda nm: self.nodes[nm].y, reverse=True)
        self.control_entry_nodes = cands

    def get_entry_node_for_source(self, source_name: str) -> str:
        if source_name in self.station_nodes:
            return source_name
        if self.control_entry_nodes:
            return self.control_entry_nodes[0]
        if self.main_node_name:
            return self.main_node_name
        raise RuntimeError("No entry node available.")

    # ==================================================================
    # layout_Machine.csv overlay
    # ==================================================================

    def load_machine_layout(self, machine_csv_path: str):
        """
        Read layout_Machine.csv and set machine info (has_machine, machine_no, machine_dir)
        on the vertex node at each M_N cell position.
        If the file is missing, just print a warning and skip.
        """
        import os
        if not os.path.exists(machine_csv_path):
            print(f"[Layout] layout_Machine.csv not found -> skip: {machine_csv_path}")
            return

        df    = pd.read_csv(machine_csv_path, header=None, keep_default_na=False)
        count = 0

        for r in range(min(df.shape[0], self.rows)):
            for c in range(min(df.shape[1], self.cols)):
                val = str(df.values[r][c]).strip()
                m   = re.match(r"^M_(\d+)$", val, re.IGNORECASE)
                if not m:
                    continue
                machine_no = int(m.group(1))

                # use the cell's start_corner vertex as the machine position
                base_tok = self._cell_base_code(r, c)
                if base_tok not in self.TILE_CORNERS:
                    continue
                start_c, _ = self.TILE_CORNERS[base_tok]
                node_name  = self._corner_vertex(r, c, start_c)

                node             = self.nodes[node_name]
                node.has_machine = True
                node.machine_no  = machine_no
                node.machine_dir = int(base_tok) if base_tok.isdigit() else 1
                count += 1

        print(f"[Layout] layout_Machine.csv -> set {count} machine positions")

    def get_machine_draw_offset(self, node_name: str) -> tuple:
        """
        Return the machine box offset (dx, dy) per machine_dir.
          1 (W->E): up    +y      4 (E->W): down -y
          2 (S->N): right +x      3 (N->S): left -x
        """
        node = self.nodes.get(node_name)
        if node is None:
            return (0.0, 0.0)
        cs = float(self.oht_config.cell_size)
        d  = getattr(node, "machine_dir", 1) or 1
        return {1: (0.0, cs), 4: (0.0, -cs),
                2: (cs, 0.0), 3: (-cs, 0.0)}.get(d, (0.0, cs))