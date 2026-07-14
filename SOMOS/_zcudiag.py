"""Diagnose ZCU/edge wait-for resource cycles at the moment of collapse.
Usage: python _zcudiag.py <seed> [horizon]
 After freeze, for each vehicle: pos/kind, held ZCU zones, forced-next, the zone required to enter it and its owner → reconstruct the cycle.
"""
import sys, csv
import Simualtion_Main as M

sd = int(sys.argv[1])
horizon = float(sys.argv[2]) if len(sys.argv) > 2 else 3500.0

cfg = M._jssp_cfg_with_output_dir(M.JSSPConfig(), "zcudiag_s%d_dir" % sd)
r = M.SimulationRunner(layout_csv_path='layout_oht.csv',
                       machine_csv_path='layout_machine.csv', jssp_cfg=cfg)
r.oht_config.enable_animation = False
r.oht_config.sim_horizon = horizon
r.oht_config.seed = sd
r.oht_config.n_oht = 120
for a, b in [('load_time_min',25),('load_time_max',25),('unload_time_min',25),('unload_time_max',25)]:
    setattr(r.oht_config, a, b)
r.oht_config.oht_dispatch_mode = "HUNGARIAN"
r.oht_config.oht_blocking_weight = 0; r.oht_config.oht_predispatch_lookahead = 0
r.oht_config.oht_urgency_weight = 0; r.oht_config.oht_congestion_cost_weight = 0
r.oht_config.oht_idle_positioning = False

kpi = r.run(enable_animation=False)

# ── post-freeze wait-for diagnosis ──
node_to_zone = r.zcu_node_to_zone          # node -> set(zones)
zone_owner = r.zcu_occupied_by             # zone -> vid
nodes = r.nodes

def zones_for(nds):
    z = set()
    for nd in nds:
        z |= set(node_to_zone.get(nd, ()))
    return z

def forced_next(v):
    cargo = getattr(v, 'cargo_job', None); task = getattr(v, 'assigned_task', None)
    tgt = None
    if cargo is not None:
        tgt = cargo.next_candidate_nodes[0] if getattr(cargo,'next_candidate_nodes',None) else (task.dest_name if task else None)
    elif task is not None:
        tgt = task.source_name
    if not tgt or tgt == v.pos_node or tgt not in nodes: return None
    p = v.shortest_path(v.pos_node, tgt)
    return p[1] if len(p) > 1 else None

# node occupancy map
occ = {}
for v in r.vehicles:
    occ[v.pos_node] = getattr(v, 'vid', id(v))

rows = []
waitfor = {}   # vid -> set(vid): forced_next node is occupied
n_nxt_free = n_nxt_occ = n_nxt_none = 0
for v in r.vehicles:
    vid = getattr(v, 'vid', id(v))
    pos = v.pos_node; kind = nodes[pos].kind if pos in nodes else '?'
    held = sorted(getattr(v, 'current_zcu_zones', set()))
    nxt = forced_next(v)
    blockers = set()
    if nxt is None:
        n_nxt_none += 1
    elif nxt in occ and occ[nxt] != vid:
        n_nxt_occ += 1
        blockers.add(occ[nxt])       # vehicle occupying forced_next
    else:
        n_nxt_free += 1              # forced_next is free — why can't it move?
    nxt_kind = nodes[nxt].kind if (nxt and nxt in nodes) else '?'
    waitfor[vid] = blockers
    rows.append((vid, pos, kind, nxt, nxt_kind, sorted(blockers)))
print("forced_next: free=%d occupied=%d none=%d" % (n_nxt_free, n_nxt_occ, n_nxt_none))

# cycle detection (simple DFS)
def find_cycle():
    color = {}
    stack = []
    def dfs(u):
        color[u] = 1; stack.append(u)
        for w in waitfor.get(u, ()):
            if color.get(w,0) == 1:
                i = stack.index(w); return stack[i:] + [w]
            if color.get(w,0) == 0:
                c = dfs(w)
                if c: return c
        stack.pop(); color[u] = 2; return None
    for u in waitfor:
        if color.get(u,0) == 0:
            c = dfs(u)
            if c: return c
    return None

cyc = find_cycle()
n_wait = sum(1 for b in waitfor.values() if b)
print("ZCUDIAG seed=%d thr=%s | forced_next-occupied wait=%d/120" % (sd, kpi.get('throughput_24h'), n_wait))
print("pos kind dist:", {k: sum(1 for x in rows if x[2]==k) for k in set(x[2] for x in rows)})
print("forced_next kind dist:", {k: sum(1 for x in rows if x[4]==k) for k in set(x[4] for x in rows)})
print("NODE CYCLE:", cyc if cyc else "none found")
if cyc:
    for vid in cyc[:-1]:
        r_ = next(x for x in rows if x[0]==vid)
        print("  vid%s pos=%s(%s) -> next=%s(%s) blockedBy=%s" % (r_[0], r_[1], r_[2], r_[3], r_[4], r_[5]))
# ── track merge-yield targets of leaders (forced_next FREE but stuck) ──
occ_v = {v.pos_node: v for v in r.vehicles}
def occ_or_intent(nd):
    v = occ_v.get(nd)
    if v: return getattr(v,'vid',id(v))
    for v in r.vehicles:
        if getattr(v,'next_hop_intent',None) == nd: return getattr(v,'vid',id(v))
    return None

myield = {}   # vid -> set(vid) it yields to at merge
leader_merge = {}  # vid -> merge station
leaders = [x for x in rows if x[3] is not None and not x[5] and x[3] not in occ]
print("--- leaders (forced_next FREE but stuck): %d ---" % len(leaders))
for x in leaders:
    vid = x[0]; v = next(vv for vv in r.vehicles if getattr(vv,'vid',id(vv))==vid)
    nxt = x[3]
    ms, chain = v._curve_merge_station(nxt) if hasattr(v,'_curve_merge_station') else (None,[])
    tgts = set()
    if ms is not None:
        leader_merge[vid] = ms
        preds = set(v._incoming_predecessors(ms)) - set(chain) - {v.pos_node}
        for nd in list(preds) + [ms]:
            o = occ_or_intent(nd)
            if o is not None and o != vid: tgts.add(o)
    myield[vid] = tgts
    print("  vid%s pos=%s -> curve %s -> merge=%s yieldsTo=%s" % (vid, x[1], nxt, ms, sorted(tgts)))

# merge-yield cycle detection
def find_cycle2(graph):
    color={}; stack=[]
    def dfs(u):
        color[u]=1; stack.append(u)
        for w in graph.get(u,()):
            if color.get(w,0)==1:
                return stack[stack.index(w):]+[w]
            if color.get(w,0)==0:
                c=dfs(w)
                if c: return c
        stack.pop(); color[u]=2; return None
    for u in graph:
        if color.get(u,0)==0:
            c=dfs(u)
            if c: return c
    return None
# ── combined wait-for: queue (occupancy) + leaders (merge-yield) ──
combined = {}
for v in r.vehicles:
    vid = getattr(v,'vid',id(v))
    if vid in myield:                      # leader
        combined[vid] = set(myield[vid])
    else:
        nxt = forced_next(v)
        combined[vid] = {occ[nxt]} if (nxt in occ and occ[nxt]!=vid) else set()
cc = find_cycle2(combined)
print("COMBINED WAIT-FOR CYCLE:", cc if cc else "none")
if cc:
    print("  cycle length:", len(cc)-1)
    for vid in cc[:-1]:
        r_ = next(x for x in rows if x[0]==vid)
        role = "LEADER" if vid in myield else "queued"
        print("  vid%s(%s) pos=%s next=%s(%s) -> %s" % (vid, role, r_[1], r_[3], r_[4], sorted(combined[vid])))
    # ── HOTSPOT: merge stations involved in the cycle ──
    cyc_merges = sorted({leader_merge[vid] for vid in cc[:-1] if vid in leader_merge})
    print("HOTSPOT_CYCLE_MERGES seed=%d:" % sd, cyc_merges)
# merges (contention points) of all leaders — for aggregating bay patterns
print("HOTSPOT_ALL_LEADER_MERGES seed=%d:" % sd, sorted(set(leader_merge.values())))
