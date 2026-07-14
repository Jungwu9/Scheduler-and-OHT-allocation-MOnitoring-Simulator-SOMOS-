"""
gen_machine_travel.py — generate the M_i -> M_j machine-to-machine travel-time matrix.

From the rail layout (layout_oht.csv) + machine placement (layout_machine.csv), builds
machine_travel.csv by adding load+unload handling to the free-flow shortest travel time
(Dijkstra) for every machine pair.
Feeding this file via gen_machine.py --machine_travel makes op-to-op transport in the
planned gantt reflect the 'actual M_i->M_j rail shortest time' instead of a 'per-bay approximation'.

Output columns: from_machine, to_machine, with_handling_sec   (format read by gen_machine)

Run: (salabim environment)  python gen_machine_travel.py
"""
import csv
import Simualtion_Main as M
from Simulation_Machine_Config import JSSPConfig


def main():
    cfg = JSSPConfig()
    r = M.SimulationRunner(
        layout_csv_path='layout_oht.csv',
        machine_csv_path='layout_machine.csv',
        jssp_cfg=cfg,
    )
    r.oht_config.enable_animation = False
    r.build_environment()
    r.build_layout()
    r.build_machines()

    mnm = dict(r.machine_node_map)   # {"M7": node_name, ...}
    load = float(getattr(r.oht_config, 'load_time_min', 0.0) or 0.0)
    unload = float(getattr(r.oht_config, 'unload_time_min', 0.0) or 0.0)
    handling = load + unload

    def mnum(s):
        d = ''.join(c for c in str(s) if c.isdigit())
        return int(d) if d else 0
    names = sorted(mnm.keys(), key=mnum)

    rows = []
    unreachable = 0
    for a in names:
        na = mnm[a]
        for b in names:
            if a == b:
                continue
            nb = mnm[b]
            cost = r._dijkstra_cost(na, nb)
            if cost <= 0.0:            # unreachable or same node
                unreachable += 1
                continue
            rows.append((a, b, round(cost + handling, 3)))

    out = 'machine_travel.csv'
    with open(out, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['from_machine', 'to_machine', 'with_handling_sec'])
        w.writerows(rows)
    print(f"[machine_travel] {len(rows)} pairs written → {out} "
          f"(handling +{handling:.0f}s = load {load:.0f} + unload {unload:.0f}, unreachable={unreachable})")


if __name__ == '__main__':
    main()
