#!/usr/bin/env python3
"""Study 4: matched hypothetical bridge datums and geometric screening."""
from pathlib import Path
import json
import sys
import argparse
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT), str(HERE.parent)]
import shared
from core.optimization.analysis.transmission import Transmission
from core.optimization.analysis.bridge import attachment, screen, hardware_gap_mm
from core.optimization.mechanism import read_config_file, mechanism


def run(path=HERE / 'config.yaml'):
    cfg = read_config_file(path)
    out = ROOT / cfg['output']
    if out.exists():
        raise FileExistsError(f'results already exist: {out}; choose a new output')
    out.mkdir(parents=True)
    refs = {r.reference_id:r for r in shared.load_references()}
    rows, paths, designs = [], [], []
    for rid in cfg['references']:
        r = refs[rid]
        t = Transmission(r.mechanism, r.x, r.config)
        designs.append(dict(reference_id=rid, mechanism=r.mechanism, design=list(r.x),
                            config=mechanism(r.mechanism).config_to_json(r.config)))
        for depth in cfg['skin_depths_m']:
            p0 = np.array([cfg['proximal_x_m'], -depth])
            d0 = np.array([cfg['distal_x_m'], -depth])
            span = float(np.linalg.norm(d0-p0))
            for phi_deg in cfg['flexions_deg']:
                phi = np.radians(phi_deg)
                p, d = attachment(t, phi, p0, 'proximal'), attachment(t, phi, d0, 'distal')
                metrics, curve = screen(p, d, phi, span, cfg['curve_samples'])
                scene = t.scene(phi)
                gap = hardware_gap_mm(scene, curve, cfg['skin_radius_mm'], cfg['link_radius_mm'])
                rows.append(dict(reference_id=rid, depth_m=depth, phi_deg=phi_deg,
                     proximal_x_m=p[0], proximal_z_m=p[1], distal_x_m=d[0], distal_z_m=d[1],
                     neutral_span_m=span, **metrics, projected_hardware_gap_mm=gap,
                     projected_route_clear=gap >= 0))
                if phi_deg in (0, 30, 60, 90, 120):
                    for i, point in enumerate(curve):
                        paths.append(dict(reference_id=rid, depth_m=depth, phi_deg=phi_deg,
                                          node=i, x_m=point[0], z_m=point[1]))
    shared.write_csv(out/'screen.csv', rows)
    shared.write_csv(out/'paths.csv', paths)
    (out/'manifest.json').write_text(json.dumps(dict(config=cfg, source=shared.git_commit(out),
        references=designs, formulation='common neutral datums; rigid attachments; chord lower bound; prescribed tangent cubic Hermite path',
        scope='hypothetical skin extensions, not existing pad endpoints; projected hardware routing, no material equilibrium, contact loading or durability'), indent=2))
    # Preserve the exact config, governing goal and common reference definitions.
    import shutil
    for src in [Path(path), ROOT/'GOAL.md', ROOT/shared.REFERENCES]:
        target = out/'inputs'/src.resolve().relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)
    print(json.dumps(dict(output=str(out), rows=len(rows))))
    return out


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('config', nargs='?', type=Path, default=HERE/'config.yaml')
    run(p.parse_args().config)
