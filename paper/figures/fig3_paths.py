"""Contact-point paths for Fig. 3(h): Study 1 3 mm reference, Study 2 highest-coverage 7 mm redesign and Study 1 7 mm design at 120 deg, per architecture -> design/output/paper/fig3_paths_q120_3to7.json.

    conda run -n cmd python paper/figures/fig3_paths.py
"""
import sys, csv, json
from pathlib import Path
D = Path('/home/dk/workspace/cmd_framework/design')
for p in (D, D/'scripts'/'study', D/'scripts'/'study'/'study2_morphology_adaptation'):
    sys.path.insert(0, str(p))
import numpy as np
import one_finger as study1
from core.optimization.analysis.contact_kinematics import paths
from core.optimization.mechanism import mechanism, read_config_file
cfg = read_config_file(study1.DEFAULT_CONFIG)
F = D/'output/study/study2_redesign_2026-11-21/front.csv'
rows = list(csv.DictReader(open(F)))
out = {}
for arch in ('cfl', '2r_remote', '2r_dd', 'rcj'):
    spec = cfg['architectures'][arch]
    ds = study1.study1_designs(arch, spec, 120.0, cfg)
    first = ds[3.0]; base = mechanism(first.mechanism)
    ref = paths(base, first.x, first.config)
    own = [r for r in rows if r['architecture']==arch and r['required_flexion_deg']=='120.0' and r['to_mm']=='7.0']
    best = max(own, key=lambda r: float(r['c_pad_mean']))
    x = [float(v) for v in best['x'].split(';')]
    red = paths(base, x, ds[7.0].config)
    u = paths(base, ds[7.0].x, ds[7.0].config)  # Study 1's coverage-only 7 mm design
    d = np.max(np.linalg.norm(red['contact']-ref['contact'],axis=1))
    du = np.max(np.linalg.norm(u['contact']-ref['contact'],axis=1))
    print(arch, 'redesign Dc %.2f (csv %.2f)  study1-7mm Dc %.2f' % (d, float(best['contact_change_mm']), du))
    out[arch] = {'q': ref['q'].tolist(), 'ref': ref['contact'].tolist(), 'red': red['contact'].tolist(), 'u': u['contact'].tolist(),
                 'ref_tip': ref['tip'].tolist(), 'red_tip': red['tip'].tolist(), 'u_tip': u['tip'].tolist()}
Path('/home/dk/workspace/cmd_framework/design/output/paper').mkdir(exist_ok=True)
json.dump(out, open('/home/dk/workspace/cmd_framework/design/output/paper/fig3_paths_q120_3to7.json','w'))
