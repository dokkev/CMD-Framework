# Study 3: transmission-reflected contact mechanics

This supports the root `GOAL.md`: holding a platform fixed does not make its
transmission irrelevant to interface comparisons. Map identical interface laws
to actuator coordinates, without presenting local mechanics as manipulation
performance or measured actuator response.

## Coordinates, model and derivation

The XZ plane is the repository's side view: O is the MCP origin, +X distal,
+Z dorsal; positive flexion turns +X toward -Z. Actuator coordinates are
`a = [MCP, PIP servo]` at the **servo output shafts**, rad. Internal servo
gearing is unspecified. Linkage servo motion may have either sign; retain the
actual signed angle. MCP is zero for reported configurations, but the analytic
Jacobian and tests include world-frame rotation. A point fraction runs from the
outer end (0) toward the joint (1) on each existing contact face. Normals are
outward/palmar in world XZ. Existing FK/solids return mm; the adapter converts
positions and derivatives to m and m/rad.

Let `p(a)` be the selected material contact position, `J = dp/da`, and freeze
its normal n at the evaluation pose. `Kc = kt I + (kn-kt) n n^T`, with an
analogous `Bc`. A grounded, zero-preload, massless Kelvin-Voigt interface gives

```
delta f_on_finger = -Kc J delta a - Bc J delta adot
restoring actuator torque = -Ka delta a - Ba delta adot
Ka = J^T Kc J                  [N m/rad]
Ba = J^T Bc J                  [N m s/rad]
Za(s) = Ba + Ka/s              [torque / angular velocity]
```

Saved `Ka`/`Ba` are positive restoring coefficients. The applied balancing effort
has the opposite sign to the restoring torque. `Z` is a quasi-static transfer
quantity, not a prediction of resonant or closed-loop response. The supplied
illustrative laws use kn=1000 N/m, bn=1 Ns/m, with kt=0 or 200 N/m and bt=0
or 0.2 Ns/m. These are assumptions, not fitted physical measurements. Thus
`Ba = 0.001 Ka s` for these configs; damping is not independent evidence.

Derivatives use existing kinematic solutions, never a second closure solver:

- MCP: `dp/damcp = -perp(p)`.
- Four-bar: existing FK supplies A, A', B, B'. With output servo on A',
  `Bdot = -perp(B-A')`. Differentiating the fixed lengths gives
  `B'dot = perp(B'-A) [(B-B') dot Bdot] / [(B-B') dot perp(B'-A)]`.
  The distal angular velocity is `cross(B-B', Bdot-B'dot)/|B-B'|^2`;
  a distal material point adds this rotation to `Bdot`. Singular denominators
  fail explicitly. No unsigned transmission ratio is substituted.
- Direct-drive and 1:1 belt 2R: fixed joint origin A,
  `dp/dapip = -perp(p-A)`.
- Equal-radius RCJ: the servo drives the carrier, `phi = 2 apip`;
  `Bdot = -perp(B-A)` and `pdot = Bdot - 2 perp(p-B)`.

Every reported contact node is checked against central differences of the
existing servo FK (`1e-6 rad`). Tests additionally check energy equivalence,
PSD, a known 2R lever arm and coordinate-rotation invariance.

## Comparisons and sampling

Primary: Study 1–2 matched references, the same modules and ROM 120°, current
mounting policy, pivot/servo/support/face checks. Designs that fail are saved
and excluded from reflection. The fixed reference's E/Q/A is retained;
mounting depths follow the established per-module policy, not a new optimization.
Actuation orientations are separate references. Equal external descriptors do
not imply identical instantaneous centers or Jacobians.

Secondary: existing Study 2 `long`, seed 0, Level 2 mechanism fronts for all
four physical mechanisms and ideal RCJ. No new optimizer runs. The common
finger-length range is the existing Study 2 range [70,110] mm; all other
current common rules, target modules and ROM remain equal. This is a design
space conditioned on prior adaptation, **not** a native architecture-optimal
front. Seed 0 is a deterministic provenance choice, not an optimizer robustness
claim. Original objectives are the saved adaptation objectives, so many designs
are tied (2R has no Q). Select 41 proposals per front using a normalized-objective
farthest-point cover with normalized design-space distance to resolve objective
ties. Original CSV rows and exact selected indices are saved. No random design
sampler, p-values, confidence intervals or statistical-replicate interpretation.

The full original fronts are copied and plotted as archived design-space
context. Reflected-mechanics distributions use the systematic covers only;
there is no claim to evaluate every archived row or estimate a population law.
Re-evaluate each chosen design with current evaluators and the common interface.
Do not claim it remains Pareto optimal after this change of conditions.

Primary abstraction: one point at the center of one contact face, analyzed
separately for proximal and distal regions. Secondary: five independent grounded
point springs at fractions .1,.3,.5,.7,.9, weights 1/5. The **total** stiffness
and damping are held equal to the single-point law. This tests spatial
second-moment effects, without confounding node count with total stiffness.
It assumes simultaneous independent engagement, not a solved object-contact
force distribution. It does not couple proximal and distal contacts or solve
object equilibrium. No finite object geometry or contact activation is solved.

Save complete 2x2 coefficients (symmetric off-diagonal once), normalized traces
`tr(Ka)/(kn L^2)` and `tr(Ba)/(bn L^2)`, and impedance Frobenius norm at .5,
2 and 10 Hz. L is each design's external extended finger length. Normalization
removes a gross length scale but cannot remove kinematic or actuation differences.
Figures aggregate the same pose grid within each design before comparing
alternatives. Neither higher nor lower reflected stiffness is universally better.

## Idealization and deferred effects

Physical RCJ retains joint servo and roller packaging; ideal RCJ is the existing
no-joint-servo upper-bound model. It does not remove all rollers: minimum pin
housing/radius and existing collision checks remain. Cable retention is absent
from both. No new ideal CoDe model is introduced; physical CoDe never silently
loses its linkage or pivot/servo constraints. Ideal RCJ is reported separately
and is not compared as a physical packaged design.

Zero preload deliberately removes geometric stiffness. At nonzero force a
consistent tangent also includes `sum_i f_i Hessian(p_i)` (with signs defined
by the chosen energy/load convention) and any rotating-frame/load-direction
terms. They are deferred, not hidden inside `J^T Kc J`. Frictional stick/slip,
preload/contact activation, coupled multi-contact load distribution, nonlinear
viscoelasticity, transmission inertia, belt elasticity, backlash, internal servo
gearing and motor controller dynamics are deferred. Tangential springs model
local sticking only, not a friction cone. No physical actuator response claim.

## Reproduce and artifacts

Use the installed `cmd` Python environment (NumPy, Numba, PyMoo, Matplotlib,
PyYAML, pytest). Local interpreter: `/home/dk/miniconda3/envs/cmd/bin/python`.

```
python scripts/study/study3_transmission/run.py --pilot
python scripts/study/study3_transmission/run.py
python scripts/study/figures34.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_transmission_contact.py -q
```

`--pilot` uses three proposals/front and a reduced pose grid; it is diagnostic
only. Existing result directories are protected; use a copied config with a new
`output` for reruns. Source commit plus working-tree patch/untracked sources,
resolved model configs, input hashes, original fronts/metadata and selected
indices are retained. The uploaded `/mnt/data/붙여넣은 마크다운.md` was unavailable;
no uninspected content from it is used. Study 1–2 data/formulations remain intact.
