# CoDe Hand: North Star

Functional contact interfaces require substantial physical volume. Thick compliant
structures, specialized surfaces and sensor modules compete with servo bodies and
articulation hardware for space. Changing an interface can consequently require
expensive redesign of finger dimensions, joints or transmission geometry. If both
the interface and underlying morphology change, manipulation-performance changes
cannot be attributed to the interface alone.

**CoDe is a contact-centric hand design and research platform that enables
systematic contact-interface variation while keeping underlying morphology,
actuation and articulation as consistent as practical.** Reduce the cost of
interface changes and make unavoidable embodiment changes explicit. Mechanism
choices and individual studies serve this objective; they do not replace it.

## Supporting evidence

- Study 1: interfaces accommodated by frozen reference morphologies (ROM and
  usable contact coverage). Preserve the established formulation and results.
- Study 2: mechanism and external morphology changes required to accommodate a
  target interface, recorded separately. Preserve the established baseline.
  A separate compact-accommodation extension relaxes length/proximity bounds,
  first minimizing total finger length and then support/articulation change;
  report proximal/distal support spans and articulation shifts explicitly.
- Study 3: how transmissions reflect identical local contact mechanics to the
  actuator output shafts. Use existing optimizer candidates, recheck feasibility,
  and distinguish matched comparisons from sampled historical design spaces.
- Study 4: whether a bridge across proximal/distal regions can make the joint a
  continuous functional contact interface. Start with matched geometric screening;
  test, rather than presume, a polycentric advantage.

## Claim discipline and decisions

Keep physical CoDe, physical RCJ and declared idealized ablations separate. An
idealization removes only its named constraint. Report failures and negative
results. Never tune constraints to make CoDe win. Candidate designs are design
alternatives, not statistical replicates. Matched morphology reduces confounding;
it does not establish perfect causal isolation.

Separate demonstrated observations, model-derived findings, assumptions and
hypotheses. Planar feasibility is not buildability; local reflected stiffness is
not validated actuator response; geometric bridge continuity is not skin durability
or usable loaded contact. Defer production hardware details, sensing and policy
experiments unless evidence warrants a separate study.

Quantitative figures must read saved results with source commit/patch, configs,
input artifacts and sampling provenance. Missing models or artifacts block their
branch of work; do not invent measurements. The next useful research decision is
one that improves controlled interface variation or exposes its limits, whether or
not it favors CoDe.
