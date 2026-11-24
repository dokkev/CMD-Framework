# Related-work notes (Literature Review stage)

All keys refer to `refs.bib` in this directory. Every entry was checked by web search on 2026-11-21 (authors, title, venue, year). Fields that no source confirmed were left out of the BibTeX. Entries marked *(preprint)* are arXiv-only as far as the search could tell.

## 1. Vision-based and tactile finger sensors that take up physical volume

Camera-based tactile sensors started with GelSight `yuan2017gelsight`. Later work shrank and reshaped them to fit robot fingers: GelSlim `donlon2018gelslim`, DIGIT `lambeta2020digit`, GelSight Wedge `wang2021gelsightwedge`, 9DTact `lin2024ninedtact`. Others made the sensor round or 3D so it can touch from many directions: Insight `sun2022insight`, Minsight `andrussow2023minsight`, DenseTact `do2022densetact`, AllSight `azulay2024allsight`, Digit 360 `lambeta2024digit360` *(preprint)*. GelSight Svelte `zhao2023svelte` covers a whole finger. Magnetic skins `bhirangi2021reskin` are thinner, but they still add a compliant layer. Many of these papers report packaging tradeoffs directly. GelSlim, for example, discusses the tradeoff between finger thickness, camera depth of field and pad size, and Svelte names articulation as future work. Even so, all of them treat the finger or hand that carries the sensor as fixed. **Gap:** none of them asks how the joint architecture limits where a thick sensing or compliant layer can sit as the finger flexes. That clearance question is what this paper addresses.

## 2. Dexterous and research robot hands

Our surveys and classic references are `salisbury1982articulated` and `piazza2019century`. Learning research relies on the Shadow hand `openai2020dexterous` and the Allegro hand (`qi2022hora`, `qi2023rotateit`), and increasingly on low-cost or open designs: TriFinger `wuthrich2020trifinger`, LEAP `shaw2023leap`, D'Manus `bhirangi2022allthefeels` *(preprint)*, ORCA `christoph2025orca` *(preprint)*, Faive `toshimitsu2023faive`, and the linkage-driven ILDA hand `kim2021ilda`. Tactile sensing usually goes onto these hands as add-on fingertips or skins, such as DIGIT on Allegro in `qi2023rotateit`, ReSkin on D'Manus, and ORCA's fingertip sensors. **Gap:** in these hands, link and joint geometry are fixed first, so changing the pad or sensor means redesigning the finger. That is the confound this paper motivates. None of these hands is parameterized by its contact interface.

## 3. Soft and compliant finger pads, contact mechanics

Classic work studied pad materials and friction (`cutkosky1987skin`, `shimoga1996soft`) and soft-contact models: the power-law contact radius `xydas1999soft`, an elastic fingertip potential `inoue2006elastic`, and an adaptive soft-finger model for simulation `ciocarlie2007soft`. `kao2016contact` is the handbook treatment. `lu2019softfingertips` shows that fingertip hardness and geometry change the reachable in-hand workspace, which has direct consequences for hand design. `bhatt2021surprisingly` argues that compliant hardware makes manipulation robust. **Gap:** these papers model or exploit pad compliance on a given finger. They do not consider pad *thickness* as a geometric design variable that interacts with joint kinematics, for example pads colliding or losing coverage at high flexion.

## 4. Joint mechanisms and transmissions for fingers

Finger joints in the literature include:

- Serial pin joints with direct or remote (tendon) actuation: `salisbury1982articulated`, `birglen2004kinetostatic`, `dollar2010sdm`.
- Rolling-contact joints: CORE `cannon2005core`, `kim2016rolling`, Faive `toshimitsu2023faive`, general-surface rolling kinematics `ha2024rolling` *(preprint)*.
- Crossed or polycentric four-bar joints in fingers: `bai2016crossfourbar`, `bai2018monolithic`.
- The same four-bar joints in prosthetic knees, where the ICR trajectory is the design target: `phoengsongkhro2023knee`.
- Biomimetic joints without a fixed axis `xu2016biomimetic`, and pneumatic soft fingers `deimel2016rbo2`.
- Linkage-driven fingers with the actuators placed in the palm: `kim2021ilda`.

Joint papers argue for compactness, robustness or a human-like moving center of rotation. **Gap:** no paper compares these architectures by how much pad-carrying surface survives as pad thickness and flexion grow. Polycentric and rolling joints are known to move the instantaneous center. Nobody has used the *finite* displacement pole to derive a pad clearance law or to redesign a four-bar so that contact-face motion is preserved under thicker pads.

## 5. Computational design and co-design of grippers, fingers and hands

Design optimization runs from quasi-static underactuated-hand optimization `ciocarlie2010datadriven` to learned generative finger shapes `ha2020fit2form`, differentiable contact-aware morphology `xu2021contactaware`, hardware-as-policy co-optimization `chen2020hwasp`, diffusion-based manipulator design `xu2024dgdm`, and co-design of morphology and behaviour `luck2019coadaptation`, `pathak2019selfassembling`. Recent work co-designs dexterous hands `fay2026dextra` and fingertip geometry with policies `ye2025power` *(preprint)*. **Gap:** these methods optimize finger or link shape for a task. They do not keep the contact interface as an exchangeable module with a fixed specification while the joint is re-synthesized around it. Most also treat joint type as given rather than comparing architectures.

## 6. Embodiment as a variable or confound in learning

Cross-embodiment learning conditions policies on morphology (`huang2020onepolicy`, `gupta2022metamorph`, `patel2024getzero` *(preprint)*, `yuan2025crossdex`) or scales data or embodiments across robots (`oxe2024`, `ai2025embodimentscaling`). On the touch side, `zhao2024t3` and `higuera2024sparsh` learn representations across heterogeneous tactile sensors, and `wang2022tacto` simulates vision-based sensors. All of them confirm that hardware differences shift learned behaviour. **Gap:** they vary kinematics, or average over sensor differences. None isolates the contact interface (pad material, thickness, sensor module) as the *only* variable while the rest of the finger kinematics stays the same. A finger that keeps contact-face motion under interface swaps would make that controlled experiment possible.

## 7. Classical kinematics, geometry, optimization

The finite displacement pole (centre of finite rotation) of planar rigid motion and its use in linkage synthesis come from `bottema1979theoretical`, `mccarthy1990introduction` and `mccarthy2011geometric`. Kinematics notation and contact or grasp background come from `murray1994mathematical` and `kao2016contact`. The separating-axis collision test comes from `gottschalk1996obbtree`, and multi-objective search from `deb2002nsga2`.

## Verification table

| key | source URL used to verify |
|---|---|
| yuan2017gelsight | https://www.mdpi.com/1424-8220/17/12/2762 |
| donlon2018gelslim | https://arxiv.org/abs/1803.00628 ; https://dspace.mit.edu/handle/1721.1/125641 |
| lambeta2020digit | https://arxiv.org/abs/2005.14679 ; https://fis.tu-dresden.de (RA-L 5(3) record) |
| wang2021gelsightwedge | https://arxiv.org/abs/2106.08851 ; https://ieeexplore.ieee.org/document/9560783 |
| sun2022insight | https://www.nature.com/articles/s42256-021-00439-3 |
| do2022densetact | https://arxiv.org/abs/2201.01367 ; https://par.nsf.gov/biblio/10473813 |
| andrussow2023minsight | https://is.mpg.de/publications/andrussow23-ais-minsight ; https://api.crossref.org/works/10.1002%2FAISY.202300042 |
| zhao2023svelte | https://arxiv.org/abs/2309.10885 ; https://researchr.org/publication/ZhaoA23-5 |
| azulay2024allsight | https://arxiv.org/abs/2307.02928 ; https://cris.tau.ac.il/en/publications/allsight-a-low-cost-and-high-resolution-round-tactile-sensor-with/ |
| lin2024ninedtact | https://arxiv.org/abs/2308.14277 ; https://researchr.org/publication/LinZXWX24 |
| lambeta2024digit360 | https://arxiv.org/abs/2411.02479 ; https://ai.meta.com/research/publications/digitizing-touch-with-an-artificial-multimodal-fingertip/ |
| bhirangi2021reskin | https://proceedings.mlr.press/v164/bhirangi22a.html |
| salisbury1982articulated | https://www.citedrive.com/en/discovery/articulated-hands (DOI 10.1177/027836498200100102) |
| piazza2019century | https://arpi.unipi.it/handle/11568/995616 ; https://portal.fis.tum.de/en/publications/a-century-of-robotic-hands/ |
| openai2020dexterous | https://arxiv.org/abs/1808.00177 ; https://eld.bl.uk/catalog/100094909964_0x000001 |
| wuthrich2020trifinger | https://proceedings.mlr.press/v155/wuthrich21a.html |
| kim2021ilda | https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8671524/ |
| qi2022hora | https://proceedings.mlr.press/v205/qi23a.html |
| bhirangi2022allthefeels | https://arxiv.org/abs/2210.15658 |
| shaw2023leap | https://roboticsproceedings.org/rss19/p089.html |
| qi2023rotateit | https://proceedings.mlr.press/v229/qi23a.html |
| christoph2025orca | https://arxiv.org/abs/2504.04259 |
| cutkosky1987skin | https://dblp.uni-trier.de/rec/conf/icra/CutkoskyJW87.html |
| shimoga1996soft | https://researchr.org/publication/ShimogaG96 |
| xydas1999soft | https://api.crossref.org/works/10.1177%2F02783649922066673 ; https://www.osti.gov/biblio/20001201 |
| inoue2006elastic | https://www.cambridge.org/core/journals/robotica/article/feedback-control-for-object-manipulation-by-a-pair-of-soft-tip-fingers/EDE4D84BC4D7FB66668937BA17E86BD9 (citation record) |
| ciocarlie2007soft | https://dblp.uni-trier.de/rec/conf/haptics/CiocarlieLA07.html ; https://academiccommons.columbia.edu/doi/10.7916/D8HD84Z2 |
| kao2016contact | https://link.springer.com/chapter/10.1007/978-3-319-32552-1_37 |
| lu2019softfingertips | https://spiral.imperial.ac.uk/entities/publication/4e93fd70-234a-4699-a0fa-6c84e98d5adc |
| bhatt2021surprisingly | https://roboticsproceedings.org/rss17/p089.html |
| birglen2004kinetostatic | https://search.isc.ac/inventory/49/951486.htm ; https://publications.polymtl.ca/34716/ |
| cannon2005core | https://www.me.byu.edu/faculty/larryhowell (lists IDETC 2005 best-paper finalist) |
| dollar2010sdm | https://softroboticstoolkit.com/publications/highly-adaptive-sdm-hand-design-and-performance-evaluation |
| deimel2016rbo2 | https://dblp.uni-trier.de/rec/journals/ijrr/DeimelB16.html ; https://depositonce.tu-berlin.de/items/131ec20b-6ba6-4f1c-8af5-bd2178dda079/full |
| xu2016biomimetic | https://dblp.uni-trier.de/rec/conf/icra/XuT16.html |
| kim2016rolling | https://researchr.org/publication/KimISC16 |
| bai2016crossfourbar | https://researchr.org/publication/BaiWK16 ; https://api.crossref.org/works/10.1007%2F978-3-319-42417-0_3 |
| bai2018monolithic | https://dblp1.uni-trier.de/rec/conf/humanoids/BaiR18.html |
| phoengsongkhro2023knee | https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10739707/ |
| toshimitsu2023faive | https://arxiv.org/abs/2308.02453 ; https://www.research-collection.ethz.ch/handle/20.500.11850/653537 |
| ha2024rolling | https://arxiv.org/abs/2409.00616 |
| ciocarlie2010datadriven | https://dblp.dagstuhl.de/rec/conf/icra/CiocarlieA10.html |
| luck2019coadaptation | https://proceedings.mlr.press/v100/luck20a.html |
| pathak2019selfassembling | https://proceedings.neurips.cc/paper/2019/hash/c26820b8a4c1b3c2aa868d6d57e14a79-Abstract.html |
| ha2020fit2form | https://proceedings.mlr.press/v155/ha21b.html |
| chen2020hwasp | https://proceedings.mlr.press/v155/chen21a.html |
| xu2021contactaware | https://www.roboticsproceedings.org/rss17/p008.html |
| xu2024dgdm | https://proceedings.mlr.press/v270/xu25d.html |
| ye2025power | https://arxiv.org/abs/2511.13710 |
| fay2026dextra | https://arxiv.org/abs/2512.03743 ; https://www.iclr.cc/virtual/2026/poster/10007820 |
| huang2020onepolicy | https://proceedings.mlr.press/v119/huang20d.html |
| gupta2022metamorph | https://iclr.cc/virtual/2022/poster/6939 ; https://arxiv.org/abs/2203.11931 |
| wang2022tacto | https://arxiv.org/abs/2012.08456 ; https://ai.meta.com/research/publications/tacto-a-fast-flexible-and-open-source-simulator-for-high-resolution-vision-based-tactile-sensors/ |
| oxe2024 | https://arxiv.org/abs/2310.08864 (IEEE ICRA 2024 record, DOI 10.1109/icra57147.2024.10611477) |
| patel2024getzero | https://arxiv.org/abs/2407.15002 |
| zhao2024t3 | https://proceedings.mlr.press/v270/zhao25c.html |
| higuera2024sparsh | https://proceedings.mlr.press/v270/higuera25a.html |
| yuan2025crossdex | https://proceedings.iclr.cc/paper_files/paper/2025/hash/ca8c6f28d8ba1e732e3f217ab05c4ec0-Abstract-Conference.html |
| ai2025embodimentscaling | https://arxiv.org/abs/2505.05753 ; https://embodiment-scaling-laws.github.io |
| bottema1979theoretical | https://openlibrary.org/works/OL4462650W ; https://libcat.library.tamu.edu/Record/in00000153725 |
| mccarthy1990introduction | https://find.library.upatras.gr/Record/138163 |
| mccarthy2011geometric | https://link.springer.com/book/9781441978912 |
| murray1994mathematical | https://catalog.hathitrust.org/Record/002811962 |
| gottschalk1996obbtree | https://history.siggraph.org/?p=116994 ; https://www.cs.unc.edu/~geom/OBB/paper.html |
| deb2002nsga2 | https://repository.ias.ac.in/9403 |
| hsu2019rcm | https://www.researchgate.net/publication/338942048 (IROS 2019, pp. 3172-3178; verified 2026-11-24) |
| khullar2021rcm | https://waseda.elsevierpure.com/en/publications/a-multi-fingered-robot-hand-with-remote-center-of-motion-mechanis/ (ROBIO 2021, pp. 564-570; verified 2026-11-24) |

### Caveats on individual fields

- `deb2002nsga2`: volume, issue and pages were verified. The DOI (10.1109/4235.996017) is the standard one, but it did not appear in the search snippet.
- `ciocarlie2007soft`: authors, venue (WHC/Haptics 2007) and pages were verified. The exact title wording comes from the Columbia record. Check the capitalization against IEEE Xplore.
- `inoue2006elastic`: only the starting page (1273) was confirmed.
- `ai2025embodimentscaling`: the first author's given name (Bo) and the CoRL 2025 acceptance come from the arXiv listing and the project page. The full author list is truncated.
- `fay2026dextra`: the published title in later arXiv versions is "House of Dextra: Cross-Embodied Co-Design for Dexterous Hands". The ICLR 2026 poster uses the shorter title.
- `bhirangi2022allthefeels`, `christoph2025orca`, `lambeta2024digit360`, `ha2024rolling`, `patel2024getzero`, `ye2025power`: the search found only an arXiv version. ORCA may have appeared at IROS 2025, per an OpenAlex DOI 10.1109/IROS60139.2025.11246681, but this is unconfirmed. Recheck all of these before submission.
- Long author lists were shortened to "and others" where the full list was not seen: `openai2020dexterous`, `wuthrich2020trifinger`, `kim2021ilda`, `toshimitsu2023faive`, `higuera2024sparsh`, `lambeta2024digit360`, `bhirangi2022allthefeels`, `ye2025power`.

## Looked for but not verified (excluded)

- **Collins / Herder "CORE" rolling joint**: no such attribution found. CORE is Cannon, Lusk and Howell (BYU), so `cannon2005core` is cited instead.
- **"Kim / Cho rolling joint" journal paper**: only conference papers were found, so `kim2016rolling` (BioRob 2016) is cited. Kim and Cho, ICCAS 2016, also exists but was not added.
- **Hillberry rolling-contact joint (1976)**: could not verify, so it was left out.
- **D'Manus at ICRA 2023**: the venue is not confirmed, so the entry is cited as arXiv.
- **Allegro, Shadow and Inspire product pages**: no stable official page was confirmed (allegrohand.com only appeared through third parties). The hands are cited through papers that use them (`qi2022hora`, `openai2020dexterous`). DexHand and the Ability Hand were not searched.
- **2017 Prosthetics and Orthotics International four-bar knee paper** (DOI 10.1177/0309364616652017): no author list was found, so `phoengsongkhro2023knee` is cited instead.
- **Crossed four-bar or polycentric knee exoskeleton papers**: none was verified in this pass.
- **Studies that vary fingertip friction or compliance as a controlled variable in learned in-hand manipulation (simulation)**: no clean, peer-reviewed example was found. This is a positioning opportunity rather than a missing citation.
