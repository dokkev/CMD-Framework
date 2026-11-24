"""PI report of Studies 0-4: one JSON of every number the report shows, with its metadata, and a static HTML page of
tables (no figures, no scripts) for printing to PDF from a browser.

    python3 report/pi/make_report.py   ->  report/pi/pi_report_data.json, report/pi/pi_report.html

Reads only committed files under report/ (and the Drake per-placement extract report/study3/data/
drake_e21_point_pairs_RU.json); the page text is report/pi/template.html.
"""
from __future__ import annotations

import csv
import datetime
import html
import json
import math
import statistics
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REP = ROOT / "report"
OUT = REP / "pi"
L_F_MM = 90.0  # finger length of the study model; Study 2 changes are stored as a fraction of it

ARCH1 = ["CFL", "2R-remote", "2R-DD", "RCJ", "CFL ideal", "RCJ ideal"]
NAME = {"CFL": "CFL (CoDe)", "2R-remote": "2R-remote", "2R-DD": "2R-DD", "RCJ": "RCJ",
        "CFL ideal": "CFL ideal (상한)", "RCJ ideal": "RCJ ideal (상한)"}
ARCH2 = {"cfl": "CFL", "2r_remote": "2R-remote", "2r_dd": "2R-DD", "rcj": "RCJ", "cfl_ideal": "CFL ideal",
         "rcj_ideal": "RCJ ideal"}
SETTINGS = [  # key, file, label, what it is (group and task measure in GROUP and MEASURE below)
    ("isaac_A", "isaac_dose_A_all.json", "Isaac Lab",
     "Isaac Lab 3 (Newton MuJoCo-Warp), point contact; O's placements; friction mu 1.3 -> 2.0; pull-out"),
    ("drake_point", "drake_e21_point.json", "Drake point",
     "Drake (SAP), point contact 2.5 kN/m; experiment 21; pull-out"),
    ("drake_hydro", "drake_e21_hydro.json", "Drake hydro",
     "Drake, hydroelastic pads 0.1 MPa over 3 mm on every hand; experiment 21; pull-out"),
    ("soft_pad", "drake_e25_hydro_own.json", "두께만큼 부드러운 pad",
     "Drake, hydroelastic pads 0.1 MPa over each pad's own thickness, against O carrying the same pad; experiment 25"),
    ("acquire", "drake_e23_point.json", "grasp 획득",
     "Drake point; closing from open to draw a cylinder in against a load; experiment 23"),
    ("dial", "drake_e26_point.json", "다이얼",
     "Drake point; turning a sprung dial by 15-42 deg in one closing; experiment 26"),
    ("contact_setpoint", "drake_e27_point.json", "접촉 기준 setpoint",
     "Drake point; each hand's joint setpoint re-chosen from the kinematics so its distal fingertip contact point "
     "matches O's (no runtime contact); experiment 27; pull-out"),
    ("rolling", "drake_e22_point.json", "in-hand rolling",
     "Drake point; rolling a cylinder in hand by a flexion sweep; experiment 22"),
]
GROUP = {"isaac_A": "contact", "drake_point": "contact", "drake_hydro": "contact", "soft_pad": "contact",
         "acquire": "task", "dial": "task", "rolling": "task", "contact_setpoint": "command"}
MEASURE = {  # what each setting measures; D = measure(mu 2.0) - measure(mu 1.3) in every setting
    "isaac_A": "pull-out limit load (N)", "drake_point": "pull-out limit load (N)",
    "drake_hydro": "pull-out limit load (N)", "soft_pad": "pull-out limit load (N); reference O carries the same pad",
    "acquire": "distance the cylinder is drawn in toward the MCP (mm)", "dial": "dial rotation (deg)",
    "rolling": "cylinder rotation during the flexion sweep (deg)",
    "contact_setpoint": "pull-out limit load (N), joint setpoint re-chosen per hand from the kinematics"}
DIRECTION = {"+x": "+x (손가락 끝 쪽)", "-x": "−x (손바닥 쪽)", "-z": "−z (빼내는 쪽)", "+x-z": "+x−z (대각)"}
EVAL_KEYS = ["clean_timeout_rate", "signed_rotation_deg_mean", "quarter_turn_within_40mm_and_60deg_rate",
             "half_turn_within_40mm_and_60deg_rate", "three_quarter_turn_within_40mm_and_60deg_rate",
             "full_turn_and_clean_timeout_rate"]
CHECKPOINTS = [("s15", "s15 (현재 최고)", 4597), ("scale_seg1", "MLP 확장 +500 iter", 5096),
               ("scale_seg2", "MLP 확장 +1,000 iter", 5595), ("scale_seg3", "MLP 확장 +1,500 iter", 6094)]


def rel(p: Path) -> str:
    return str(p.relative_to(ROOT))


def git(*args: str) -> str:
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def per_hand(path: Path) -> list[dict]:
    d = json.loads(path.read_text())
    if isinstance(d, dict) and "hands" in d:  # Isaac Lab summary: hands keyed by name
        return [{"hand": k, "thickness_mm": v["thickness_mm"], "d_contact_mm": v["D_c_mm"], "d_skel_mm": v["D_s_mm"],
                 "E": v["E_rel_median"], "pairs": v["pairs"]} for k, v in d["hands"].items()]
    rows = d["rows"] if isinstance(d, dict) else d
    return [{"hand": r["hand"], "d_contact_mm": r["d_contact"], "d_skel_mm": r["d_skel"], "E": r.get("E"),
             "pairs": r.get("pairs")} for r in rows]


def build() -> dict:
    src = []

    s1_path = REP / "study1/data/thickness_coverage_summary.csv"
    src.append(rel(s1_path))
    study1 = [{"architecture": r["architecture"].replace("*", ""), "required_flexion_deg": int(float(r["required_flexion_deg"])),
               "pad_thickness_mm": float(r["thickness_mm"]), "c_pad": float(r["c_pad_mean"]),
               "c_pad_worst_run": float(r["c_pad_run_low"]), "runs": int(r["runs"])}
              for r in csv.DictReader(s1_path.open())]

    s2_path = REP / "study2/data/redesign_summary.csv"
    src.append(rel(s2_path))
    study2 = [{"architecture": ARCH2[r["architecture"]], "required_flexion_deg": int(float(r["required_flexion_deg"])),
               "from_mm": float(r["from_mm"]), "to_mm": float(r["to_mm"]),
               "d_contact_mm": float(r["change_at_most_c_pad"]) * L_F_MM,
               "fixed_axis_bound_mm": float(r["fixed_axis_change"]) * L_F_MM, "c_pad": float(r["most_c_pad"])}
              for r in csv.DictReader(s2_path.open()) if r["most_c_pad"]]

    pairs_path = REP / "study3/data/drake_e21_point_pairs_RU.json"
    src.append(rel(pairs_path))
    pairs = json.loads(pairs_path.read_text())

    settings = {}
    thickness = {}
    for key, fname, label, what in SETTINGS:
        p = REP / "study3/data" / fname
        src.append(rel(p))
        rows = [r for r in per_hand(p) if r["hand"] != "O"]
        settings[key] = {"label": label, "group": GROUP[key], "task_measure": MEASURE[key], "description": what,
                         "source": rel(p), "hands": rows}
        for r in rows:
            if r.get("thickness_mm"):
                thickness[r["hand"]] = r["thickness_mm"]
    grid = per_hand(REP / "study3/data/isaac_dose_A-grid_expanded.json")
    src.append(rel(REP / "study3/data/isaac_dose_A-grid_expanded.json"))
    for r in grid:
        thickness.setdefault(r["hand"], r["thickness_mm"])
    for s in settings.values():
        for r in s["hands"]:
            r["thickness_mm"] = thickness.get(r["hand"])

    study4 = []
    for prefix, label, it in CHECKPOINTS:
        for seed, h in ((94000, "17699c520a"), (95000, "823fbc8f87")):
            p = REP / f"study4/data/{prefix}_model_{it}_rotation_seed{seed}_{h}.json"
            src.append(rel(p))
            j = json.loads(p.read_text())
            study4.append({"checkpoint": label, "iteration": it, "eval_seed": seed, "settings_hash": h,
                           "num_grasps": j["num_envs"], "episode_s": j["episode_s_max"], "source": rel(p),
                           **{k: j[k] for k in EVAL_KEYS}})

    return {
        "metadata": {
            "title": "CoDe Hand Study 0-4 PI report: data",
            "created": datetime.date.today().isoformat(),
            "git_commit": git("rev-parse", "HEAD"),
            "generator": rel(Path(__file__)),
            "sources": sorted(set(src)),
            "definitions": {
                "c_pad": "pad coverage C̄_pad: mean share of the finger length covered by the proximal and distal pads "
                         "(fraction, 0-1), best design found by the search (GA and NSGA-II, 3 seeds each)",
                "c_pad_worst_run": "worst of the search runs for that cell",
                "required_flexion_deg": "q_req, the PIP flexion the design must reach without collision",
                "d_contact_mm": "Δ_contact: change of the distal contact point's path, seen from the proximal pad face "
                                "(the PIP column of the contact Jacobian), between the 3 mm reference and the redesign; "
                                "max over flexion (Study 2) or the hand's value (Study 3)",
                "d_skel_mm": "Δ_skel: change of the fingertip path seen from the link line (skeleton motion)",
                "fixed_axis_bound_mm": "lowest Δ_contact any fixed-axis joint can reach for that thickness change: "
                                       "2 sin(q_req/2) Δh",
                "F": "pull-out limit load (N): the load at which the two-pad grasp of the cylinder gives way",
                "D": "effect of the friction change: F(mu 2.0) - F(mu 1.3), N",
                "E": "error a redesign adds to the measured effect: |D(hand) - D(O)| / |D(O)|; per hand, the median "
                     "over the placements and directions scored on both hands",
                "O, R, U": "O: 3 mm reference finger; R: 7 mm redesign keeping the contact-surface motion "
                           "(Δ_contact 0.86 mm); U: 7 mm redesign for coverage only (Δ_contact 3.91 mm)",
                "clean_timeout_rate": "share of episodes in which the cube was still held at 20 s",
                "turn rates": "share of episodes whose cube turned at least 1/4, 1/2, 3/4 of a turn about the vertical "
                              "axis while within 40 mm and 60 deg tilt; the full turn also requires a clean timeout",
            },
            "units": {"lengths": "mm", "loads": "N", "rates and c_pad": "fraction (0-1)", "angles": "deg"},
            "scope": "Simulation only. Studies 1-2: 2D side model, finger length 90 mm. Study 3: CFL, q_req 120 deg, "
                     "joint impedance control with O's joint setpoint reused unless stated. Study 4: Isaac Lab 3, "
                     "hand O, 1,024 held-out grasps.",
        },
        "study1_coverage": study1,
        "study2_redesign": study2,
        "study3_pairs_drake_point": {"description": pairs["note"], "source": pairs["source"], "hands": pairs["hands"]},
        "study3_errors_by_hand": settings,
        "study4_in_hand_rotation": study4,
    }


def pct(v, d=1):
    return "" if v is None else f"{v * 100:.{d}f}%"


def table(head: list[str], rows: list[list[str]], num_from: int, cls: str = "", bold=lambda r: False) -> str:
    h = [f'<div class="tbl"><table class="{cls}"><thead><tr>']
    h += [f'<th class="{"num" if i >= num_from else ""}">{c}</th>' for i, c in enumerate(head)]
    h.append("</tr></thead><tbody>")
    for r in rows:
        b = " hl" if bold(r) else ""
        h.append("<tr>" + "".join(
            f'<td class="{("num" if i >= num_from else "") + b}"{c[1] if isinstance(c, tuple) else ""}>'
            f'{c[0] if isinstance(c, tuple) else c}</td>' for i, c in enumerate(r)) + "</tr>")
    h.append("</tbody></table></div>")
    return "".join(h)


def source(*paths: str) -> str:
    return '<p class="note">출처: ' + ", ".join(f"<code>{html.escape(p)}</code>" for p in paths) + "</p>"


def render(d: dict) -> str:
    s1 = d["study1_coverage"]
    rows = []
    for q in (90, 120, 150):
        for a in ARCH1:
            r = [f"{q}°", NAME[a]]
            for t in (3, 5, 7, 9):
                x = next((x for x in s1 if x["architecture"] == a and x["required_flexion_deg"] == q
                          and x["pad_thickness_mm"] == t), None)
                r.append("설계 없음" if not x or x["c_pad"] <= 0 else pct(x["c_pad"]))
            rows.append(r)
    S1 = table(["필요 굽힘", "구조", "3 mm", "5 mm", "7 mm", "9 mm"], rows, 2, bold=lambda r: r[1] == "CFL (CoDe)") \
        + source("report/study1/data/thickness_coverage_summary.csv")

    s2 = d["study2_redesign"]
    rows = []
    for q in (90, 120, 150):
        b = [f"{q}°", "<i>fixed-axis 하한</i>"]
        for t in (5, 7, 9):
            x = next(x for x in s2 if x["required_flexion_deg"] == q and x["to_mm"] == t)
            b.append(f'<i>{x["fixed_axis_bound_mm"]:.2f} mm</i>')
        rows.append(b)
        for a in ["CFL", "2R-remote", "2R-DD", "RCJ", "CFL ideal", "RCJ ideal"]:
            r = [f"{q}°", NAME[a]]
            for t in (5, 7, 9):
                x = next((x for x in s2 if x["architecture"] == a and x["required_flexion_deg"] == q
                          and x["to_mm"] == t), None)
                r.append(f'{x["d_contact_mm"]:.2f} mm' if x else "설계 없음")
            rows.append(r)
    S2 = table(["필요 굽힘", "구조", "3 → 5 mm", "3 → 7 mm", "3 → 9 mm"], rows, 2,
               bold=lambda r: r[1] == "CFL (CoDe)") + source("report/study2/data/redesign_summary.csv (L_f 90 mm로 mm 환산)")

    P = []
    for h, label in (("R", "R (접촉면 운동 유지, Δ<sub>contact</sub> 0.86 mm)"), ("U", "U (coverage만, Δ<sub>contact</sub> 3.91 mm)")):
        prs = d["study3_pairs_drake_point"]["hands"][h]
        rows = [[f'{r["rho_mm"]:g} mm', f'{r["q0_deg"]:g}°', DIRECTION.get(r["direction"], r["direction"]),
                 f'{r["O_mu1.3"]:.2f} N', f'{r["O_mu2"]:.2f} N', f'{r["D_O"]:.2f} N',
                 f'{r["H_mu1.3"]:.2f} N', f'{r["H_mu2"]:.2f} N', f'{r["D_H"]:.2f} N', pct(r["E"], 0)] for r in prs]
        es = [r["E"] for r in prs]
        n20 = sum(e <= 0.2 for e in es)
        P.append(f"<h3>{label}</h3>")
        P.append(table(["원기둥 반지름", "쥔 각도 q₀", "하중 방향", "O, μ 1.3", "O, μ 2.0", "Δ(O)",
                        f"{h}, μ 1.3", f"{h}, μ 2.0", f"Δ({h})", "오차"], rows, 3, cls="small"))
        P.append(f'<p class="note">{h}: {len(prs)}개 배치·방향 중 {n20}개({round(100 * n20 / len(prs))}%)에서 오차 20% 이하, '
                 f"중앙값 {pct(statistics.median(es), 0)}.</p>")
    PAIRS = "".join(P) + source("report/study3/data/drake_e21_point_pairs_RU.json",
                                "sim/drake/logs/dk_{O,R,U}_mu{1.3,2}_point-e21_r{15,20}.json (원본)")

    st = d["study3_errors_by_hand"]
    hands = {}
    for s in st.values():
        for r in s["hands"]:
            hands.setdefault(r["hand"], r)
    order = sorted(hands, key=lambda n: hands[n]["d_contact_mm"])

    def by_hand(keys: list[str], labels: list[str]) -> str:
        rows = []
        for n in order:
            x = hands[n]
            r = [n, f'{x["thickness_mm"]:g} mm' if x.get("thickness_mm") else "", f'{x["d_contact_mm"]:.2f} mm',
                 f'{x["d_skel_mm"]:.2f} mm']
            for k in keys:
                v = next((y["E"] for y in st[k]["hands"] if y["hand"] == n), None)
                shade = "" if v is None else \
                    f' style="background:color-mix(in srgb, var(--s1) {round(55 * min(math.log10(max(v * 100, 1)) / math.log10(400), 1))}%, transparent)"'
                r.append((pct(v, 0), shade))
            rows.append(r)
        return table(["손", "pad", "Δcontact", "Δskel"] + labels, rows, 1, cls="small",
                     bold=lambda r: r[0] in ("R", "U")) + source(*[st[k]["source"] for k in keys])

    common = ("한 행이 재설계 손 하나이고, Δ<sub>contact</sub> 순으로 정렬했습니다. 칸은 그 손의 오차 E(배치별 오차의 "
              "중앙값)이고, 진할수록 큽니다. 빈칸은 O와 함께 쥘 수 있는 배치가 없던 손입니다.")
    HANDS = (
        "<h3>표 3-2. 같은 과제(pull-out), 접촉 환경만 바꿈</h3>"
        f'<p class="note">{common} 과제는 표 3-1과 같은 pull-out이고, 시뮬레이터와 접촉 모델만 다릅니다. '
        "표 3-1의 오차 열을 손마다 중앙값으로 줄인 값이어서, R과 U의 Drake point 칸이 표 3-1의 중앙값(19%, 72%)과 같습니다. "
        "Isaac Lab: Newton MuJoCo-Warp, point contact. Drake point: 접촉 강성 2.5 kN/m. Drake hydro: 모든 손에 0.1 MPa, "
        "3 mm 두께의 탄성 pad. 두께만큼 부드러운 pad: 각 pad가 자기 두께만큼 부드럽고, 기준은 같은 pad를 단 O입니다.</p>"
        + by_hand(["isaac_A", "drake_point", "drake_hydro", "soft_pad"],
                  ["Isaac Lab", "Drake point", "Drake hydro", "두께만큼 부드러운 pad"])
        + "<h3>표 3-3. 다른 과제 (Drake point, 같은 명령 재사용)</h3>"
        f'<p class="note">{common} 모든 과제에서 마찰 μ 1.3 → 2.0의 효과를 그 과제의 측정값으로 잽니다. '
        "pull-out: grasp가 풀리는 하중(N, 표 3-2의 Drake point와 같은 열). "
        "grasp 획득: 편 손가락을 O의 명령까지 닫으며 하중을 거스르고 원기둥을 끌어들인 거리(mm). "
        "다이얼: 스프링 다이얼을 한 번 닫으며 돌린 각도(°, O 기준 약 18–42°). "
        "in-hand rolling: 쥔 뒤 굽힘을 8° 또는 15° 더 밀며 원기둥이 구른 각도(°). "
        "grasp 획득에서 끝에 쥐지 못한 경우는 가장 큰 오차로 셉니다.</p>"
        + by_hand(["drake_point", "acquire", "dial", "rolling"], ["pull-out", "grasp 획득", "다이얼", "in-hand rolling"])
        + "<h3>표 3-4. 명령 방식 (Drake point, pull-out)</h3>"
        f'<p class="note">{common} 왼쪽은 O의 관절 setpoint를 그대로 준 경우(표 3-2의 Drake point와 같은 열), '
        "오른쪽은 관절 임피던스 제어는 그대로 두고 setpoint만 손마다 기구학으로 다시 잡은 경우입니다. "
        "손끝 접촉점이 O와 같은 자리에 오도록 했고, 런타임 접촉 정보는 쓰지 않습니다(27차).</p>"
        + by_hand(["drake_point", "contact_setpoint"], ["O의 setpoint 재사용", "접촉 기준 setpoint"])
    )

    rows = [[r["checkpoint"], str(r["eval_seed"]), pct(r["clean_timeout_rate"]), f'{r["signed_rotation_deg_mean"]:.0f}°',
             pct(r["quarter_turn_within_40mm_and_60deg_rate"]), pct(r["half_turn_within_40mm_and_60deg_rate"]),
             pct(r["three_quarter_turn_within_40mm_and_60deg_rate"]), pct(r["full_turn_and_clean_timeout_rate"], 2)]
            for r in d["study4_in_hand_rotation"]]
    S4 = table(["체크포인트", "평가 seed", "20 s 동안 유지", "평균 회전", "1/4 바퀴 이상", "반 바퀴 이상", "3/4 바퀴 이상",
                "한 바퀴"], rows, 1) + source("report/study4/data/{s15,scale_seg1-3}_model_*_rotation_seed*.json")

    page = (OUT / "template.html").read_text()
    for k, v in {"S1": S1, "S2": S2, "PAIRS": PAIRS, "HANDS": HANDS, "S4": S4}.items():
        page = page.replace("{{" + k + "}}", v)
    assert "{{" not in page
    return page


def main() -> int:
    d = build()
    (OUT / "pi_report_data.json").write_text(json.dumps(d, ensure_ascii=False, indent=1))
    (OUT / "pi_report.html").write_text(render(d))
    print(OUT / "pi_report_data.json", OUT / "pi_report.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
