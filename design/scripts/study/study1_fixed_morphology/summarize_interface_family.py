#!/usr/bin/env python3
"""Summarize only saved interface-family results; no optimizer or geometry evaluation."""
from __future__ import annotations

import csv
import hashlib
import json
import shutil
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import run as study1

ROOT = study1.REPOSITORY_ROOT
OUT = ROOT / "output/study/interface_family_2026-11-09"


def anchored_family(grid, thickness):
    """Largest sampled all-feasible rectangle anchored at 3 mm on both axes.

    Maximize physical parameter area, then number of sampled combinations. No interpolation.
    """
    start = thickness.index(3)
    best = (0, 0, 0, 0)
    for i in range(start, len(thickness)):
        for j in range(start, len(thickness)):
            if grid[start:i+1, start:j+1].all():
                score = ((thickness[i]-3)*(thickness[j]-3), (i-start+1)*(j-start+1),
                         thickness[i], thickness[j])
                best = max(best, score)
    common = 0
    for i in range(start, len(thickness)):
        if grid[start:i+1, start:i+1].all():
            common = thickness[i]
    return best, common


def main():
    manifest = json.loads((OUT / "manifest.json").read_text())
    assert manifest["status"] == "complete"
    raw = OUT / "assembly_results.csv"
    assert hashlib.sha256(raw.read_bytes()).hexdigest() == manifest["results_sha256"]
    with raw.open() as f:
        rows = list(csv.DictReader(f))
    refs = list(dict.fromkeys(r["reference_id"] for r in rows))
    variants = {(r["reference_id"], r["variant"]) for r in rows}
    cfg = manifest["config"]
    thickness = cfg["thickness_mm"]
    lookup = {(r["reference_id"], r["variant"], float(r["proximal_length_mm"]),
               float(r["distal_length_mm"]), float(r["proximal_thickness_mm"]),
               float(r["distal_thickness_mm"]), float(r["required_flexion_deg"])): r for r in rows}
    assert len(lookup) == len(rows) == manifest["evaluations"]
    summary, failures, maps = [], [], {}
    for rid in refs:
        for rom in cfg["required_flexion_deg"]:
            conditioned = f"conditioned_{rom}" if (rid, f"conditioned_{rom}") in variants else "nominal"
            for protocol, variant in [("rom_conditioned", conditioned), ("nominal_control", "nominal")]:
                for lp, ld in cfg["length_pairs_mm"]:
                    endpoint = np.zeros((len(thickness), len(thickness)), dtype=bool)
                    sampled = endpoint.copy()
                    causes = Counter()
                    for i, tp in enumerate(thickness):
                        for j, td in enumerate(thickness):
                            end = lookup[(rid, variant, lp, ld, tp, td, rom)]
                            endpoint[i,j] = end["feasible"] == "True"
                            selected = [lookup[(rid, variant, lp, ld, tp, td, phi)]
                                        for phi in cfg["sampled_flexion_deg"] if phi <= rom]
                            sampled[i,j] = all(r["feasible"] == "True" for r in selected)
                            if not endpoint[i,j]:
                                causes.update(end["failure"].split(";"))
                    for mode, grid in [("endpoint", endpoint), ("sampled_postures", sampled)]:
                        best, common = anchored_family(grid, thickness)
                        summary.append(dict(reference_id=rid, protocol=protocol, variant=variant,
                                            rom=rom, proximal_length_mm=lp, distal_length_mm=ld,
                                            criterion=mode, feasible_pairs=int(grid.sum()), total_pairs=grid.size,
                                            sampled_family_tp_max=best[2], sampled_family_td_max=best[3],
                                            sampled_family_pairs=best[1], common_thickness_max=common,
                                            endpoint_losses_at_sampled_postures=int((endpoint & ~sampled).sum())))
                        if protocol == "rom_conditioned":
                            maps[(rid, rom, lp, ld, mode)] = grid
                    failures.extend(dict(reference_id=rid, protocol=protocol, rom=rom,
                                         proximal_length_mm=lp, distal_length_mm=ld,
                                         constraint=name, failed_pairs=count) for name,count in causes.items())
    study1.shared.write_csv(OUT / "summary.csv", summary)
    study1.shared.write_csv(OUT / "failure_counts.csv", failures)
    # 20 mm is an illustrative stratum; the CSV and length plot include all six declared strata.
    fig, axes = plt.subplots(3, len(refs), figsize=(18, 8), constrained_layout=True)
    names = ["CoDe 15°", "DD 0°", "DD 90°", "Belt 15°", "RCJ 0°", "RCJ 90°", "Ideal RCJ"]
    for c, (rid, name) in enumerate(zip(refs, names)):
        for r, rom in enumerate(cfg["required_flexion_deg"]):
            end = maps[rid,rom,20,20,"endpoint"]
            sampled = maps[rid,rom,20,20,"sampled_postures"]
            ax = axes[r,c]
            ax.imshow(end.astype(int)+sampled.astype(int), origin="lower", vmin=0, vmax=2,
                      cmap=matplotlib.colors.ListedColormap(["#eeeeee", "#efb15d", "#197a78"]))
            ticks = [0,2,5,8,11]
            ax.set_xticks(ticks, [thickness[i] for i in ticks])
            ax.set_yticks(ticks, [thickness[i] for i in ticks])
            if r == 0:
                ax.set_title(name)
            if c == 0:
                ax.set_ylabel(f"ROM {rom}°\nProximal thickness (mm)")
            if r == 2:
                ax.set_xlabel("Distal thickness (mm)")
            ax.text(.03,.95,f"{end.sum()}/{end.size}",transform=ax.transAxes,va="top",fontsize=8,
                    bbox=dict(facecolor="white",alpha=.8,edgecolor="none"))
    fig.suptitle("Fixed skeleton/actuation, 20 mm pads: grey fail; orange endpoint only; teal sampled-posture pass\n"
                 "ROM-conditioned baselines; interface-dependent seating; sampled thickness bins are not equally spaced")
    fig.savefig(OUT / "thickness_maps.png", dpi=180)
    fig.savefig(OUT / "thickness_maps.pdf")
    plt.close(fig)
    fig, axes = plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True)
    for ax, rom in zip(axes,cfg["required_flexion_deg"]):
        for rid,name in zip(refs,names):
            data = [s for s in summary if s["reference_id"]==rid and s["rom"]==rom
                    and s["protocol"]=="rom_conditioned" and s["criterion"]=="endpoint"
                    and s["proximal_length_mm"]==s["distal_length_mm"]]
            ax.plot([s["proximal_length_mm"] for s in data], [s["feasible_pairs"] for s in data],
                    marker="o",label=name,linestyle="--" if "ideal" in rid else "-")
        ax.set(title=f"ROM {rom}°",xlabel="Each pad length (mm)",ylim=(-3,147),xticks=[15,20,25,30])
        ax.grid(alpha=.2)
    axes[0].set_ylabel("Supported thickness combinations / 144")
    axes[-1].legend(fontsize=8)
    fig.suptitle("Fixed-H family support across declared length strata — endpoint criterion, not manipulation performance")
    fig.savefig(OUT / "length_sensitivity.png",dpi=180)
    fig.savefig(OUT / "length_sensitivity.pdf")
    plt.close(fig)
    dense_summary = []
    additional_inputs = {}
    for folder in ["dense_postures", "dense_fold_off"]:
        path = OUT / folder / "assembly_results.csv"
        if not path.exists():
            continue
        saved = json.loads((path.parent / "manifest.json").read_text())
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert saved["status"] == "complete" and digest == saved["results_sha256"]
        additional_inputs[folder] = digest
        with path.open() as f:
            dense = list(csv.DictReader(f))
        dense_lookup = {(r["reference_id"],float(r["proximal_thickness_mm"]),
                         float(r["distal_thickness_mm"]),float(r["required_flexion_deg"])): r for r in dense}
        assert len(dense_lookup) == len(dense) == 30240
        for rid in refs:
            grid = np.array([[all(dense_lookup[rid,tp,td,phi]["feasible"] == "True"
                                  for phi in range(5,151,5)) for td in thickness] for tp in thickness])
            best, common = anchored_family(grid, thickness)
            dense_summary.append(dict(reference_id=rid, protocol=folder, feasible_pairs=int(grid.sum()),
                                      total_pairs=grid.size, sampled_family_tp_max=best[2],
                                      sampled_family_td_max=best[3], sampled_family_pairs=best[1],
                                      common_thickness_max=common))
    if dense_summary:
        study1.shared.write_csv(OUT / "dense_summary.csv",dense_summary)
        fig, ax = plt.subplots(figsize=(12,5),constrained_layout=True)
        x = np.arange(len(refs))
        for offset, (mode, label) in enumerate([
                ("endpoint", "150° endpoint"), ("sampled_postures", "6 sampled poses"),
                ("dense_postures", "30 poses, 5° steps"), ("dense_fold_off", "30 poses, fold rule OFF control")]):
            if mode.startswith("dense"):
                selected = {s["reference_id"]:s for s in dense_summary if s["protocol"]==mode}
            else:
                selected = {s["reference_id"]:s for s in summary if s["protocol"]=="rom_conditioned"
                            and s["rom"]==150 and s["proximal_length_mm"]==s["distal_length_mm"]==20
                            and s["criterion"]==mode}
            if selected:
                ax.bar(x+(offset-1.5)*.2,[selected[rid]["feasible_pairs"] for rid in refs],width=.2,label=label)
        ax.set(xticks=x,xticklabels=names,ylabel="Supported thickness pairs / 144",ylim=(0,150),
               title="20 mm pads on frozen 150° baselines: posture sampling and fold-rule sensitivity")
        ax.legend(fontsize=9)
        ax.grid(axis="y",alpha=.2)
        fig.savefig(OUT / "posture_sensitivity.png",dpi=180)
        fig.savefig(OUT / "posture_sensitivity.pdf")
        plt.close(fig)
    source = OUT / "analysis_source"
    source.mkdir(exist_ok=True)
    shutil.copy2(__file__, source / Path(__file__).name)
    record = {"input_sha256": manifest["results_sha256"],
              "additional_input_sha256": additional_inputs,
              "analysis_source": study1.shared.git_commit(source),
              "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "scope": "Design grid counts, not replicates or continuous ranges; no physical validation."}
    (OUT / "analysis_manifest.json").write_text(json.dumps(record,indent=2)+"\n")
    print(f"wrote summaries and figures from {len(rows)} saved evaluations")


if __name__ == "__main__":
    main()
