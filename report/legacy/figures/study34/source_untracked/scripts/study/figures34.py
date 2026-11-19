#!/usr/bin/env python3
"""Study 3/4 figures and summaries, reading saved CSV/JSON only."""
from pathlib import Path
import csv
import hashlib
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(HERE)]
from visualize.style import COLORS, SHORT_LABELS, GRID, INK
import shared

S3 = ROOT/'output/study/study3_transmission'
S4 = ROOT/'output/study/study4_joint_interface'
OUT = ROOT/'report/figures/study34'
KINDS = ['linkage','revolute','revolute_parallel','rcj','rcj_idealized']
MATCHED = ['code_matched','2r_dd_tilt90_matched','belt_2r_matched','rcj_tilt90_matched']
inputs = {}


def read(path):
    inputs[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    if path.suffix == '.json':
        return json.loads(path.read_text())
    with path.open() as f:
        return list(csv.DictReader(f))


def number(rows,key):
    return np.array([float(r[key]) for r in rows])


def save(fig,name):
    OUT.mkdir(parents=True,exist_ok=True)
    for suffix in ['png','pdf']:
        fig.savefig(OUT/f'{name}.{suffix}',dpi=240,bbox_inches='tight')
    plt.close(fig)


def main():
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,
                         'axes.spines.right':False,'axes.labelcolor':INK})
    rows=read(S3/'reflection.csv'); acceptance=read(S3/'acceptance.csv')
    manifest=read(S3/'manifest.json')
    designs=read(S3/'designs.csv')
    bridge=read(S4/'screen.csv'); paths=read(S4/'paths.csv'); read(S4/'manifest.json')
    fig,axs=plt.subplots(2,2,figsize=(10,6.5),sharex=True,layout='constrained')
    for col,module in enumerate(['baseline','thick']):
        for i,rid in enumerate(MATCHED):
            data=[r for r in rows if r['candidate_id']==rid and r['module_id']==module
                  and r['side']=='distal' and r['abstraction']=='point' and r['contact_model']=='normal_only']
            if not data: continue
            k=data[0]['mechanism']
            for ax,metric in zip(axs[:,col],['K11','K01']):
                ax.plot(number(data,'phi_deg'),number(data,metric),color=COLORS[k],label=SHORT_LABELS[k],
                        marker=['o','s','v','^'][i],markersize=4,linestyle='--' if k=='revolute_parallel' else '-')
        axs[0,col].set_title(f'{module}: identical distal point spring')
        axs[1,col].set_xlabel('Distal flexion (deg)')
    axs[0,0].set_ylabel(r'$K_{a,11}$: PIP servo stiffness (N m/rad)')
    axs[1,0].set_ylabel(r'$K_{a,01}$: MCP–PIP coupling (N m/rad)')
    for ax in axs.flat: ax.grid(color=GRID)
    handles, labels = axs[0,0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=4, fontsize=9)
    fig.suptitle('Matched morphology, feasible through 120°; output shafts; zero preload')
    save(fig,'study3_matched')
    # Each point is ONE design's mean over the common pose grid, never a replicate.
    means=[]
    for c in designs:
        if c['group']!='adaptation_front_cover':continue
        for module in ['baseline','thick','long']:
            for abstraction in ['point','distributed']:
                data=[r for r in rows if r['candidate_id']==c['candidate_id'] and r['module_id']==module
                      and r['side']=='distal' and r['contact_model']=='normal_only' and r['abstraction']==abstraction]
                if data: means.append(dict(candidate_id=c['candidate_id'],mechanism=c['mechanism'],module_id=module,
                    abstraction=abstraction,mean_K_trace_normalized=float(number(data,'K_trace_normalized').mean())))
    shared.write_csv(S3/'design_summary.csv',means)
    fig,axs=plt.subplots(1,3,figsize=(12,4),sharey=True,layout='constrained')
    for ax,module in zip(axs,['baseline','thick','long']):
        for j,k in enumerate(KINDS):
            for abstraction,offset,marker in [('point',-.12,'o'),('distributed',.12,'x')]:
                data=[r['mean_K_trace_normalized']for r in means if r['mechanism']==k and
                      r['module_id']==module and r['abstraction']==abstraction]
                data=np.sort(data)
                # Deterministic spread to expose alternatives, not estimated noise.
                x=j+offset+np.linspace(-.07,.07,len(data))
                ax.scatter(x,data,color=COLORS[k],marker=marker,s=15,alpha=.7,
                           label=abstraction if j==0 else None)
            attempted=[r for r in acceptance if r['group']=='adaptation_front_cover' and r['mechanism']==k and r['module_id']==module]
            n=sum(r['feasible']=='True'for r in attempted)
            ax.text(j,1.01,f'{n}/{len(attempted)}',transform=ax.get_xaxis_transform(),ha='center',fontsize=8)
        ax.axvspan(3.5,4.5,color='#eef5f1',zorder=-1)
        ax.set_xticks(range(5),['CoDe','DD','Belt','RCJ','Ideal RCJ'],rotation=30,ha='right')
        ax.set_title(module,pad=27);ax.grid(axis='y',color=GRID)
    axs[0].set_ylabel(r'Mean over poses: tr($K_a$) / ($k_n L^2$)')
    axs[0].legend(fontsize=9)
    fig.suptitle('Re-evaluated adaptation-front cover; accepted/attempted above; ideal bound shaded',y=1.08)
    save(fig,'study3_design_distributions')
    fig,axs=plt.subplots(1,5,figsize=(13,3.2),layout='constrained')
    for ax,spec in zip(axs,manifest['fronts']):
        archived=read(S3/'inputs'/spec['id']/'pareto.csv')
        k=next(c['mechanism']for c in designs if c['candidate_id'].startswith(spec['id']+'__'))
        ax.scatter(number(archived,'pad_coverage'),number(archived,'torque_ratio_max'),s=3,color=COLORS[k],alpha=.25)
        chosen=[archived[i]for i in spec['selected_csv_indices']]
        ax.scatter(number(chosen,'pad_coverage'),number(chosen,'torque_ratio_max'),s=16,facecolors='none',edgecolors=INK)
        ax.set_title(f'{SHORT_LABELS[k]}\n{len(archived)} archived / {len(chosen)} sampled',fontsize=9)
        ax.set_xlabel('Original coverage');ax.grid(color=GRID)
    axs[0].set_ylabel('Original max transmission ratio')
    fig.suptitle('Full archived Study 2 long-target fronts; circles = selected proposals, before re-evaluation',fontsize=11)
    save(fig,'study3_archived_fronts')
    fig,axs=plt.subplots(2,3,figsize=(12,6),sharex=True,layout='constrained')
    for col,depth in enumerate([.005,.008,.012]):
        for rid,k in [('code_matched','linkage'),('belt_2r_matched','revolute_parallel')]:
            data=[r for r in bridge if r['reference_id']==rid and float(r['depth_m'])==depth]
            x=number(data,'phi_deg')
            axs[0,col].plot(x,100*number(data,'arc_change_fraction'),color=COLORS[k],label=SHORT_LABELS[k]+' tangent path')
            axs[0,col].plot(x,100*number(data,'chord_change_fraction'),color=COLORS[k],ls=':',label=SHORT_LABELS[k]+' chord bound')
            axs[1,col].plot(x,number(data,'projected_hardware_gap_mm'),color=COLORS[k],label=SHORT_LABELS[k])
        axs[0,col].set_title(f'Common neutral depth {depth*1000:g} mm')
        axs[1,col].axhline(0,color=INK,lw=.7)
        axs[1,col].set_xlabel('Distal flexion (deg)')
    axs[0,0].set_ylabel('Path-length change from neutral (%)')
    axs[1,0].set_ylabel('Projected hardware gap (mm)')
    axs[0,0].legend(fontsize=8)
    for ax in axs.flat:ax.grid(color=GRID)
    fig.suptitle('Joint bridge screening: prescribed paths, hypothetical skin datums; not material strain')
    save(fig,'study4_screening')
    fig,axs=plt.subplots(1,2,figsize=(10,4),sharex=True,sharey=True,layout='constrained')
    for ax,rid in zip(axs,['code_matched','belt_2r_matched']):
        for phi in [0,30,60,90,120]:
            data=[r for r in paths if r['reference_id']==rid and float(r['depth_m'])==.008 and float(r['phi_deg'])==phi]
            ax.plot(1000*number(data,'x_m'),1000*number(data,'z_m'),label=f'{phi}°')
            ax.scatter([float(data[0]['x_m'])*1000,float(data[-1]['x_m'])*1000],
                       [float(data[0]['z_m'])*1000,float(data[-1]['z_m'])*1000],s=10)
        ax.set_title(rid);ax.set_xlabel('X (mm)');ax.set_aspect('equal');ax.grid(color=GRID)
    axs[0].set_ylabel('Z (mm), dorsal positive');axs[1].legend(fontsize=8)
    fig.suptitle('Same neutral 16.5 mm bridge, depth 8 mm; tangent-path geometry only')
    save(fig,'study4_paths')
    summary=dict(study3_designs=len(designs),study3_rows=len(rows),
        accepted_by_group={g:sum(r['feasible']=='True' for r in acceptance if r['group']==g) for g in ['matched','adaptation_front_cover']},
        max_fd_error=manifest['max_fd_error_m_per_rad'],
        study4_self_intersecting_rows=sum(r['path_self_intersects']=='True'for r in bridge))
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2))
    (OUT/'manifest.json').write_text(json.dumps(dict(inputs_sha256=inputs,source=shared.git_commit(OUT),
        script='scripts/study/figures34.py',design_distribution='one mean per design, no inferential confidence intervals',
        note='Normal discontinuity column is support-normal mismatch; the constructed bridge is endpoint tangent-continuous'),indent=2))
    print(json.dumps(summary))


if __name__=='__main__':main()
