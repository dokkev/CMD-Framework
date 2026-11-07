#!/usr/bin/env python3
"""Compact accommodation figures, generated from saved results and configs."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path[:0]=[str(ROOT),str(HERE.parent)]
import shared
from visualize.style import COLORS,GRID,INK
from core.optimization.mechanism import mechanism

ORDER=['code_matched','2r_dd_tilt0_matched','2r_dd_tilt90_matched','belt_2r_matched','rcj_tilt0_matched','rcj_tilt90_matched','rcj_ideal_matched']
LABEL=['CoDe','DD 0°','DD 90°','Belt','RCJ 0°','RCJ 90°','Ideal RCJ']
TARGETS=['baseline','thick','long','long_thick','asym_long_proximal','asym_long_distal','asym_proximal_heavy','asym_distal_heavy']
SHORT=['baseline','thick','long','long+thick','long prox.','long dist.','prox. heavy','dist. heavy']


def figures(source,out):
    with(source/'summary_best.csv').open()as f:rows=list(csv.DictReader(f))
    out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.spines.top':False,'axes.spines.right':False})
    lookup={(r['target_id'],r['reference_id']):r for r in rows}
    def found(r):return r is not None and r['found']=='True'
    def save(fig,name):
        for ext in ['png','pdf']:fig.savefig(out/f'{name}.{ext}',dpi=240,bbox_inches='tight')
        plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    for col,field in enumerate(['minimum_length_min_mm','delta_compact_baseline_finger_length_mm']):
        data=np.full((len(TARGETS),len(ORDER)),np.nan)
        for i,t in enumerate(TARGETS):
            for j,rid in enumerate(ORDER):
                r=lookup.get((t,rid))
                if found(r)and r.get(field):data[i,j]=float(r[field])
        cmap=plt.get_cmap('viridis').copy();cmap.set_bad('#e4e3df')
        im=axs[col].imshow(data,cmap=cmap,aspect='auto')
        for i in range(len(TARGETS)):
            for j in range(len(ORDER)):
                r=lookup.get((TARGETS[i],ORDER[j]))
                text='not found'if not np.isfinite(data[i,j])else f'{data[i,j]:.1f}'
                color=INK if not np.isfinite(data[i,j])or data[i,j]>np.nanmedian(data)else 'white'
                axs[col].text(j,i,text,ha='center',va='center',fontsize=7,color=color)
        axs[col].set_xticks(range(7),LABEL,rotation=35,ha='right')
        axs[col].set_yticks(range(8),SHORT)
        axs[col].axvline(5.5,color=INK,lw=1)
        fig.colorbar(im,ax=axs[col],shrink=.8,label='mm')
    axs[0].set_title('Shortest nominal length found (mm)')
    axs[1].set_title('Selected length change from own compact baseline (mm)')
    fig.suptitle('Given interface, common 120° ROM; ideal RCJ separate at right; no global minimum certificate')
    save(fig,'compact_lengths')
    fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True,layout='constrained')
    for k,(field,title)in enumerate([('proximal_span_mm','Proximal contact-bearing span'),('distal_span_mm','Distal contact-bearing span'),('articulation_mm','Articulation station')]):
        for j,rid in enumerate(ORDER):
            points=[(i,lookup[(t,rid)])for i,t in enumerate(TARGETS)if found(lookup.get((t,rid)))]
            if not points:continue
            mech=points[0][1]['mechanism']
            x=np.array([p[0]for p in points])+(j-3)*.09
            y=np.array([float(r[f'delta_{field}'])for _,r in points])
            axs[k].scatter(x,y,label=LABEL[j],color=COLORS[mech],marker=['o','s','D','v','^','P','x'][j],s=30,
                facecolors='none'if j in [1,4]else None)
        axs[k].axhline(0,color=INK,lw=.8);axs[k].grid(axis='y',color=GRID)
        axs[k].set_ylabel('Change from 90 mm reference (mm)');axs[k].set_title(title)
    axs[-1].set_xticks(range(8),SHORT,rotation=20,ha='right')
    handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=7)
    save(fig,'compact_morphology_changes')
    # Show optimizer spread as individual seed minima, not statistical confidence.
    with(source/'summary.csv').open()as f:seeds=list(csv.DictReader(f))
    fig,axs=plt.subplots(2,4,figsize=(12,6),layout='constrained')
    for ax,t,title in zip(axs.flat,TARGETS,SHORT):
        for j,rid in enumerate(ORDER):
            vals=[float(r['minimum_length_mm'])for r in seeds if r['target_id']==t and r['reference_id']==rid and r['found']=='True']
            ax.scatter(j+np.linspace(-.09,.09,len(vals)),vals,s=20,color=COLORS[lookup[(t,rid)]['mechanism']])
        ax.axhline(90,color=INK,lw=.7,ls=':');ax.set_title(title)
        ax.set_xticks(range(7),LABEL,rotation=60,ha='right',fontsize=7);ax.grid(axis='y',color=GRID)
        ax.set_ylabel('Primary search length (mm)')
    fig.suptitle('Three optimizer seeds; dotted line = matched 90 mm reference; missing points = not found')
    save(fig,'compact_seed_spread')
    # Native geometry renderer, with saved configs. Common scale and pose.
    from visualize.finger.view import view_of
    refs={r.reference_id:r for r in shared.load_references()}
    show=[rid for rid in ['code_matched','2r_dd_tilt0_matched','2r_dd_tilt90_matched','belt_2r_matched','rcj_tilt90_matched']if found(lookup.get(('long_thick',rid)))]
    if show:
        fig,axs=plt.subplots(len(show),3,figsize=(12,2.4*len(show)),squeeze=False,layout='constrained')
        for i,rid in enumerate(show):
            ref=refs[rid]
            for j,(t,tag)in enumerate([(None,'Matched reference, 20×3 mm'),('baseline','Compact baseline, 20×3 mm'),('long_thick','Compact target, 35×8 mm')]):
                if t is None:
                    from core.optimization.analysis.tradeoff import with_modules
                    cfg=with_modules(ref.config,shared.catalog()['baseline'].modules);x=ref.x;length=90.
                else:
                    r=lookup[(t,rid)];cfg=mechanism(ref.mechanism).config_from_json(json.loads(r['final_config']))
                    x=json.loads(r['x']);length=float(r['final_length_mm'])
                view_of(ref.mechanism,cfg,x).draw(axs[i,j],0,style='sketch')
                axs[i,j].set_aspect('equal');axs[i,j].set_xlim(-20,135);axs[i,j].set_ylim(-25,35)
                axs[i,j].axis('off');axs[i,j].set_title(f'{rid}: {tag}\nL={length:.1f} mm',fontsize=8)
        fig.suptitle('Same scale, extended pose; reference vs compact designs; geometry from saved model configs')
        save(fig,'compact_geometry')
    inputs={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [source/'summary.csv',source/'summary_best.csv',source/'manifest.json',source/'aggregation.json']}
    (out/'manifest.json').write_text(json.dumps(dict(inputs_sha256=inputs,source=shared.git_commit(out),
        figures='saved results; best-of-three search incumbents, no global optimality claim'),indent=2))
    print(f'wrote figures to {out}')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=ROOT/'output/study/study2_compact_morphology')
    p.add_argument('--output',type=Path,default=ROOT/'report/figures/study2_compact')
    a=p.parse_args();figures(a.source,a.output)
