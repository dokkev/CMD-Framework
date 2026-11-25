"""E = |dH - dO|/|dO| (mu 1.3 -> 2.0), noise sum = (|seed diff of dR| + |seed diff of dU|)/|dO|; transfer ratios at mu 1.3.
usage: emetric.py <pattern with {H},{MU},{S}> O R U [R5 U5 ...]   (pairs after O)"""
import json, sys
pat = sys.argv[1]; hands = sys.argv[2:]
M = {"rot": "signed_rotation_deg_mean", "goals": "goals_per_episode_mean",
     "full30": "full_turn_within_40mm_and_upright30deg_rate", "fullclean": "full_turn_and_clean_timeout_rate"}
def g(h, mu, s, k):
    return json.load(open(pat.format(H=h, MU=mu, S=s)))[M[k]]
S = (94000, 95000)
def d(h, k): return [g(h, "2.0", s, k) - g(h, "1.3", s, k) for s in S]
O = hands[0]
for k in M:
    try:
        dO = d(O, k); dOm = sum(dO) / 2
        print(f"== {k}: O mu1.3 = {[round(g(O,'1.3',s,k),3) for s in S]}, dO = {[round(x,3) for x in dO]} (mean {dOm:.3f}; seed diff {abs(dO[0]-dO[1]):.3f})")
        for i in range(1, len(hands), 2):
            R, U = hands[i], hands[i + 1]
            dR, dU = d(R, k), d(U, k)
            ER = abs(sum(dR)/2 - dOm) / abs(dOm); EU = abs(sum(dU)/2 - dOm) / abs(dOm)
            noise = (abs(dR[0]-dR[1]) + abs(dU[0]-dU[1])) / abs(dOm)
            rr = sum(g(R,'1.3',s,k) for s in S)/sum(g(O,'1.3',s,k) for s in S)
            ru = sum(g(U,'1.3',s,k) for s in S)/sum(g(O,'1.3',s,k) for s in S)
            rsd = [abs(g(H,'1.3',94000,k)/g(O,'1.3',94000,k) - g(H,'1.3',95000,k)/g(O,'1.3',95000,k)) for H in (R,U)]
            verdict = "R smaller beyond noise" if EU-ER > noise else ("U smaller beyond noise" if ER-EU > noise else "indistinguishable")
            print(f"  {R}/{U}: dR={sum(dR)/2:.3f} dU={sum(dU)/2:.3f} E(R)={ER:.3f} E(U)={EU:.3f} noise={noise:.3f} -> {verdict}; "
                  f"ratio@1.3 R={rr:.3f} U={ru:.3f} (seed diff of ratio {rsd[0]:.3f}/{rsd[1]:.3f})")
    except FileNotFoundError as e:
        print(k, "missing", e.filename)
