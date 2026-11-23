#!/bin/bash
# Experiment 21: every hand, point (replica) then hydroelastic, with the retry rule
H="O R U h3to5_hr h3to5_hu h3to9_hr h3to9_hu d1p5_minDs_s0 d2p25_minDs_s0 d2p25_maxDs_s0 d3_minDs_s0 d4p5_s0 d5p5_s0 d1p5_maxDs_s0 d3_maxDs_s0 h3to5_d3_s0 h3to5_d3_minDs_s0 h3to9_d3p5_maxDs_s0 h3to9_d5_minDs_s0"
./run_batch.sh point e21 $H
./run_batch.sh hydro e21 $H
