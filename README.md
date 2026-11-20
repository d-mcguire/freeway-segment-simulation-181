
# freeway-segment-simulation-181
CEE 181 project: 3-mile single-lane freeway, IDM car-following, metered on-ramp
at mile 2.5, connected vehicles (ACC) at 0/20/50/70 % penetration, braking disturbance.
## Run
    pip install -r requirements.txt
    python verify_idm.py                                   # IDM checks
    python run_simulation.py --quick                       # ~10 s smoke test
    python run_simulation.py --seeds 1 2 3 4 5 --ramp-control fixed alinea --trajectories
    python analysis.py                                     # tables + figures in results/
Python 3.9+
## Files
| file | role |
|---|---|
| idm.py, verify_idm.py | IDM model and its numerical checks |
| platoon_experiment.py | closed platoon braking experiment (0/20/50/70 % CV) |
| ramp_simulation.py | mainline entry, ramp merge gap acceptance (reused by run_simulation.py) |
| detectors.py | virtual detectors (30 s intervals; time-mean and space-mean speed) |
| observations.py | detector defects, state estimate, 60 s flow forecasts |
| controller.py | ramp meters (none / fixed / ALINEA-style) and the ACC controller |
| run_simulation.py | integrated simulation, one folder of logs per scenario and seed |
| analysis.py | aggregation across seeds, paired differences, figures |
## Outputs
results/runs_summary.csv (one row per run), results/<scenario>/seed_<n>/*.csv, results/figures/*.png
## Known limitations
See the notes in the design record: merge model gives the mainline priority,
placeholder controller gains and ACC bounds, one lane, deterministic IDM drivers.
## Reproducing the mid-report figures and tables
Run the following commands **from the repository root** using Python 3.9 or later. These commands reproduce the outputs from the code in this repository using the mid-report's 3-mile, single-lane freeway, 2,000 veh/h mainline demand, 400 veh/h ramp demand, fixed ramp metering, four CV penetration levels, five seeds, 1,200-second runs, 300-second warm-up, and a braking event at 600 seconds. Run them in order. The commands use separate output folders to avoid mixing braking and no-braking runs.
### 1. Install dependencies and verify IDM
```bash
python3 -m pip install -r requirements.txt
python3 verify_idm.py
```
### 2. Preliminary platoon experiment — Figures 1–4
```bash
python3 platoon_experiment.py
```
| Report figure | Generated file |
|---|---|
| Figure 1: 0% CV speed propagation | `platoon_results/speeds_00pct.png` |
| Figure 2: 20% CV speed propagation | `platoon_results/speeds_20pct.png` |
| Figure 3: 50% CV speed propagation | `platoon_results/speeds_50pct.png` |
| Figure 4: 70% CV speed propagation | `platoon_results/speeds_70pct.png` |
`platoon_results/summary.csv` contains the minimum-gap results for the four scenarios; `platoon_results/trajectories_XXpct.csv` contains vehicle-level time histories. The script also prints the 0.1 s versus 0.05 s time-step comparison.
### 3. Full freeway experiment — 20 braking runs
```bash
python3 run_simulation.py \
  --penetrations 0 0.2 0.5 0.7 \
  --seeds 1 2 3 4 5 \
  --ramp-control fixed \
  --mainline-demand 2000 \
  --ramp-demand 400 \
  --duration 1200 \
  --warmup 300 \
  --disturbance-time 600 \
  --disturbance-mile 2.0 \
  --ramp-entry match_leader \
  --out results_with_braking
python3 analysis.py --results results_with_braking
```
This generates `results_with_braking/runs_summary.csv` (one row per run), `results_with_braking/analysis_summary.csv` (mean and variability across seeds), `results_with_braking/paired_differences.csv` (paired comparisons with 0% CV), and `results_with_braking/figures/`.
### 4. No-braking comparison — 20 more runs
```bash
python3 run_simulation.py \
  --penetrations 0 0.2 0.5 0.7 \
  --seeds 1 2 3 4 5 \
  --ramp-control fixed \
  --mainline-demand 2000 \
  --ramp-demand 400 \
  --duration 1200 \
  --warmup 300 \
  --disturbance-time 600 \
  --disturbance-mile 2.0 \
  --ramp-entry match_leader \
  --no-disturbance \
  --out results_without_braking
python3 analysis.py --results results_without_braking
python3 compare_disturbance.py \
  --with-braking results_with_braking \
  --without-braking results_without_braking \
  --out disturbance_comparison
```
`disturbance_comparison/` contains the paired detector comparison CSVs. The detector measurements are aggregated over 30-second intervals, so they do not fully resolve a 3-second braking event.
### 5. Mapping the full freeway outputs to the report
| Report item | Source file and columns | How to reproduce the report item |
|---|---|---|
| Table 4: throughput and delay | `results_with_braking/analysis_summary.csv`: `exit_rate_vph_window`, `mean_delay_s_per_request` | Filter each metric and report the `mean` for the four CV scenarios. **Use seconds/request for the delay column, not `total_delay_veh_h`.** |
| Table 5: disturbance propagation | Same summary: `dist_n_affected`, `dist_mean_dip_mps` | Filter each metric and report the scenario means. |
| Table 6: disturbance recovery | `analysis_summary.csv`: `dist_recovery_fraction`; `runs_summary.csv`: `dist_n_recovered` | Report mean recovery fractions from the analysis summary. Compute the mean `dist_n_recovered` across the five seeds for each CV penetration; this metric is not included in `analysis_summary.csv`. Multiply recovery fractions by 100 for percentages. |
| Figure 5: throughput vs CV penetration | `analysis_summary.csv`: `exit_rate_vph_window` | Plot the four scenario means (the report-specific standalone figure is not created automatically). |
| Figure 6: delay vs CV penetration | `analysis_summary.csv`: `mean_delay_s_per_request` | Plot the four scenario means (standalone report figure not created automatically). |
| Figure 7: affected followers vs CV penetration | `analysis_summary.csv`: `dist_n_affected` | Plot the four scenario means (standalone report figure not created automatically). |
| Figure 8: recovery fraction vs CV penetration | `analysis_summary.csv`: `dist_recovery_fraction` | Plot the four scenario means, expressed as percentages (standalone report figure not created automatically). |
| Figure 9: multi-panel metrics across CV penetration | `results_with_braking/figures/metrics_vs_penetration.png` | Generated directly by `analysis.py`; includes individual seeds and mean ± 1 standard deviation. |
| Braking vs no-braking comparison discussion | `disturbance_comparison/` CSVs | Generated by `compare_disturbance.py`. |

**Important:** Figures 5–8 in the report are created based on the analysis CSVs. The current `analysis.py` creates a combined six-panel figure (`metrics_vs_penetration.png`) and other diagnostic plots, but does **not** generate those four standalone report figures under their report figure numbers. Recreating their exact formatting requires the original plotting steps. The commands above reproduce their underlying numerical data.

