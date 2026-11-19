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
