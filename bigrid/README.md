# Model checking the FEDT reaction rules

The bigraphical reactive system is exported to a DTMC with **BigraphER** and checked with **PRISM**.
This verifies the reaction-rule set itself, independently of the running ROS 2 system.

## Requirements

| Tool | Version | Install |
| --- | --- | --- |
| BigraphER | 2.0.0 | [opam instructions](https://www.dcs.gla.ac.uk/~michele/bigrapher.html#opam) |
| PRISM | 4.9 | [prismmodelchecker.org](https://www.prismmodelchecker.org/download.php) |
| Python | 3.x | used for post-processing inside the scripts |

`bigrapher`, `prism`, and `python3` must be on your `PATH`. The scripts are bash-only and use
relative paths, so run them from inside this directory on Linux, macOS, or WSL.

## Files

| File | Description |
| --- | --- |
| `uavs_3d_verified_2x2x2.big` | Reduced configuration: 2×2×2 bi-grid (8 locales), 2 UAS, 1 charging station. Small enough to explore exhaustively. |
| `uavs_in_3d_bigrid.big` | Main configuration: 3×3×3 bi-grid (27 locales), 3 UAS, 2 charging stations, 1 obstacle. Explored up to a state bound. |
| `verification_queries_verified.props` | PCTL queries for the exhaustive run. |
| `verification_queries.props` | PCTL queries for the bounded run. |
| `verification_results/` | Recorded output of both analyses. |

Both models share one signature and the same 28 reaction-rule schemas, grouped into four priority
classes: charging and take-off, emergency landing, emergency descent, then nominal movement,
landing, and state changes.

## Running

```bash
cd bigrid

# Exhaustive verification of the 2x2x2 configuration -> verification_results/verified/
bash run_verified_analysis.sh

# Bounded exploration of the 3x3x3 configuration, for state bound M
bash verification_analysis.sh 500
bash verification_analysis.sh 1000
bash verification_analysis.sh 2000      # default if no argument is given
```

## What is checked

| Query | Property | Meaning |
| --- | --- | --- |
| Q1 | `P=? [G !"collision"]` | Safety: no two UAS ever occupy the same grid cell. |
| Q2.1–Q2.7 | `P>0 [F …]` / `P=? [F …]` | Reachability of the nominal landing chain and of each fail-safe path (low battery, empty battery, communication loss, compound fault, reaching a charging station, starting to charge). |
| Q3 | `P=? [G !"drone_landed_in_air"]` | Consistency: no UAS is reported as landed while located at an air locale. |
| Q4 | `P>0 [F !"drone_flying"]` | Progress: a configuration in which all UAS have landed is reachable. |
| Q5 | — | Deadlock freedom, checked structurally by `run_verified_analysis.sh` rather than as a PCTL query. |

Occupancy exclusion is structural rather than probabilistic: every movement rule requires the
destination locale to be provably empty (`OccupiedBy.1`), which also prevents entry into cells
holding an obstacle.

## Two things to know

- **`verification_analysis.sh` reuses an existing state-space export.** If
  `verification_results/<M>/uavs_3d.tra` and `.csl` already exist, BigraphER is skipped and only
  PRISM re-runs. Delete `verification_results/<M>/` for a genuinely fresh run.
- **Bounded runs report lower bounds.** Exploration stops at the state bound `M`, and PRISM closes
  the truncation frontier with self-loops (`Warning: Deadlocks detected and fixed in N states`).
  The `P=? [F …]` values therefore increase with `M` and should be read as lower bounds, not as
  physical probabilities. 
