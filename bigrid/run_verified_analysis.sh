#!/usr/bin/env bash
# =====================================================================
# Tier-1 exhaustive verification for uavs_3d_verified_2x2x2.big.
#
# Explores the reachable state space to a fixpoint (no -M truncation),
# runs the qualitative PCTL queries in verification_queries_verified.props
# through PRISM, performs a structural deadlock-freedom analysis, and
# writes a machine-readable summary consumed by
# generate_verification_artifacts.py.
#
# Usage:  bash run_verified_analysis.sh
# =====================================================================
set -euo pipefail

MODEL="uavs_3d_verified_2x2x2.big"
PROPS="verification_queries_verified.props"
OUT="verification_results/verified"
TRA="$OUT/verified.tra"
CSL="$OUT/verified.csl"
QCSL="$OUT/verified_q.csl"
JSON="$OUT/verified_results.json"
# High bound: exploration terminates at the fixpoint well below this.
MAXSTATES=5000000

mkdir -p "$OUT"

echo "[1/4] Exhaustive state-space export (bigrapher full) ..."
bigrapher full -M "$MAXSTATES" -p "$TRA" -l "$CSL" "$MODEL" 2>&1 | tee "$OUT/export.log" | \
  grep -iE "States:|Transitions:|Maximum number" || true

if grep -qi "Maximum number of states reached" "$OUT/export.log"; then
  echo "WARNING: exploration hit the bound; increase MAXSTATES."; exit 1
fi

read -r NSTATES NTRANS < <(head -1 "$TRA")
echo "  States=$NSTATES  Transitions=$NTRANS  (fixpoint reached -> 100% coverage)"

echo "[2/4] Assembling PRISM property file ..."
cp "$CSL" "$QCSL"
grep -E "^P=\?|^P>|^P>=" "$PROPS" | grep -v "^//" >> "$QCSL"
NPROPS=$(grep -cE "^P=\?|^P>|^P>=" "$PROPS")
echo "  $NPROPS queries"

echo "[3/4] Running PRISM (-dtmc) over the complete DTMC ..."
declare -a RESULTS
for p in $(seq 1 "$NPROPS"); do
  r=$(prism -importtrans "$TRA" "$QCSL" -dtmc -prop "$p" -javastack 512m 2>/dev/null \
        | grep -E "Result:" | head -1 | sed -E 's/Result:\s*//; s/ .*//')
  RESULTS[$p]="$r"
  echo "  prop $p: $r"
done

echo "[4/4] Structural deadlock-freedom analysis ..."
python3 - "$TRA" "$CSL" "$JSON" "$NSTATES" "$NTRANS" "${RESULTS[@]:1}" <<'PY'
import sys, re, json
tra, csl, out_json, nstates, ntrans = sys.argv[1:6]
prism_results = sys.argv[6:]
nstates, ntrans = int(nstates), int(ntrans)

lines = open(tra).read().splitlines()
srcs = set(int(l.split()[0]) for l in lines[1:] if l.strip())
dead = set(range(nstates)) - srcs

csl_txt = open(csl).read()
def states_of(label):
    m = re.search(r'label "%s" = ([^;]*);' % re.escape(label), csl_txt)
    if not m: return set()
    return set(int(x) for x in re.findall(r'x\s*=\s*(\d+)', m.group(1)))

flying = states_of('drone_flying')
coll   = states_of('collision')
dead_flying = dead & flying          # gridlocks: stuck with a flying UAS
dead_coll   = dead & coll

summary = {
  "config": {"grid": "2x2x2", "locales": 8, "uas": 2,
             "charging_stations": ["v0"], "obstacles": 0,
             "initial": "d0@v0, d1@v1, both Flying/Battery.Normal/Communication.Normal"},
  "state_space": {"states": nstates, "transitions": ntrans,
                  "coverage": "100% (fixpoint; no bound reached)"},
  "prism_results": prism_results,
  "deadlock": {"total": len(dead), "with_flying_uas": len(dead_flying),
               "collision": len(dead_coll),
               "all_landed_terminals": len(dead - flying),
               "gridlock_free": len(dead_flying) == 0 and len(dead_coll) == 0},
}
json.dump(summary, open(out_json, "w"), indent=2)
print(f"  deadlocks={len(dead)}  with_flying_uas={len(dead_flying)}  "
      f"collision={len(dead_coll)}  gridlock_free={summary['deadlock']['gridlock_free']}")
print(f"  wrote {out_json}")
PY

echo "Done. Summary: $JSON"
