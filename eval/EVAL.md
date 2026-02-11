# Evaluation Harness (PyRoboSim + BTs)

This folder contains the evaluation harness used to score generated Behavior Trees (BTs).

## What It Evaluates
We evaluate **composition quality** of BTs (structure + parameterization), not low‑level action execution. Success is based on end‑state predicates.

**Static checks**
- Valid BT JSON and allowed node types
- Actions/locations required by the task
- Required control flow (e.g., selector, memory sequence)

**Runtime checks**
- `exec_status`: SUCCESS / FAILURE / TIMEOUT / STALL
- `goal_satisfied`: end‑state predicate (see below)
- `feasible`: coarse pre‑state feasibility check (object exists / location exists)

## Success Criteria (Current)
`run_eval.py` supports these end‑state predicates:

- `must_hold_name`: robot holds a specific object instance
- `object_at`: specific object is at a location
- `robot_at`: robot ends at a location
- `not_holding`: robot ends empty‑handed

## Limitations
- Feasibility is **coarse** (does not check reachability or open/closed state).
- Many tasks require **spawn‑level** locations (e.g., `table0_tabletop`).
- Prompts often require **exact object names** (e.g., `banana1`).
- Reactive/persistent tasks are **not evaluated**; we focus on episodic end‑state checks.

## Run the Evaluation
Use the CLI flag to choose a tasks file.

```bash
python3 eval/run_eval.py --tasks-file tasks50.yaml
python3 eval/run_eval.py --tasks-file tasks50_extended.yaml
python3 eval/run_eval.py --tasks-file tasks100_extended.yaml
```

Single task:
```bash
python3 eval/run_eval.py --tasks-file tasks50_extended.yaml --task-id task01_hold_banana0
```

Step through tasks:
```bash
python3 eval/run_eval.py --tasks-file tasks50_extended.yaml --step
```

Results are written incrementally to:
```
eval/results.json
```
