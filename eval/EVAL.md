# Evaluation Harness (PyRoboSim + BTs)

This folder contains the evaluation harness used to score generated Behavior Trees (BTs).

## What It Evaluates
We evaluate **composition quality** of BTs (structure + parameterization), not low‑level action execution. Success is based on end‑state predicates.

Current experimental setup:
- World: `roscon_2024_workshop_world.yaml`
- Task file: `eval/tasks50_roscon.yaml`
- Baselines: `schema-only` and `sequence-only`
- Assumption: agent has access to `list_world_entities` for canonical vocabulary grounding.

## Validation Separation
Validation is intentionally split between server-time checks and evaluation-time checks.

- `send_to_robot` and MCP `validate_bt` run the same **schema/vocabulary** static checks:
  - BT JSON shape (including top-level `root`)
  - allowed node types
  - known skill/action names
  - parameter presence/unknown fields
  - optional vocabulary grounding against world entities
- These server-time checks do **not** enforce behavior-policy rules (for example, detect-before-pick ordering).
- Behavior-policy and task-structure expectations are evaluated in the harness (`run_eval.py`) via task-specific `required` fields and runtime outcomes.
- Runtime success/failure remains dynamic and is measured after execution (`exec_status`, `goal_satisfied`, `feasible`).

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
- `must_hold_category`: robot holds an object of a specific category
- `must_place_category_at`: at least one object of a category is at a location
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
python3 eval/run_eval.py --tasks-file tasks50_roscon.yaml
```

Single task:
```bash
python3 eval/run_eval.py --tasks-file tasks50_roscon.yaml --task-id task01_hold_bread
```

Step through tasks:
```bash
python3 eval/run_eval.py --tasks-file tasks50_roscon.yaml --step
```

Results are written incrementally to:
```
eval/results.json
```
