# Architecture: Bus Charging Scheduler

---

## 1. Scheduling Approach

### Why OR-Tools CP-SAT?

The problem is a **constrained scheduling optimisation**: given known bus departure times, travel distances, station charger capacities, and a fixed charging duration, find the optimal start time for every charging session that minimises a weighted combination of waiting time, throughput, and operator fairness.

CP-SAT (Constraint Programming – Satisfiability) is a natural fit because:
- All constraints are **hard**: buses cannot charge simultaneously at a single-charger station, travel-time precedence is non-negotiable, and charging duration is fixed.
- The objective is a **weighted sum** of multiple terms that operators can tune.
- The solver guarantees **global optimality** (or the best feasible solution within a time budget), unlike greedy heuristics that can get stuck in local minima.

**The algorithm in four stages:**

1. **Greedy charging plan per bus** — for each bus, compute the minimum set of stations to charge at using a greedy range check: travel as far as possible before charging. This guarantees physical feasibility (no segment exceeds battery range) while minimising total stops. This stage decides *where* buses charge.

2. **Build CP-SAT model** — for each charging stop, create integer variables for arrival time, charge start, charge end, and wait time. Add constraints:
   - **Precedence**: arrival at stop `k+1` = end of charging at stop `k` + travel time between stations.
   - **Station capacity**: `AddNoOverlap` for single-charger stations; `AddCumulative` for multi-charger stations.
   - **Timing**: charge end = charge start + fixed duration; wait = charge start − arrival; charge start ≥ arrival.

3. **Weighted objective** — minimise:
   - `w_individual × Σ(bus wait times)` — reduce individual bus delays
   - `w_overall × Σ(bus final arrival times)` — improve network throughput
   - `w_operator × max-min operator average wait gap` — ensure fairness across operators

4. **Solve and extract** — CP-SAT solves the model (30-second time limit, single worker) and returns variable values. The solution is mapped into `BusResult` / `StationResult` records for the Streamlit UI.

**Why not FCFS?** First-come-first-served ignores operator fairness and accumulated wait. CP-SAT considers the global picture.

**Why not a greedy heuristic?** A greedy dispatcher makes locally optimal decisions but cannot foresee that delaying one bus now prevents a cascade of waiting later. CP-SAT evaluates all possibilities.

**Why not multi-threaded solving?** The current OR-Tools build exhibits threading issues on macOS ARM64; single-worker mode is used for reliability. The model is small enough (~40 interval variables for 20 buses) that single-threaded solving completes in under a second.

---

## 2. Data Structure Design

### Why JSON scenario files?

Each scenario is a **self-contained, declarative specification** of the entire simulation world. The scheduler has no hardcoded knowledge of distances, speeds, operators, or charger counts. This enables:

- **Zero code changes** to change physical parameters
- **Zero code changes** to add a new operator or station
- **Versioned scenarios** that can be diffed in git
- **Human-readable** — operators can inspect and edit scenarios without engineering

### Key design decisions

| Field | Why it exists |
|-------|---------------|
| `world.speed_kmh` | Not hardcoded — change once in JSON, affects all travel times |
| `world.battery_range_km` | Per-scenario; future buses may have different battery sizes |
| `stations[].chargers` | Integer ≥1; adding a second charger requires only a data change |
| `route.segments[]` | Explicit list — adding a new stop is an array append, not a code change |
| `bus.direction` | String, not boolean — supports multi-route future extensions |
| `bus.operator` | Plain string — no enum; add "megabus" with zero code change |
| `weights` | Object with named floats — add a new key = new rule with no migration |

---

## 3. Anticipated Future Changes

All of the following require **zero code changes** to the scheduler unless noted.

### Adding more chargers at a station
Change `"chargers": 1` to `"chargers": 2` in the station object. The scheduler reads `station_cap[id]` and switches between `AddNoOverlap` (1 charger) and `AddCumulative` (multiple chargers) automatically.

### Adding a new intermediate station
Add the station to `route.stops` and two entries to `route.segments` (split the existing segment). Add `{"id": "E", "chargers": 1}` to `stations`. No code change.

### Changing segment distances
Edit `distance_km` in the relevant segment. All travel times and range calculations derive from this value at runtime.

### Adding a new operator
Add buses with `"operator": "megabus"` to the buses array. The scheduler treats operator as an opaque string — the fairness term averages wait times across all buses sharing the same string.

### Adding time-of-day electricity cost rule
1. Add `"time_of_day": 0.5` to the `weights` object in the scenario JSON
2. Add `time_of_day: float = 0.0` to the `Weights` dataclass
3. Add a penalty term to the CP-SAT objective:
   ```python
   # For each charging start time, add a scaled penalty when charging
   # falls within peak hours (e.g. 18:00-22:00).
   peak_penalty = model.NewIntVar(0, horizon, f"peak_{bus.id}_{idx}")
   # ... compute peak overlap based on start_var ...
   model.Minimize(... + w_tod * total_peak_penalty)
   ```

### Priority buses (VIP / emergency)
Add `"priority": true` to a bus object. Add `bus.priority: bool = False` to the `Bus` dataclass. Add a large penalty term for non-priority buses in the objective, effectively telling the solver to schedule priority buses with minimal wait.

### Multiple routes sharing stations
`Route` is already a first-class object on `Scenario`. To share stations across routes, build a single CP-SAT model with interval variables from all routes and apply the same station-capacity constraints. The `AddNoOverlap`/`AddCumulative` constraints naturally handle cross-route contention.

### Driver shift constraints
Add `"shift_start": "18:00", "shift_end": "04:00"` to the bus object. In the CP-SAT model, add a constraint that the bus's final arrival time must be ≤ shift_end (in minutes). The solver will then schedule charging to meet this hard deadline, or report infeasibility if it cannot.

### More than 20 buses
CP-SAT handles 40 interval variables (20 buses × 2 stops) in under a second. For 200 buses (~400 intervals), the solver may need a few seconds. The 30-second time limit ensures the solver returns the best solution found within budget.

### Different battery sizes per bus
Add an optional `"battery_range_km": 320` override on individual bus objects. In `build_charging_plan()`, check `bus.battery_range_km` before falling back to `world.battery_range_km`. One additional field in the `Bus` dataclass; no scheduler logic changes.

---

## 4. How to Change a Weight

Open the scenario JSON and edit the `weights` block. Example — prioritise network throughput over individual fairness:

```json
"weights": {
  "individual": 0.5,
  "operator": 0.5,
  "overall": 2.0
}
```

Restart the app. The solver recalculates the optimal schedule with the new coefficients on every run.

---

## 5. How to Add a New Rule

**Example: penalise buses that have been travelling for more than 4 hours (fatigue proxy)**

**Step 1** — Add weight to scenario JSON:
```json
"weights": {
  "individual": 1.0,
  "operator": 1.0,
  "overall": 1.0,
  "fatigue": 1.5
}
```

**Step 2** — Add field to `Weights` dataclass in `models.py`:
```python
@dataclass
class Weights:
    individual: float = 1.0
    operator:   float = 1.0
    overall:    float = 1.0
    fatigue:    float = 0.0   # new field with safe default
```

**Step 3** — Add the objective term in `scheduler.py`:
```python
# In run_scheduler(), after building bus variables:
fatigue_penalties = []
for bus in scenario.buses:
    plan = charging_plans[bus.id]
    for idx, station in enumerate(plan):
        travel = arrival_vars[(bus.id, idx)] - _to_scaled(bus.departure_time_min)
        over_4h = model.NewIntVar(0, horizon, f"fatigue_{bus.id}_{idx}")
        model.AddMaxEquality(over_4h, [travel - _to_scaled(240), model.NewConstant(0)])
        fatigue_penalties.append(over_4h)
total_fatigue = sum(fatigue_penalties)

# Add to objective:
model.Minimize(
    w_ind * total_wait
    + w_overall * total_arrival
    + w_op * fairness_gap
    + w_fatigue * total_fatigue     # ← new term
)
```

That's the entire change. Scenarios that don't include `"fatigue"` in weights will use the default `0.0` and behave identically to before.

---

## 6. Assumptions

1. **All buses start with a full charge** (battery_range_km) at their departure terminal.
2. **Charging always refills to 100%** — partial charging is not modelled.
3. **Buses travel at constant speed** (`world.speed_kmh`) and do not stop except to charge.
4. **No backtracking** — a bus visits stations in strict route order. A BK bus will never go back to a previously passed station.
5. **Charger slots are interchangeable** — if a station has `chargers: 2`, any waiting bus can use whichever slot becomes free first.
6. **Time is simulated from midnight (minute 0)** — 19:00 departure = minute 1140.
7. **The solver produces a globally optimal schedule** within the 30-second time budget. For the 20-bus scenarios in this assignment, optimal solutions are found in under a second.
8. **The route is identical for all buses in a scenario** — there is no express service skipping stations.
9. **There is no minimum dwell time** at a station other than the charging time itself.
10. **Departures are on the same calendar day** — no midnight wrap-around is handled beyond modular hour display.
