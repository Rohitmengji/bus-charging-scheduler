"""
scheduler.py — OR-Tools CP-SAT optimization engine for the Bus Charging Scheduler.

This module uses a constraint programming model (CP-SAT) to schedule charging
sessions while respecting:
- bus travel-time precedence constraints,
- station charger capacity constraints,
- fixed charging duration,
- route feasibility constraints.

Objective (weighted):
- minimize total bus waiting time,
- minimize overall arrival times (throughput),
- minimize operator wait imbalance (fairness across operators).
"""

from collections import defaultdict
from dataclasses import dataclass
import pickle
from pathlib import Path
import subprocess
import sys
from typing import Dict, List, Tuple

from models import (
    Bus,
    BusResult,
    ChargingStop,
    Route,
    Scenario,
    ScheduleResult,
    StationResult,
    StationSlot,
    World,
)

try:
    from ortools.sat.python import cp_model
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "OR-Tools is required. Install dependencies via: pip install -r requirements.txt"
    ) from exc


TIME_SCALE = 1


@dataclass
class StopTask:
    bus: Bus
    station: str
    stop_idx: int
    station_seq: int


def travel_time(distance_km: float, speed_kmh: float) -> float:
    """Return travel time in minutes."""
    return (distance_km / speed_kmh) * 60.0


def stations_for_direction(route: Route, direction: str) -> List[str]:
    """Return intermediate stations in travel order for the bus direction."""
    intermediate = route.stops[1:-1]
    return intermediate if direction == "BK" else list(reversed(intermediate))


def ordered_segments(route: Route, direction: str) -> List[Tuple[str, str, float]]:
    """Return (from, to, dist_km) in travel order for the given direction."""
    segs = [(s.from_stop, s.to_stop, s.distance_km) for s in route.segments]
    if direction == "KB":
        segs = [(t, f, d) for f, t, d in reversed(segs)]
    return segs


def build_charging_plan(bus: Bus, world: World, route: Route) -> List[str]:
    """
    Build a feasible charging-station sequence using greedy range checking.

    This stage decides *where* buses charge. CP-SAT decides *when* they charge.
    """
    segs = ordered_segments(route, bus.direction)
    station_set = {s for s in route.stops[1:-1]}

    plan: List[str] = []
    range_left = world.battery_range_km

    for i, (frm, to, dist) in enumerate(segs):
        if dist > world.battery_range_km:
            raise ValueError(
                f"Segment {frm}->{to} ({dist} km) exceeds max range "
                f"({world.battery_range_km} km)."
            )
        range_left -= dist

        if to in station_set:
            next_dist = segs[i + 1][2] if i + 1 < len(segs) else 0.0
            if range_left < next_dist or range_left < 0:
                plan.append(to)
                range_left = world.battery_range_km

    # Defensive minimum for this assignment's route specs.
    if len(plan) < 2:
        stations_in_order = stations_for_direction(route, bus.direction)
        if stations_in_order and stations_in_order[0] not in plan:
            plan.insert(0, stations_in_order[0])
        if len(plan) < 2:
            for s in stations_in_order[1:]:
                if s not in plan:
                    plan.append(s)
                    break

    return plan


def segment_distance(route: Route, direction: str, from_stop: str, to_stop: str) -> float:
    """Return cumulative distance between two stops along the directed route."""
    segs = ordered_segments(route, direction)
    total = 0.0
    counting = False
    for frm, to, dist in segs:
        if frm == from_stop:
            counting = True
        if counting:
            total += dist
        if counting and to == to_stop:
            break
    return total


def _to_scaled(minutes: float) -> int:
    return int(round(minutes * TIME_SCALE))


def _from_scaled(value: int) -> float:
    return value / TIME_SCALE


def _run_scheduler_impl(scenario: Scenario) -> ScheduleResult:
    """Core solver logic — build CP-SAT model, solve, and extract results."""
    world = scenario.world
    route = scenario.route
    weights = scenario.weights

    charging_plans: Dict[str, List[str]] = {
        bus.id: build_charging_plan(bus, world, route) for bus in scenario.buses
    }

    station_cap: Dict[str, int] = {s.id: max(1, int(s.chargers)) for s in scenario.stations}
    charge_dur = _to_scaled(world.charge_time_min)

    # Build task list (one charging operation = one CP task).
    tasks: List[StopTask] = []
    tasks_by_bus: Dict[str, List[StopTask]] = defaultdict(list)
    tasks_by_station: Dict[str, List[StopTask]] = defaultdict(list)

    for bus in scenario.buses:
        for idx, station in enumerate(charging_plans[bus.id]):
            task = StopTask(bus=bus, station=station, stop_idx=idx, station_seq=len(tasks_by_station[station]))
            tasks.append(task)
            tasks_by_bus[bus.id].append(task)
            tasks_by_station[station].append(task)

    # Horizon estimate: departures + all travel + all charging + buffer.
    max_departure = max((b.departure_time_min for b in scenario.buses), default=0.0)
    max_trip_time = travel_time(sum(s.distance_km for s in route.segments), world.speed_kmh)
    horizon_min = max_departure + max_trip_time + len(tasks) * world.charge_time_min + 300.0
    horizon = _to_scaled(horizon_min)

    model = cp_model.CpModel()

    start_vars: Dict[Tuple[str, int], cp_model.IntVar] = {}
    end_vars: Dict[Tuple[str, int], cp_model.IntVar] = {}
    arrival_vars: Dict[Tuple[str, int], cp_model.IntVar] = {}
    wait_vars: Dict[Tuple[str, int], cp_model.IntVar] = {}
    interval_vars_by_station: Dict[str, List[cp_model.IntervalVar]] = defaultdict(list)

    # Create vars and precedence constraints.
    for bus in scenario.buses:
        plan = charging_plans[bus.id]
        dep = _to_scaled(bus.departure_time_min)

        for idx, station in enumerate(plan):
            key = (bus.id, idx)
            arr = model.NewIntVar(0, horizon, f"arr_{bus.id}_{idx}")
            start = model.NewIntVar(0, horizon, f"start_{bus.id}_{idx}")
            end = model.NewIntVar(0, horizon, f"end_{bus.id}_{idx}")
            wait = model.NewIntVar(0, horizon, f"wait_{bus.id}_{idx}")

            model.Add(end == start + charge_dur)
            model.Add(wait == start - arr)
            model.Add(start >= arr)

            if idx == 0:
                origin = route.stops[0] if bus.direction == "BK" else route.stops[-1]
                dist = segment_distance(route, bus.direction, origin, station)
                travel = _to_scaled(travel_time(dist, world.speed_kmh))
                model.Add(arr == dep + travel)
            else:
                prev_station = plan[idx - 1]
                prev_end = end_vars[(bus.id, idx - 1)]
                dist = segment_distance(route, bus.direction, prev_station, station)
                travel = _to_scaled(travel_time(dist, world.speed_kmh))
                model.Add(arr == prev_end + travel)

            start_vars[key] = start
            end_vars[key] = end
            arrival_vars[key] = arr
            wait_vars[key] = wait

            interval = model.NewIntervalVar(start, charge_dur, end, f"int_{bus.id}_{idx}")
            interval_vars_by_station[station].append(interval)

    # Station capacity constraints.
    for station_id, intervals in interval_vars_by_station.items():
        if station_cap.get(station_id, 1) == 1:
            model.AddNoOverlap(intervals)
        else:
            demands = [1] * len(intervals)
            model.AddCumulative(intervals, demands, station_cap[station_id])

    # Bus-level totals and arrivals.
    bus_wait_total: Dict[str, cp_model.IntVar] = {}
    bus_arrival_final: Dict[str, cp_model.IntVar] = {}

    total_wait = model.NewIntVar(0, horizon * max(1, len(scenario.buses)), "total_wait")
    total_arrival = model.NewIntVar(0, horizon * max(1, len(scenario.buses)), "total_arrival")

    wait_terms = []
    arrival_terms = []

    for bus in scenario.buses:
        plan = charging_plans[bus.id]

        bw = model.NewIntVar(0, horizon, f"bus_wait_{bus.id}")
        bus_wait_total[bus.id] = bw

        if plan:
            waits = [wait_vars[(bus.id, idx)] for idx in range(len(plan))]
            model.Add(bw == sum(waits))

            last_end = end_vars[(bus.id, len(plan) - 1)]
            last_station = plan[-1]
            destination = route.stops[-1] if bus.direction == "BK" else route.stops[0]
            dist_last = segment_distance(route, bus.direction, last_station, destination)
            travel_last = _to_scaled(travel_time(dist_last, world.speed_kmh))

            ba = model.NewIntVar(0, horizon * 2, f"bus_arrival_{bus.id}")
            model.Add(ba == last_end + travel_last)
            bus_arrival_final[bus.id] = ba
        else:
            model.Add(bw == 0)
            total_dist = sum(s.distance_km for s in route.segments)
            travel_full = _to_scaled(travel_time(total_dist, world.speed_kmh))
            ba = model.NewIntVar(0, horizon * 2, f"bus_arrival_{bus.id}")
            model.Add(ba == _to_scaled(bus.departure_time_min) + travel_full)
            bus_arrival_final[bus.id] = ba

        wait_terms.append(bw)
        arrival_terms.append(bus_arrival_final[bus.id])

    model.Add(total_wait == sum(wait_terms) if wait_terms else 0)
    model.Add(total_arrival == sum(arrival_terms) if arrival_terms else 0)

    # Operator fairness via average-wait range minimization.
    buses_by_operator: Dict[str, List[Bus]] = defaultdict(list)
    for bus in scenario.buses:
        buses_by_operator[bus.operator].append(bus)

    op_avg_vars: List[cp_model.IntVar] = []
    for op, op_buses in buses_by_operator.items():
        op_total = model.NewIntVar(0, horizon * len(op_buses), f"op_total_wait_{op}")
        model.Add(op_total == sum(bus_wait_total[b.id] for b in op_buses))
        op_avg = model.NewIntVar(0, horizon, f"op_avg_wait_{op}")
        model.AddDivisionEquality(op_avg, op_total, len(op_buses))
        op_avg_vars.append(op_avg)

    fairness_gap = model.NewIntVar(0, horizon, "operator_fairness_gap")
    if op_avg_vars:
        max_op = model.NewIntVar(0, horizon, "max_op_avg_wait")
        min_op = model.NewIntVar(0, horizon, "min_op_avg_wait")
        model.AddMaxEquality(max_op, op_avg_vars)
        model.AddMinEquality(min_op, op_avg_vars)
        model.Add(fairness_gap == max_op - min_op)
    else:
        model.Add(fairness_gap == 0)

    # Weighted objective.
    w_ind = int(round(weights.individual * 1000))
    w_op = int(round(weights.operator * 1000))
    w_overall = int(round(weights.overall * 1000))

    model.Minimize(w_ind * total_wait + w_overall * total_arrival + w_op * fairness_gap)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 5.0
    solver.parameters.num_workers = 1
    status = solver.Solve(model)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError("CP-SAT could not find a feasible schedule for this scenario.")

    # Build output records.
    bus_results: List[BusResult] = []
    station_slots: Dict[str, List[StationSlot]] = defaultdict(list)

    for bus in scenario.buses:
        plan = charging_plans[bus.id]
        charging_stops: List[ChargingStop] = []
        total_wait_min = 0.0

        for idx, station in enumerate(plan):
            arr = _from_scaled(solver.Value(arrival_vars[(bus.id, idx)]))
            start = _from_scaled(solver.Value(start_vars[(bus.id, idx)]))
            end = _from_scaled(solver.Value(end_vars[(bus.id, idx)]))
            wait = max(0.0, start - arr)
            total_wait_min += wait

            if idx == 0:
                origin = route.stops[0] if bus.direction == "BK" else route.stops[-1]
            else:
                origin = plan[idx - 1]
            dist_to_stop = segment_distance(route, bus.direction, origin, station)
            range_remaining = max(0.0, world.battery_range_km - dist_to_stop)

            charging_stops.append(
                ChargingStop(
                    station=station,
                    arrival_time_min=arr,
                    wait_minutes=wait,
                    charge_start_min=start,
                    charge_end_min=end,
                    range_remaining_on_arrival_km=range_remaining,
                )
            )

            station_slots[station].append(
                StationSlot(
                    bus_id=bus.id,
                    operator=bus.operator,
                    start_min=start,
                    end_min=end,
                    wait_minutes=wait,
                    arrival_time_min=arr,
                )
            )

        arrival_final = _from_scaled(solver.Value(bus_arrival_final[bus.id]))
        bus_results.append(
            BusResult(
                bus_id=bus.id,
                operator=bus.operator,
                direction=bus.direction,
                departure_time_min=float(bus.departure_time_min),
                charging_stops=charging_stops,
                total_wait_minutes=total_wait_min,
                arrival_time_min=arrival_final,
                trip_duration_minutes=arrival_final - bus.departure_time_min,
            )
        )

    station_results = []
    for st in scenario.stations:
        slots = sorted(station_slots.get(st.id, []), key=lambda s: s.start_min)
        station_results.append(StationResult(station_id=st.id, charging_order=slots))

    return ScheduleResult(buses=bus_results, stations=station_results)


# ═══════════════════════════════════════════════════════════════════════════════
# Subprocess wrapper — isolates OR-Tools native code from Streamlit's threads.
# ═══════════════════════════════════════════════════════════════════════════════

_WORKER_SCRIPT = Path(__file__).parent / "_solve_worker.py"


def run_scheduler(scenario: Scenario) -> ScheduleResult:
    """Public entry point.  Delegates to a subprocess for Streamlit compatibility.

    OR-Tools CP-SAT's native solver deadlocks inside Streamlit's threaded
    script runner on macOS.  Running the solver in a fresh subprocess avoids
    this entirely.
    """
    proc = subprocess.run(
        [sys.executable, str(_WORKER_SCRIPT)],
        input=pickle.dumps(scenario),
        capture_output=True,
        timeout=15,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"Solver subprocess failed (exit {proc.returncode}):\n"
            + proc.stderr.decode(errors="replace")
        )
    return pickle.loads(proc.stdout)
