# Performance Recording Slimdown Design

## Goal

Refactor runtime performance recording according to `docs/sentry/reparams.md`: remove AWARD-style aggregate metrics from live modules and keep only raw per-cycle timing, frequency, status, velocity, and projection classification data needed for tuning and field debugging.

## Scope

This change updates the runtime logging surfaces for:

- MINCO planner CSV at `/tmp/minco_perf_detailed.csv`
- MPC controller CSV at `/tmp/mpc_perf_detailed.csv`
- ROGMap detailed CSV at `/tmp/rog_map_perf_detailed.csv`
- ROGMap summary CSV at `/tmp/rog_map_perf_summary.csv`
- Performance parameter comments/defaults in `src/navigation/navi2_bringup/params/sentry1.yaml`

The change does not modify Point-LIO cloud paths, intra-process transport, ROGMap H-ratio classification behavior, FieldLayer/ESDF construction, MINCO optimization behavior, MPC solving behavior, recovery behavior, or MapSnapshot freeze logic.

## MINCO Planner

`MincoPerfSample` becomes a small raw sample:

- `stamp_ros`
- `stamp_steady_ns`
- `planner_mode`
- `success`
- `failure_reason`
- `global_search_time_ms`
- `local_search_time_ms`
- `optimizer_time_ms`
- `total_replan_time_ms`
- `planner_hz`

`PlanGlobalPath(...)` measures global search elapsed time and stores it for the next `ReplanLocal(...)` row. `ReplanLocal(...)` measures total replan time, local front-end time before optimizer invocation, optimizer invocation time, and planner row-to-row frequency. If no fresh global search time is available, the CSV writes `NaN`.

The old CSV-only sampling helpers are removed:

- `recordQueryMetadata(...)`
- `sampleSeedClearance(...)`
- `sampleTrajectoryMetrics(...)`

Existing warning/error diagnostics and safety checks remain intact.

## MPC Controller

`MpcPerfSample` keeps only:

- `stamp_ros`
- `stamp_steady_ns`
- `success`
- `solve_time_ms`
- `cycle_time_ms`
- `controller_hz`
- `cmd_vx`
- `cmd_vy`
- `cmd_wz`
- `ref_vx`
- `ref_vy`
- `ref_wz`

`computeVelocityCommands(...)` measures full cycle time and solver call time. It records final command velocities and the first reference velocity when a reference exists. It no longer computes tracking, cross-track, along-track, or yaw error for CSV output. Existing debug-only prints may remain.

The MPC performance deadline parameter and deadline CSV field are removed from the active logging path.

## ROGMap Performance Monitor

ROGMap detailed CSV writes one row per valid map update with the header from `docs/sentry/reparams.md`:

```csv
run_id,scenario,variant,stamp_ros,stamp_steady_ns,cloud_callback_hz,map_update_hz,input_points,cloud_convert_time_ms,raycast_time_ms,prob_update_time_ms,decay_time_ms,projection_time_ms,field_time_ms,snapshot_time_ms,total_update_time_ms,free_count,passable_count,occupied_count,unknown_count,reason_insufficient_observation,reason_empty_column,reason_thin_surface,reason_solid_vertical_wall,reason_hollow_tunnel,reason_ambiguous_occupied
```

ROGMap summary CSV writes only window frequency plus the latest module timings and projection type counts:

```csv
run_id,scenario,variant,window_start_stamp,window_duration_sec,cloud_callback_hz,map_update_hz,field_update_hz,last_total_update_time_ms,last_raycast_time_ms,last_projection_time_ms,last_field_time_ms,last_free_count,last_passable_count,last_occupied_count,last_unknown_count
```

The monitor no longer opens or writes `navigation_award_metrics.csv` or `navigation_award_run_manifest.json`. It does not compute average/max/percentile/RMSE/success-rate summaries in the runtime path.

## Projection Classification Counts

Projection classification counts are taken from the existing `ProjectionLayer` update results. The implementation may either keep the current `ProjectionUpdateStats` reason counters or introduce a small `ProjectionClassCounts` struct, but it must not change classification decisions or mask/value generation.

The required counts are:

- `free_count`
- `passable_count`
- `occupied_count`
- `unknown_count`
- `reason_insufficient_observation`
- `reason_empty_column`
- `reason_thin_surface`
- `reason_solid_vertical_wall`
- `reason_hollow_tunnel`
- `reason_ambiguous_occupied`

## Parameters

Runtime logging uses the minimal performance parameter set documented in `docs/sentry/reparams.md`. Existing path parameter names may remain where changing them would force broad YAML migration, but comments and active behavior must no longer advertise AWARD metrics, deadline miss metrics, or field stale CSV metrics.

`performance.enable=false` keeps logging and timing overhead minimal: no CSV files are opened, scoped monitor timers disable themselves, and row writers return immediately.

## Error Handling

CSV file open failures log errors and disable only the affected CSV writer. Navigation, mapping, planning, and control behavior continue.

Non-finite numeric values write `NaN` in CSV output.

CSV flush behavior remains configurable and flushes every `csv_flush_every_n` rows for ROGMap and every 30 rows for existing MINCO/MPC writers unless a local flush parameter already exists.

## Verification

Because project instructions prohibit building without explicit permission, verification for this pass uses static checks only:

- Search for removed CSV field names in active logger headers/rows.
- Confirm new CSV headers match the requested field order.
- Confirm removed helper functions are no longer declared or called.
- Confirm AWARD manifest/metrics writing is no longer active.
- Confirm YAML no longer exposes AWARD/deadline/field-stale logging comments.

