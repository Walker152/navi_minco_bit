# MincoPlanner Planner Mode Validation

## PRIORMAP

Commands:

```bash
ros2 param get /planner_server MincoPlanner.planner_mode
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /opt_path --once
```

Expected:

- `planner_mode = PRIORMAP`
- `/opt_path.header.frame_id = map`
- Global search logs show `global_search=Nav2Costmap`.
- ROGMap is queried only through the dynamic query path used by optimization, corridor, safety, and recovery distance checks.
- No start or goal conversion error mentions raw ROGMap global planning.

## EXPLORATION

Set `MincoPlanner.planner_mode: EXPLORATION` in the planner yaml and restart `planner_server`.

Commands:

```bash
ros2 param get /planner_server MincoPlanner.planner_mode
ros2 topic echo /aft_mapped_to_init --once
ros2 topic echo /opt_path --once
```

Expected:

- `planner_mode = EXPLORATION`
- `/opt_path.header.frame_id = camera_init`
- Startup logs show `static_esdf=disabled`.
- `getRobotPose()` uses `/aft_mapped_to_init` directly in `camera_init`.
- If the goal is outside the ROGMap window, global search selects a reachable boundary candidate instead of projecting the goal to the map edge.

## Hot Reload Rejection

Command:

```bash
ros2 param set /planner_server MincoPlanner.planner_mode EXPLORATION
```

Expected:

- The set request is rejected.
- Log contains: `planner_mode is not hot-reloadable, please restart planner_server`.
