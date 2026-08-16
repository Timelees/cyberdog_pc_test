# Source layout

The implementation is grouped by responsibility. ROS executable and library
target names remain unchanged; this layout only separates source ownership.

- `core/`: reusable geometry and planning calculations without ROS node entry points.
- `control/`: striker goal generation, trajectory adaptation, and Nav2 action control.
- `coordination/`: shared ball state and team role assignment.
- `simulation/`: fake-world publishers and the single-robot simulation controller.
- `plugins/`: Nav2 costmap/progress-checker plugin implementation.
- `visualization/`: RViz marker aggregation and presentation.

Public headers remain under `include/football_navigation/`, so moving a source
between these folders does not change the package's public include paths.
