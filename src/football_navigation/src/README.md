# Source layout

The implementation is grouped by responsibility. ROS executable and library
target names remain unchanged; this layout only separates source ownership.

- `core/`: reusable geometry and planning calculations without ROS node entry points.
- `control/`: striker goal generation, trajectory adaptation, and Nav2 action control.
- `coordination/`: shared ball state and team role assignment.
- `simulation/`: fake-world publishers and the single-robot simulation controller.
- `plugins/`: Nav2 costmap/progress-checker plugin implementation.
- `visualization/`: RViz marker aggregation and presentation.

Public headers mirror these folders under `include/football_navigation/`.
Include paths therefore state the owning responsibility explicitly.

ROS node targets use a consistent three-file structure:

- `include/.../<name>.hpp`: node class declaration and owned state.
- `src/.../<name>.cpp`: out-of-line method implementations.
- `src/.../<name>_node.cpp` (or `*_main.cpp`): process entry point only.
