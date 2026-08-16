# Public header layout

Public headers mirror the implementation responsibilities under `src/`:

- `core/`: reusable algorithms and data structures.
- `control/`: goal generation and navigation control nodes.
- `coordination/`: ball and team coordination nodes.
- `simulation/`: simulation-only nodes.
- `plugins/`: Nav2 plugin APIs.
- `visualization/`: visualization node APIs.

Consumers should include the responsibility in the path, for example:

```cpp
#include "football_navigation/control/football_tracking_action_client.hpp"
#include "football_navigation/core/football_geometry.hpp"
```

Node entry points are not declared here; they remain small translation units
under the matching `src/<responsibility>/` directory.
