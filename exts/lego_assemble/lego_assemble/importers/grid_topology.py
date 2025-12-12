"""
Shared utilities for converting a list of bricks on a discrete grid
into a lego_assemble JsonTopology-compatible Python dict.

This module factors out the common logic used by:
  - stabletext2brick.py
  - legolization.py

The core idea:
  - Importers parse some source format into a list of bricks on a 3D integer grid.
  - Each brick is represented as (L, W, x, y, z) in stud / layer units.
  - This module turns that list into:
        {
          "schema": "lego_assemble/lego_topology@2",
          "parts": [ ... ],
          "connections": [ ... ],
          "pose_hints": [ ... ],
        }
"""

from typing import Any

Brick = tuple[int, int, int, int, int]  # (L, W, x, y, z)

BRICK_UNIT_LENGTH = 0.008      # meters per stud
PLATE_HEIGHT = 0.0032          # meters per plate
BRICK_HEIGHT = 3 * PLATE_HEIGHT  # 1 brick = 3 plates = 0.0096 m

SCHEMA_STRING = "lego_assemble/lego_topology@2"

# From BrickPart in C++:
# static constexpr InterfaceId HoleId = 0;
# static constexpr InterfaceId StudId = 1;
HOLE_IFACE_ID = 0
STUD_IFACE_ID = 1


def bricks_grid_to_topology_json(
    bricks: list[Brick],
    color: tuple[int, int, int] | list[tuple[int, int, int]] | None = (255, 255, 255),
    *,
    include_base_plate: bool = False,
    base_plate_size: tuple[int, int] | None = None,
    base_plate_color: tuple[int, int, int] | None = None,
) -> dict[str, Any]:
    """
    Convert a list of bricks on a discrete grid into a JsonTopology dict
    matching lego_assemble.io.json.JsonTopology.

    bricks:
        list of (L, W, x, y, z) where:
          - L, W are footprint dimensions in studs (along x and y),
          - x, y are integer grid coords (studs) of the footprint's lower-left corner,
          - z is integer brick layer (0-based; each layer = 1 brick height).
        All orientation (e.g. legolization's `ori`) must already be baked into
        L and W by the caller.

    color:
        - None: use (255, 255, 255) for all bricks.
        - (r, g, b): single RGB color applied to all bricks.
        - iterable of (r, g, b): per-brick colors, same length/order as `bricks`.

    include_base_plate:
        If True, a base plate part with id == 0 is added, and connections are
        created from the plate studs to all bricks on the lowest z layer.

    base_plate_size:
        Optional (L, W) override for the base plate in studs. If None, the plate
        footprint tightly covers the projected footprint of all bricks.

    base_plate_color:
        Optional RGB color for the base plate. If None, uses `color`.

    Returns:
        A dict with keys:
            - "schema": SCHEMA_STRING,
            - "parts": [ { "id", "type", "payload" }, ... ],
            - "connections": [ { ... JsonConnection ... }, ... ],
            - "pose_hints": [ { ... JsonPoseHint ... }, ... ].
    """
    bricks_list = list(bricks)
    num_bricks = len(bricks_list)

    if num_bricks == 0:
        return {
            "schema": SCHEMA_STRING,
            "parts": [],
            "connections": [],
            "pose_hints": [],
        }

    # Normalize colors:
    # - single_color: one color for all bricks
    # - per_brick_colors: list[color_i] per brick
    single_color: tuple[int, int, int] | None
    per_brick_colors: list[tuple[int, int, int]] | None

    if color is None:
        single_color = (255, 255, 255)
        per_brick_colors = None
    else:
        # Try to interpret as a single (r,g,b) tuple.
        try:
            c_seq = tuple(color)  # type: ignore[arg-type]
        except TypeError:
            c_seq = ()

        if len(c_seq) == 3 and all(isinstance(v, (int, float)) for v in c_seq):
            single_color = (int(c_seq[0]), int(c_seq[1]), int(c_seq[2]))
            per_brick_colors = None
        else:
            # Must be an iterable of per-brick colors.
            per_brick_colors = []
            for item in color:  # type: ignore[operator]
                t = tuple(item)
                if len(t) != 3:
                    raise ValueError(
                        "Per-brick colors must be iterables of length 3 (r,g,b)."
                    )
                per_brick_colors.append((int(t[0]), int(t[1]), int(t[2])))
            if len(per_brick_colors) != num_bricks:
                raise ValueError(
                    f"color iterable length ({len(per_brick_colors)}) "
                    f"does not match number of bricks ({num_bricks})."
                )
            single_color = None

    # Canonicalize brick dimensions so that L >= W for the topology payload.
    # The input bricks specify L/W such that L spans x-axis and W spans y-axis
    # in the dataset/grid coordinates. We normalize the dimensions used for
    # BrickPart payloads, and we track a per-brick orientation such that the
    # canonical (L, W) footprint is related to the dataset footprint by a
    # 0/90 degree rotation and a translation. Grid occupancy still uses the
    # original (L, W); connections encode the rotation via (offset, yaw).
    canonical_dims: list[tuple[int, int]] = []
    # Per-brick orientation in C4 (0, 1, 2, 3) and origin of the canonical
    # interface grid in dataset coordinates.
    brick_yaws: list[int] = []
    brick_origins: list[tuple[int, int]] = []

    for (L, W, x, y, z) in bricks_list:
        if L >= W:
            # Long side already along +x: no rotation.
            L_can, W_can = L, W
            yaw = 0
            origin_x = x
            origin_y = y
        else:
            # Long side along +y in the dataset: rotate canonical brick by +90°
            # so that its long side matches +y. The canonical interface origin
            # is shifted so that the rotated L_can x W_can grid covers the
            # same footprint cells as the original L x W grid.
            #
            # See docs / derivation:
            #   C_d = (x + L/2, y + W/2)
            #   C_c = (W/2, L/2)   (since L_can=W, W_can=L)
            #   origin = C_d - R_90(C_c) = (x + L, y)
            L_can, W_can = W, L
            yaw = 1  # 90 degrees
            origin_x = x + L
            origin_y = y

        canonical_dims.append((L_can, W_can))
        brick_yaws.append(yaw)
        brick_origins.append((origin_x, origin_y))

    # Compute grid bounds (dataset is typically 20x20x20, but we infer from data)
    # using the original (L, W) so connectivity matches the source layout. The
    # grid itself is indexed from 0, so we allow negative coordinates by
    # subtracting the global minima when indexing into the occupancy grid.
    min_x = min(x for (L, W, x, y, z) in bricks_list)
    min_y = min(y for (L, W, x, y, z) in bricks_list)
    min_z = min(z for (L, W, x, y, z) in bricks_list)
    max_x = max(x + L for (L, W, x, y, z) in bricks_list)
    max_y = max(y + W for (L, W, x, y, z) in bricks_list)
    max_z = max(z + 1 for (L, W, x, y, z) in bricks_list)  # z layer plus one

    grid_size_x = max_x - min_x
    grid_size_y = max_y - min_y
    grid_size_z = max_z - min_z

    # 3D occupancy grid: grid[ix][iy][iz] = brick_idx or -1
    grid = [
        [[-1 for _ in range(grid_size_z)] for _ in range(grid_size_y)]
        for _ in range(grid_size_x)
    ]

    for brick_idx, (L, W, x, y, z) in enumerate(bricks_list):
        for dx in range(L):
            for dy in range(W):
                gx = x + dx
                gy = y + dy
                gz = z
                ix = gx - min_x
                iy = gy - min_y
                iz = gz - min_z
                curr = grid[ix][iy][iz]
                if curr != -1 and curr != brick_idx:
                    raise ValueError(
                        f"Overlapping bricks at ({gx},{gy},{gz}): "
                        f"{curr} and {brick_idx}"
                    )
                grid[ix][iy][iz] = brick_idx

    # Build JsonPart list
    parts: list[dict[str, Any]] = []

    pid_offset = 1 if include_base_plate else 0

    # Optional base plate as the first part (id == 0).
    # The base plate is modeled as a 1-plate-high brick whose footprint
    # tightly covers the projected footprint of all bricks by default,
    # unless overridden via base_plate_size.
    px_plate = py_plate = 0.0
    if include_base_plate:
        if base_plate_size is None:
            plate_L = int(max_x - min_x)
            plate_W = int(max_y - min_y)
        else:
            plate_L, plate_W = base_plate_size

        if base_plate_color is not None:
            plate_color = base_plate_color
        elif single_color is not None:
            plate_color = single_color
        else:
            plate_color = (255, 255, 255)
        plate_x = min_x
        plate_y = min_y
        cx_plate = plate_x + 0.5 * plate_L
        cy_plate = plate_y + 0.5 * plate_W
        px_plate = cx_plate * BRICK_UNIT_LENGTH
        py_plate = cy_plate * BRICK_UNIT_LENGTH

        parts.append(
            {
                "id": 0,
                "type": "brick",
                "payload": {
                    "L": int(plate_L),
                    "W": int(plate_W),
                    "H": 1,
                    "color": [
                        int(plate_color[0]),
                        int(plate_color[1]),
                        int(plate_color[2]),
                    ],
                },
            }
        )

    for brick_idx, (L, W, x, y, z) in enumerate(bricks_list):
        if per_brick_colors is not None:
            c = per_brick_colors[brick_idx]
        else:
            c = single_color or (255, 255, 255)

        L_can, W_can = canonical_dims[brick_idx]

        pid = brick_idx + pid_offset
        parts.append(
            {
                "id": int(pid),
                "type": "brick",
                "payload": {
                    "L": int(L_can),
                    "W": int(W_can),
                    "H": 3,
                    "color": [
                        int(c[0]),
                        int(c[1]),
                        int(c[2]),
                    ],
                },
            }
        )

    # Helper: rotate a 2D integer vector in C4.
    def _c4_rotate(idx: int, vx: int, vy: int) -> tuple[int, int]:
        k = idx % 4
        if k == 0:
            return vx, vy
        if k == 1:
            return -vy, vx
        if k == 2:
            return -vx, -vy
        # k == 3
        return vy, -vx

    # Infer vertical stud–hole connections between bricks.
    # For each pair of vertically adjacent bricks, we compute a connection
    # segment whose (offset, yaw) describe the top brick's (hole) interface
    # relative to the bottom brick's (stud) interface in the stud's interface
    # grid, taking into account each brick's canonical orientation.
    connections: list[dict[str, Any]] = []
    seen_keys: set[tuple[int, int, int, int, int]] = set()

    for ix in range(grid_size_x):
        for iy in range(grid_size_y):
            for iz in range(grid_size_z - 1):
                b_bottom = grid[ix][iy][iz]
                b_top = grid[ix][iy][iz + 1]

                if b_bottom == -1 or b_top == -1 or b_bottom == b_top:
                    continue

                # Use bottom brick as stud, top as hole.
                # Let each brick i have:
                #   - canonical interface origin O_i = brick_origins[i]
                #   - brick-local yaw yaw_i = brick_yaws[i] in C4.
                # The connection yaw is yaw_hole - yaw_stud (hole relative to stud),
                # and the offset is the hole origin expressed in the stud's
                # interface grid: rotate (O_hole - O_stud) by -yaw_stud.
                origin_stud_x, origin_stud_y = brick_origins[b_bottom]
                origin_hole_x, origin_hole_y = brick_origins[b_top]
                yaw_stud = brick_yaws[b_bottom]
                yaw_hole = brick_yaws[b_top]

                delta_x = origin_hole_x - origin_stud_x
                delta_y = origin_hole_y - origin_stud_y
                offset_x, offset_y = _c4_rotate(-yaw_stud, delta_x, delta_y)
                yaw = (yaw_hole - yaw_stud) % 4

                key = (b_bottom, b_top, offset_x, offset_y, yaw)
                if key in seen_keys:
                    continue
                seen_keys.add(key)

                connections.append(
                    {
                        "stud_id": int(b_bottom + pid_offset),
                        "stud_iface": STUD_IFACE_ID,
                        "hole_id": int(b_top + pid_offset),
                        "hole_iface": HOLE_IFACE_ID,
                        "offset": [int(offset_x), int(offset_y)],
                        "yaw": yaw,
                    }
                )

    # Optional connections from the base plate (studs) to the bottom-layer bricks (holes)
    if include_base_plate:
        plate_id = 0
        plate_x = min_x
        plate_y = min_y
        bottom_z = min_z

        for brick_idx, (L, W, x, y, z) in enumerate(bricks_list):
            if z != bottom_z:
                continue

            # Base plate is treated as a stud provider with yaw == 0 and
            # interface origin at (plate_x, plate_y) in dataset coordinates.
            # We reuse the canonical origin/yaw of the brick as the hole.
            origin_hole_x, origin_hole_y = brick_origins[brick_idx]
            yaw_hole = brick_yaws[brick_idx]

            delta_x = origin_hole_x - plate_x
            delta_y = origin_hole_y - plate_y
            offset_x, offset_y = delta_x, delta_y  # studs' yaw is 0
            yaw = yaw_hole

            connections.append(
                {
                    "stud_id": int(plate_id),
                    "stud_iface": STUD_IFACE_ID,
                    "hole_id": int(brick_idx + pid_offset),
                    "hole_iface": HOLE_IFACE_ID,
                    "offset": [int(offset_x), int(offset_y)],
                    "yaw": yaw,
                }
            )

    # Assign stable connection ids required by schema v2.
    for cid, conn in enumerate(connections):
        conn["id"] = int(cid)

    # Build pose hints: exactly one per connected component.
    pose_hints: list[dict[str, Any]] = []
    num_parts = num_bricks + (1 if include_base_plate else 0)
    if num_parts > 0:
        # Build adjacency list from connections.
        adjacency: list[list[int]] = [[] for _ in range(num_parts)]
        for conn in connections:
            stud_id = int(conn["stud_id"])
            hole_id = int(conn["hole_id"])
            if 0 <= stud_id < num_parts and 0 <= hole_id < num_parts:
                adjacency[stud_id].append(hole_id)
                adjacency[hole_id].append(stud_id)

        visited = [False] * num_parts

        for start in range(num_parts):
            if visited[start]:
                continue

            # Collect one connected component via DFS.
            stack = [start]
            visited[start] = True
            component: list[int] = []
            while stack:
                node = stack.pop()
                component.append(node)
                for nbr in adjacency[node]:
                    if not visited[nbr]:
                        visited[nbr] = True
                        stack.append(nbr)

            # Choose anchor for this component.
            if include_base_plate and 0 in component:
                # Base plate is part of this component: use it as anchor.
                anchor_pid = 0
                anchor_px = px_plate
                anchor_py = py_plate
                # Base plate has no intrinsic yaw; keep identity rotation.
                anchor_rot = [1.0, 0.0, 0.0, 0.0]
                # Place the base plate at the lowest brick layer in this topology
                # so that floating structures (z != 0) are respected.
                anchor_pz = bottom_z * BRICK_HEIGHT
            else:
                # No base plate: choose the lowest brick in this component.
                best_key: tuple[int, int, int, int] | None = None
                anchor_pid: int | None = None
                anchor_px = anchor_py = 0.0
                anchor_yaw: int | None = None
                anchor_layer_z: int | None = None
                for pid in component:
                    if pid < pid_offset:
                        # Skip base plate id in this branch (shouldn't happen).
                        continue
                    brick_idx = pid - pid_offset
                    L, W, x, y, z = bricks_list[brick_idx]
                    key = (z, y, x, brick_idx)
                    if best_key is None or key < best_key:
                        best_key = key
                        anchor_pid = pid
                        cx = x + 0.5 * L
                        cy = y + 0.5 * W
                        anchor_px = cx * BRICK_UNIT_LENGTH
                        anchor_py = cy * BRICK_UNIT_LENGTH
                        anchor_yaw = brick_yaws[brick_idx]
                        anchor_layer_z = z

                if anchor_pid is None:
                    # Component has no bricks; skip (should not occur in practice).
                    continue

                # Map anchor yaw in C4 (0..3) to a wxyz quaternion. This mirrors
                # lego_assemble.utils.c4_rotation.c4_to_quat on the C++ side.
                assert anchor_yaw is not None
                k = anchor_yaw % 4
                if k == 0:
                    anchor_rot = [1.0, 0.0, 0.0, 0.0]
                elif k == 1:
                    import math

                    s = math.sqrt(2.0) * 0.5
                    anchor_rot = [s, 0.0, 0.0, s]
                elif k == 2:
                    anchor_rot = [0.0, 0.0, 0.0, 1.0]
                else:  # k == 3
                    import math

                    s = math.sqrt(2.0) * 0.5
                    anchor_rot = [s, 0.0, 0.0, -s]

                # Use the anchor brick's dataset z-layer to place the component
                # vertically, so floating structures are preserved.
                assert anchor_layer_z is not None
                anchor_pz = anchor_layer_z * BRICK_HEIGHT

            pose_hints.append(
                {
                    "part": int(anchor_pid),
                    "pos": [anchor_px, anchor_py, anchor_pz],
                    "rot": anchor_rot,
                }
            )

    topology = {
        "schema": SCHEMA_STRING,
        "parts": parts,
        "connections": connections,
        "pose_hints": pose_hints,
    }

    return topology
