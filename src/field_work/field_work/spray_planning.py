"""Solve a fixed-endpoint spray tour under the original length and footprint constraints."""

import itertools
import math
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import lil_matrix
from shapely.geometry import LineString, shape


def heading_change(a, b):
    return abs(math.remainder(b - a, 2 * math.pi))


def angle(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def minimum_turn_route(points, allowed, initial_yaw, length_cap):
    """A vertex chooses one pair of roads. Neighbor agreement and cut constraints form one path."""
    n = len(points)
    choices = []
    for vertex in range(n):
        degree = 1 if vertex in (0, n - 1) else 2
        choices.extend(
            (vertex, peers)
            for peers in itertools.combinations(np.flatnonzero(allowed[vertex]), degree)
        )
    constraints = []
    low, high = [], []

    def add(row, lower, upper):
        constraints.append(row)
        low.append(lower)
        high.append(upper)

    for vertex in range(n):
        add({k: 1.0 for k, (i, _) in enumerate(choices) if i == vertex}, 1.0, 1.0)
    for u, v in np.argwhere(np.triu(allowed, 1)):
        # Both endpoints of an edge must select it together.
        add(
            {
                k: 1.0 if i == u else -1.0
                for k, (i, peers) in enumerate(choices)
                if (i == u and v in peers) or (i == v and u in peers)
            },
            0.0,
            0.0,
        )
    add(
        {
            k: sum(math.dist(points[i], points[j]) for j in peers) / 2
            for k, (i, peers) in enumerate(choices)
        },
        0.0,
        length_cap,
    )
    objective = []
    for i, peers in choices:
        outgoing = angle(points[i], points[peers[-1]])
        incoming = initial_yaw if i == 0 else angle(points[peers[0]], points[i])
        objective.append(0.0 if i == n - 1 else heading_change(incoming, outgoing))

    cut_count = 0
    while True:
        matrix = lil_matrix((len(constraints), len(choices)))
        for row, terms in enumerate(constraints):
            for col, value in terms.items():
                matrix[row, col] = value
        result = milp(
            objective,
            integrality=np.ones(len(choices)),
            bounds=Bounds(0, 1),
            constraints=LinearConstraint(matrix.tocsr(), low, high),
            options={"mip_rel_gap": 0.0, "time_limit": 30.0},
        )
        if not result.success or result.mip_gap > 1e-8:
            raise ValueError(
                f"Cannot certify a complete minimum-turn route: {result.message}"
            )
        adjacency = {
            i: peers for (i, peers), used in zip(choices, result.x) if used > 0.5
        }
        route, current = [], 0
        while True:
            route.append(current)
            if current == n - 1:
                break
            current = next(j for j in adjacency[current] if j not in route)
        if len(route) == n:
            return np.array(route), float(result.fun), cut_count
        # The endpoint component is a path. Every other component is an isolated
        # cycle and needs at least two edges to the rest of any complete path.
        unvisited = set(adjacency) - set(route)
        while unvisited:
            stack = [min(unvisited)]
            cycle = set()
            while stack:
                node = stack.pop()
                if node not in cycle:
                    cycle.add(node)
                    stack.extend(adjacency[node])
            unvisited -= cycle
            add(
                {
                    k: sum(j not in cycle for j in peers)
                    for k, (i, peers) in enumerate(choices)
                    if i in cycle and any(j not in cycle for j in peers)
                },
                2.0,
                np.inf,
            )
            cut_count += 1


def plan_route(inputs):
    points = np.asarray(inputs["points_map_xy_m"], dtype=float)
    boundary = shape(inputs["allowed_prior"])
    radius = max(math.hypot(x, y) for x, y in inputs["footprint"]) + inputs["margin_m"]
    distances = np.linalg.norm(points[None, :, :] - points[:, None, :], axis=2)
    edge_cap = np.max(distances.diagonal(offset=1))
    allowed = np.zeros_like(distances, dtype=bool)
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            if distances[i, j] <= edge_cap + 1e-8:
                edge = LineString([points[i], points[j]])
                allowed[j, i] = allowed[i, j] = (
                    boundary.covers(edge) and edge.distance(boundary.boundary) >= radius
                )

    def summarize(indices):
        visited = points[indices]
        headings = [
            inputs["starting_yaw_rad"],
            *(angle(a, b) for a, b in zip(visited, visited[1:])),
        ]
        length = sum(math.dist(a, b) for a, b in zip(visited, visited[1:]))
        turns = sum(heading_change(a, b) for a, b in zip(headings, headings[1:]))
        if not allowed[indices[:-1], indices[1:]].all() or len(
            set(indices.tolist())
        ) != len(points):
            raise ValueError("Invalid service order or swept edge")
        return {"length_m": length, "turn_rad": turns}

    original = summarize(np.arange(len(points)))
    ordered, bound, cuts = minimum_turn_route(
        points, allowed, inputs["starting_yaw_rad"], original["length_m"]
    )
    improved = summarize(ordered)
    if (
        not math.isclose(bound, improved["turn_rad"], abs_tol=1e-6)
        or improved["length_m"] > original["length_m"] + 1e-7
    ):
        raise ValueError("Reconstructed route disagrees with the solver")
    return {
        "original": original,
        "optimized": improved,
        "order": [inputs["point_ids"][i] for i in ordered],
        "subtour_cuts": cuts,
    }
