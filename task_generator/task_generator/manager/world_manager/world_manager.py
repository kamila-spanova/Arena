import itertools
import math
from collections.abc import Collection

import arena_simulation_setup.tree.World as World
import numpy as np
import scipy.signal
import shapely
from arena_runtime._node import NodeInterface

from task_generator.shared import Position, PositionRadius, Wall

from .utils import WorldMap, WorldOccupancy


def _disc_kernel(safe_dist_cells: float) -> np.ndarray:
    """L2 disc of radius safe_dist_cells, normalised so sum == 1."""
    r = max(1, int(math.ceil(safe_dist_cells)))
    yy, xx = np.ogrid[-r : r + 1, -r : r + 1]
    mask = (xx * xx + yy * yy) <= (safe_dist_cells * safe_dist_cells)
    kernel = mask.astype(np.float32)
    return kernel / kernel.sum()


def _occupancy_to_available(occupancy: np.ndarray, safe_dist_cells: float) -> np.ndarray:
    """Return (row, col) cells whose Euclidean safe_dist_cells neighbourhood is fully not-full. Off-map counts as full."""
    kernel = _disc_kernel(safe_dist_cells)
    free = WorldOccupancy.not_full(occupancy).astype(np.float32)
    spread = scipy.signal.convolve2d(free, kernel, mode="same", boundary="fill", fillvalue=0.0)
    available = np.isclose(spread, 1.0)
    return np.transpose(np.where(available))


def _sample_grid_positions(
    occupancy: np.ndarray,
    n: int,
    safe_dist_cells: float,
    rng: np.random.Generator,
    *,
    max_depth: int = 10,
) -> np.ndarray:
    """Pick n (row, col) cells with Euclidean safe_dist_cells clearance from non-empty cells and from each other.

    Returns an (n, 2) int array. Raises RuntimeError if fewer than n cells satisfy the constraint.
    """
    if n <= 0:
        return np.zeros((0, 2), dtype=np.int64)

    available = _occupancy_to_available(occupancy, safe_dist_cells)
    if len(available) < n:
        raise RuntimeError(f"need {n} positions, only {len(available)} cells satisfy safe_dist={safe_dist_cells} cells")

    accepted = np.zeros((n, 2), dtype=np.int64)
    accepted_n = 0

    for _ in range(max_depth):
        need = n - accepted_n
        if need <= 0:
            break
        if need > len(available):
            raise RuntimeError(f"need {need} more positions, only {len(available)} candidate cells available")
        for idx in rng.choice(len(available), need, replace=False):
            candidate = available[idx]
            if accepted_n and np.any(np.linalg.norm(accepted[:accepted_n] - candidate, axis=1) < safe_dist_cells):
                continue
            accepted[accepted_n] = candidate
            accepted_n += 1
            if accepted_n >= n:
                break

    if accepted_n < n:
        raise RuntimeError(f"failed to find {n} positions with safe_dist={safe_dist_cells} cells after {max_depth} retries")

    return accepted

def _sample_from_candidates(
    available: np.ndarray,
    n: int,
    safe_dist_cells: float,
    rng: np.random.Generator,
    *,
    max_depth: int = 10,
) -> np.ndarray:
    """Pick n cells from `available` (row, col) keeping safe_dist_cells separation between picks.

    Returns an (n, 2) int array. Raises RuntimeError if fewer than n cells fit.
    """
    if n <= 0:
        return np.zeros((0, 2), dtype=np.int64)

    if len(available) < n:
        raise RuntimeError(f"need {n} positions, only {len(available)} candidate cells available")

    accepted = np.zeros((n, 2), dtype=np.int64)
    accepted_n = 0

    for _ in range(max_depth):
        need = n - accepted_n
        if need <= 0:
            break
        if need > len(available):
            raise RuntimeError(f"need {need} more positions, only {len(available)} candidate cells available")
        for idx in rng.choice(len(available), need, replace=False):
            candidate = available[idx]
            if accepted_n and np.any(np.linalg.norm(accepted[:accepted_n] - candidate, axis=1) < safe_dist_cells):
                continue
            accepted[accepted_n] = candidate
            accepted_n += 1
            if accepted_n >= n:
                break

    if accepted_n < n:
        raise RuntimeError(f"failed to find {n} positions with safe_dist={safe_dist_cells} cells after {max_depth} retries")

    return accepted


class WorldManager(NodeInterface):
    """
    The map manager manages the static map
    and is used to get new goal, robot and
    obstacle positions.
    """

    _world: World.WorldDescription
    _map: WorldMap

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._detected_walls = None

    @property
    def world(self) -> World.WorldDescription:
        return self._world

    @property
    def map(self) -> WorldMap:
        return self._map

    @property
    def _shape(self) -> tuple[int, int]:
        return self._map.shape[0], self._map.shape[1]

    @property
    def origin(self) -> Position:
        return self._map.origin

    @property
    def resolution(self) -> float:
        return self._map.resolution

    _detected_walls: Collection[Wall] | None = None

    @property
    def detected_walls(self) -> Collection[Wall]:
        if self._detected_walls is None:
            self._detected_walls = self._map.detect_walls()
        return self._detected_walls

    def update_world(
        self,
        world_map: WorldMap,
        world_description: World.WorldDescription,
    ):
        self._detected_walls = None
        self._map = world_map

        counter = itertools.count(0)
        for entity in itertools.chain(world_description.all_static_entities, world_description.all_dynamic_entities):
            if not entity.name:
                entity.name = f'{next(counter)}_{entity.model.name}'
            entity.name = f'world_{entity.name}'

        self._world = world_description

        for obstacle in self.world.all_static_entities:
            self.map.occupancy.obstacle_occupy(
                *self.map.tf_posr2rect(
                    PositionRadius(
                        x=obstacle.pose.position.x,
                        y=obstacle.pose.position.y,
                        radius=1,  # TODO actual radius
                    )
                )
            )

    def forbid(self, forbidden_zones: list[PositionRadius]):
        for zone in forbidden_zones:
            self.map.occupancy.forbidden_occupy(*self.map.tf_posr2rect(zone))

    def forbid_clear(self):
        self._map.occupancy.forbidden_clear()

    def get_positions_on_map(
        self,
        n: int,
        safe_dist: float,
        forbidden_zones: list[PositionRadius] | None = None,
        forbid: bool = True,
        polygon: shapely.Polygon | None = None,
    ) -> list[Position]:
        """Sample n map positions with Euclidean safe_dist (metres) clearance from obstacles and from each other.

        Raises RuntimeError if fewer than n positions fit.
        """
        if forbidden_zones is None:
            forbidden_zones = []

        fork = self._map.occupancy.fork()
        for zone in forbidden_zones:
            fork.occupy(*self.map.tf_posr2rect(zone))

        safe_dist_cells = safe_dist / self.resolution
        rng = self.node.conf.General.RNG.value
        # cells = _sample_grid_positions(fork.grid, n, safe_dist_cells, rng)
        available = _occupancy_to_available(fork.grid, safe_dist_cells)

        if polygon is not None and len(available):
            world_xy = np.array(
                [(p.x, p.y) for p in (self._map.tf_grid2pos((int(r), int(c))) for r, c in available)],
            )
            available = available[shapely.contains_xy(polygon, world_xy[:, 0], world_xy[:, 1])]

        cells = _sample_from_candidates(available, n, safe_dist_cells, rng)

        if forbid:
            halo = int(math.ceil(safe_dist_cells))
            for row, col in cells:
                fork.occupy((int(col) - halo, int(row) - halo), (int(col) + halo, int(row) + halo))
            fork.commit()

        return [self._map.tf_grid2pos((int(row), int(col))) for row, col in cells]

    def get_position_on_map(self, safe_dist: float, forbidden_zones: list[PositionRadius] | None = None, forbid: bool = True, polygon: shapely.Polygon | None = None) -> Position:
        return self.get_positions_on_map(n=1, safe_dist=safe_dist, forbidden_zones=forbidden_zones, forbid=forbid, polygon=polygon)[0]

    id_gen = itertools.count()
