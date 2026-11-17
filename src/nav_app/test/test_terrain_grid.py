"""Terrain geometry and unsupported gaps, independent of any robot runtime."""
import numpy as np
import pytest
from nav_app.features.terrain_grid import terrain_grid


@pytest.fixture
def flat_columns():
    x, y = np.meshgrid(np.arange(25)*.15, np.arange(19)*.15)
    return np.c_[x.ravel(), y.ravel(), np.full(x.size, -.29)]


def test_sloping_ground_connects_far_outside_one_global_height_band(flat_columns):
    columns = flat_columns.copy()
    columns[:, 2] += columns[:, 0]*.20
    original = columns.copy()
    grid, layers, data = terrain_grid(columns, columns, ground_z=-.29)
    assert layers['ground'].all()
    assert grid.occupancy[-3, -3] == 0
    assert columns[-1, 2] > -.29+.5
    np.testing.assert_array_equal(columns, original)
    assert data['connected_ground_cells'] == len(columns)


def test_raw_wall_points_override_inferred_free_floor(flat_columns):
    wall = flat_columns[np.isclose(flat_columns[:, 0], 1.8)].copy()
    wall[:, 2] += .7
    points = np.vstack((flat_columns, wall))
    grid, _, data = terrain_grid(points, flat_columns, ground_z=-.29)
    ij = np.floor((wall[:, :2]-grid.origin[:2])/grid.resolution).astype(int)
    assert np.all(grid.occupancy[ij[:, 1], ij[:, 0]] == 100)
    assert data['occupied_cells'] == len(wall)
    assert data['free_cells'] > 100


def test_short_supported_hole_is_reported_as_interpolation(flat_columns):
    columns = flat_columns.copy()
    selected = np.isclose(columns[:, 0], 1.8) & np.isclose(columns[:, 1], 1.35)
    points = columns[~selected].copy()
    columns[selected, 2] = np.nan
    grid, layers, data = terrain_grid(points, columns, ground_z=-.29)
    assert layers['interpolated'].sum() == 1
    assert layers['ground'][9, 12]
    assert data['interpolated_surface_cells'] == 1
    assert grid.occupancy.min() == 0


def test_large_unobserved_gap_stays_unknown(flat_columns):
    columns = flat_columns.copy()
    selected = (columns[:, 0] >= 1.35) & (columns[:, 0] <= 2.25)
    points = columns[~selected].copy()
    columns[selected, 2] = np.nan
    grid, layers, _ = terrain_grid(points, columns, ground_z=-.29)
    center_x = int((1.8-grid.origin[0])/grid.resolution)
    assert np.all(grid.occupancy[:, center_x] == -1)
    assert not layers['ground'][:, 12].any()


def test_tabletop_cannot_seed_a_second_ground_component(flat_columns):
    columns = flat_columns.copy()
    selected = ((columns[:, 0] >= 1.2) & (columns[:, 0] <= 2.4)
                & (columns[:, 1] >= .9) & (columns[:, 1] <= 1.8))
    columns[selected, 2] += 1.
    _, layers, _ = terrain_grid(columns, columns, ground_z=-.29)
    chosen_yx = np.rint(columns[selected, :2]/.15).astype(int)
    assert not layers['ground'][chosen_yx[:, 1], chosen_yx[:, 0]].any()
    assert layers['ground'][2, 2]


def test_drop_larger_than_step_does_not_become_a_connected_floor(flat_columns):
    columns = flat_columns.copy()
    columns[columns[:, 0] >= 1.8, 2] -= .4
    _, layers, _ = terrain_grid(columns, columns, ground_z=-.29)
    assert not layers['ground'][:, 13:].any()
    assert layers['ground'][2, 2]


def test_translation_preserves_grid_content_and_offsets_origin(flat_columns):
    grid, _, _ = terrain_grid(flat_columns, flat_columns, ground_z=-.29)
    moved = flat_columns+np.array([1000.123, -500.456, 2.])
    shifted, _, _ = terrain_grid(moved, moved, ground_z=1.71)
    np.testing.assert_array_equal(shifted.occupancy, grid.occupancy)
    np.testing.assert_allclose(shifted.origin, grid.origin+[1000.123, -500.456, 0.])


def test_observed_vertical_wall_survives_without_nearby_floor(flat_columns):
    columns = flat_columns.copy()
    columns[columns[:, 0] >= 2.1, 2] = 3.
    wall = np.array([[3.3, 1.35, z] for z in (-.05, .25, .55, .85, 1.1)])
    points = np.vstack((columns, wall))
    grid, _, _ = terrain_grid(points, columns, ground_z=-.29)
    ix, iy = np.floor((wall[0, :2]-grid.origin[:2])/grid.resolution).astype(int)
    assert grid.occupancy[iy, ix] == 100


def test_ceiling_alone_is_neither_floor_nor_obstacle_at_robot_height(flat_columns):
    columns = flat_columns.copy()
    columns[columns[:, 0] >= 2.1, 2] = 3.
    grid, _, _ = terrain_grid(columns, columns, ground_z=-.29)
    ix = int((3.3-grid.origin[0])/grid.resolution)
    assert np.all(grid.occupancy[:, ix] == -1)
