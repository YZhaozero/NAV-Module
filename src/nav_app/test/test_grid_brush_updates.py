"""Brush edits recolor only local cells and retain full-map undo/save state."""
import math
import numpy as np
from PyQt5.QtGui import QImage
from nav_app import qt_nav_node
from nav_app.qt_nav_node import MapPanel
from nav_app.map_processing import GridMap, save_grid_map, load_grid_map
from test_height_preview import window


def image_bytes(panel):
    image = panel._grid_pixmap.toImage().convertToFormat(QImage.Format_RGB888)
    return image.bits().asstring(image.byteCount())


def test_patch_render_matches_full_render_across_all_views_and_keeps_undo_save(window, monkeypatch, tmp_path):
    occupancy = np.full((80, 120), -1, dtype=np.int8)
    occupancy[10:30, 20:50] = 100
    grid = GridMap(occupancy.copy(), .1, np.array([5., -3., .4]))
    window._set_local_grid(grid, reset_original=True)
    window.set_editor_map_mode('grid')
    window.snapshot_map_edit()
    sizes = []; full_renders = []
    color = qt_nav_node.occupancy_to_bgr
    render = window._render_grid
    def local_color(values):
        sizes.append(values.shape)
        return color(values)
    monkeypatch.setattr(qt_nav_node, 'occupancy_to_bgr', local_color)
    monkeypatch.setattr(window, '_render_grid', lambda grid: full_renders.append(True) or render(grid))
    for tool, column, row in [('occupied', 0, 0), ('free', 35, 20), ('unknown', 0, 0)]:
        x, y = (column+.5)*.1, (row+.5)*.1
        world_x = 5+math.cos(.4)*x-math.sin(.4)*y
        world_y = -3+math.sin(.4)*x+math.cos(.4)*y
        window.edit_map_at('grid', tool, world_x, world_y, .2)
    assert len(sizes) == 9 and all(h <= 5 and w <= 5 for h, w in sizes)
    assert not full_renders
    assert window._rendered_grid_edit_revision == window._grid_edit_revision
    assert grid.occupancy[20, 35] == 0 and grid.occupancy[0, 0] == -1
    assert grid.occupancy[10, 20] == 100
    monkeypatch.setattr(qt_nav_node, 'occupancy_to_bgr', color)
    reference = MapPanel()
    reference.set_map(grid.occupancy, grid.origin[:2], grid.resolution, grid.origin[2])
    for panel in (window.map_panel, window.editor_map_panel, window.active_page.map_panel):
        assert image_bytes(panel) == image_bytes(reference)
    saved, _ = save_grid_map(tmp_path/'edited.yaml', grid)
    assert np.array_equal(load_grid_map(saved).occupancy, grid.occupancy)
    assert len(window._undo_stack) == 1
    window.undo_map_edit()
    assert np.array_equal(grid.occupancy, occupancy)
    assert window._rendered_grid_edit_revision == window._grid_edit_revision
    reference.set_map(occupancy, grid.origin[:2], grid.resolution, grid.origin[2])
    assert image_bytes(window.editor_map_panel) == image_bytes(reference)
    reference.close()


def test_painting_same_value_does_not_recolor_or_advance_revision(window, monkeypatch):
    grid = GridMap(np.zeros((80, 120), dtype=np.int8), .05, np.zeros(3))
    window._set_local_grid(grid, reset_original=True)
    revision = window._grid_edit_revision
    def unexpected(*args):
        raise AssertionError('unchanged cells should not rebuild any image')
    monkeypatch.setattr(qt_nav_node, 'occupancy_to_bgr', unexpected)
    window.edit_map_at('grid', 'free', 2., 1., .2)
    assert window._grid_edit_revision == revision


def test_live_grid_is_initialized_before_first_patch(window):
    window.node.occupancy = np.full((80, 120), -1, dtype=np.int8)
    window.node.map_resolution = .1
    window.node.map_origin = np.array([0., 0.])
    window.node.map_origin_yaw = 0.
    window.node.grid_frame = 'map'
    window.local_grid = None
    window.edit_map_at('grid', 'free', 2., 1., .2)
    assert window.local_grid.occupancy[10, 20] == 0
    for panel in (window.map_panel, window.editor_map_panel, window.active_page.map_panel):
        assert panel._grid_pixmap is not None
        assert panel._grid_shape == (80, 120)
    assert window._rendered_grid_edit_revision == window._grid_edit_revision
