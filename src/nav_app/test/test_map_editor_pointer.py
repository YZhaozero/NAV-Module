"""Painting must preserve the camera while navigation gestures remain available."""
import numpy as np
import pytest
from PyQt5.QtCore import Qt, QEvent, QPointF
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from nav_app.map_processing import GridMap, load_grid_map, save_grid_map
from test_height_preview import window


@pytest.fixture
def editor(window, application):
    window._set_local_grid(GridMap(np.zeros((80, 120), dtype=np.int8), .1, np.zeros(3)), reset_original=True)
    window.set_editor_map_mode('grid')
    window.show_map_tools()
    window.resize(1280, 720)
    window.show()
    application.processEvents()
    window.edit_tool_combo.setCurrentIndex(window.edit_tool_combo.findData('occupied'))
    panel = window.editor_map_panel
    panel._map_zoom = 2.
    panel._map_pan = QPointF(35, 15)
    return window, panel


def test_double_click_paints_without_fitting_the_view(editor):
    window, panel = editor
    before = panel._map_rect()
    QTest.mouseDClick(panel, Qt.LeftButton, pos=panel.rect().center())
    QTest.mouseRelease(panel, Qt.LeftButton, pos=panel.rect().center())
    assert panel._map_rect() == before
    assert np.count_nonzero(window.local_grid.occupancy == 100) > 0


def test_changing_tool_cancels_previous_pan_before_next_brush_drag(editor):
    window, panel = editor
    start = panel.rect().center()
    QTest.mousePress(panel, Qt.RightButton, pos=start)
    window.edit_tool_combo.setCurrentIndex(window.edit_tool_combo.findData('free'))
    before = panel._map_rect()
    QTest.mousePress(panel, Qt.LeftButton, pos=start)
    QApplication.sendEvent(panel, QMouseEvent(QEvent.MouseMove, QPointF(start.x()+30, start.y()+10),
        Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
    QTest.mouseRelease(panel, Qt.LeftButton, pos=start)
    QTest.mouseRelease(panel, Qt.RightButton, pos=start)
    assert panel._map_rect() == before


def test_brush_drag_changes_cells_without_moving_map(editor):
    window, panel = editor
    start = panel.rect().center()
    before = panel._map_rect()
    QTest.mousePress(panel, Qt.LeftButton, pos=start)
    QApplication.sendEvent(panel, QMouseEvent(QEvent.MouseMove, QPointF(start.x()+40, start.y()),
        Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
    QTest.mouseRelease(panel, Qt.LeftButton, pos=start)
    assert panel._map_rect() == before
    assert np.count_nonzero(window.local_grid.occupancy == 100) > 1


def test_browse_double_click_still_resets_view(editor):
    _window, panel = editor
    panel.set_edit_tool('navigate', .1)
    QTest.mouseDClick(panel, Qt.LeftButton, pos=panel.rect().center())
    assert panel._map_zoom == 1.
    assert panel._map_pan == QPointF()


def test_loaded_grid_keeps_original_pgm_black_white_and_unknown(window, tmp_path):
    source = GridMap(np.array([[0, -1, 100], [100, 0, -1]], dtype=np.int8), .05, np.zeros(3))
    yaml, pgm = save_grid_map(tmp_path/'original.yaml', source)
    for path in (yaml, pgm):
        grid = load_grid_map(path)
        window._set_local_grid(grid, reset_original=True)
        image = window.editor_map_panel._grid_pixmap.toImage()
        # PGM rows are top-down: loading and painting must not swap cell labels.
        assert image.pixelColor(0, 0).red() == 30
        assert image.pixelColor(1, 0).red() == 215
        assert image.pixelColor(0, 1).red() == 215
        assert image.pixelColor(2, 1).red() == 30
        assert np.array_equal(grid.occupancy, source.occupancy)
