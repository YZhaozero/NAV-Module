"""Consistent, controllable touch scrolling for the device UI."""

from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QScroller,
    QScrollerProperties,
)


# Qt's desktop defaults allow a 0.5 m/s flick and can accelerate repeated
# flicks threefold.  On the 3.2-inch screen that commonly skips an entire
# short list.  Keep a deliberate drag responsive, but stop much sooner.
TOUCH_MAXIMUM_VELOCITY = 0.18


def enable_touch_scroll(widget, *, maximum_velocity=TOUCH_MAXIMUM_VELOCITY):
    """Enable slow, per-pixel kinetic scrolling on an item view/scroll area."""
    if isinstance(widget, QAbstractItemView):
        widget.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)

    viewport = widget.viewport()
    properties = QScrollerProperties()
    metrics = {
        QScrollerProperties.DragStartDistance: 0.0035,
        QScrollerProperties.DragVelocitySmoothingFactor: 0.85,
        QScrollerProperties.MinimumVelocity: 0.01,
        QScrollerProperties.MaximumVelocity: float(maximum_velocity),
        QScrollerProperties.MaximumClickThroughVelocity: 0.04,
        QScrollerProperties.AcceleratingFlickMaximumTime: 0.0,
        QScrollerProperties.AcceleratingFlickSpeedupFactor: 1.0,
        QScrollerProperties.DecelerationFactor: 0.32,
        QScrollerProperties.OvershootDragResistanceFactor: 0.65,
        QScrollerProperties.OvershootScrollDistanceFactor: 0.12,
        QScrollerProperties.HorizontalOvershootPolicy:
            QScrollerProperties.OvershootAlwaysOff,
        QScrollerProperties.VerticalOvershootPolicy:
            QScrollerProperties.OvershootWhenScrollable,
        QScrollerProperties.FrameRate: QScrollerProperties.Fps60,
    }
    for metric, value in metrics.items():
        properties.setScrollMetric(metric, value)
    QScrollerProperties.setDefaultScrollerProperties(properties)
    # Qt's offscreen plugin can crash after creating and destroying many
    # kinetic scrollers in one test process.  It has no touch input, so retain
    # the same defaults/properties for inspection without grabbing a gesture.
    if QApplication.platformName() != "offscreen":
        QScroller.grabGesture(viewport, QScroller.LeftMouseButtonGesture)
    widget.setProperty("gsTouchScroll", True)
    viewport.setProperty("gsTouchScroll", True)
