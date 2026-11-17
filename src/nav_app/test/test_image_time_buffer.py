from types import SimpleNamespace as NS
import numpy as np
from nav_app.image_time_buffer import DisplayFrame, ImageTimeBuffer


def frame(t, received=10., size=100):
    return DisplayFrame(np.zeros(size, dtype=np.uint8),
                        NS(sec=int(t), nanosec=round((t-int(t))*1e9)), received, 'camera', 'raw')


def test_images_before_tf_eventually_display_without_requiring_a_new_image():
    buffer = ImageTimeBuffer()
    a, b = frame(100.), frame(100.1)
    buffer.append(a); buffer.append(b)
    assert buffer.select(100_200_000_000, 10.2, lambda f: f.nanoseconds <= 100_050_000_000) is a
    assert buffer.select(100_200_000_000, 10.2, lambda f: True) is b
    assert buffer.select(100_200_000_000, 10.2) is b


def test_old_receive_time_and_old_source_time_never_provide_ar_fallback():
    buffer = ImageTimeBuffer()
    stale_source, stale_receive, latest = frame(90.), frame(100., 1.), frame(100.1)
    for f in (stale_source, stale_receive, latest): buffer.append(f)
    probed = []
    def ready(f): probed.append(f); return False
    assert buffer.select(100_200_000_000, 10.2, ready) is latest
    assert len(probed) == 1 and probed[0] is latest


def test_clock_reset_discards_previous_epoch_and_memory_is_bounded():
    buffer = ImageTimeBuffer(capacity=6, max_bytes=250)
    for t in range(100, 200): buffer.append(frame(t))
    assert buffer.bytes == 200 and len(buffer.frames) == 2
    reset = frame(1.)
    buffer.append(reset)
    assert len(buffer.frames) == 1 and buffer.frames[0] is reset
