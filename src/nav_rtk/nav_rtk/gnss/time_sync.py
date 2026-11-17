"""Offline motion-based clock diagnostic. Never silently changes a live clock."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class OffsetEstimate:
    accepted: bool
    offset_seconds: float | None
    correlation: float
    validation_correlation: float
    reason: str


def estimate_offset(lio_times, lio_speeds, gnss_times, gnss_speeds,
                    max_offset=2.0, step=0.01, max_lio_gap=0.25):
    """Sign convention: LIO time = GNSS time + offset.

    Inputs are speed magnitudes, in m/s, from already quality-gated data.
    Stationary/constant-speed motion cannot identify a clock offset.
    A held-out tail must agree; long LIO outages are never interpolated across.
    """
    lt, lv, gt, gv = (np.asarray(v, dtype=float) for v in
                      (lio_times, lio_speeds, gnss_times, gnss_speeds))
    bad = OffsetEstimate(False, None, 0., 0., "运动或时间数据不足，不能确定同步偏移")
    if (lt.ndim != 1 or gt.ndim != 1 or lv.shape != lt.shape or gv.shape != gt.shape
            or len(lt) < 30 or len(gt) < 20):
        return bad
    if (not all(np.isfinite(v).all() for v in (lt, lv, gt, gv))
            or np.any(np.diff(lt) <= 0) or np.any(np.diff(gt) <= 0)
            or gt[-1] - gt[0] < 10 or np.std(gv) < 0.15 or np.std(lv) < 0.15):
        return bad
    if not (0 < step <= max_offset and max_offset <= 30 and max_lio_gap > 0):
        raise ValueError("Invalid offset search limits")
    split = int(len(gt) * 0.7)

    def scores(offset):
        query = gt + offset
        indices = np.searchsorted(lt, query)
        inside = (indices > 0) & (indices < len(lt))
        clipped = np.clip(indices, 1, len(lt) - 1)
        inside &= lt[clipped] - lt[clipped - 1] <= max_lio_gap
        predicted = np.interp(query, lt, lv)
        result = []
        for segment in (np.arange(len(gt)) < split, np.arange(len(gt)) >= split):
            mask = inside & segment
            a, b = predicted[mask], gv[mask]
            if (len(a) < max(6, int(np.ceil(0.8 * np.count_nonzero(segment))))
                    or np.std(a) < 0.1 or np.std(b) < 0.1):
                result.append(-1.)
            else:
                result.append(float(np.corrcoef(a, b)[0, 1]))
        return result

    offsets = np.arange(-max_offset, max_offset + step / 2, step)
    correlations = np.asarray([scores(offset) for offset in offsets])
    best = int(np.argmax(correlations[:, 0]))
    train, validation = correlations[best]
    # Distinct local maxima expose periodic-motion ambiguity, unlike a broad
    # plateau immediately around the best sample.
    peaks = [i for i in range(1, len(offsets) - 1)
             if correlations[i, 0] >= correlations[i-1, 0]
             and correlations[i, 0] > correlations[i+1, 0]
             and abs(offsets[i] - offsets[best]) > 0.3]
    ambiguous = any(correlations[i, 0] > train - 0.02 for i in peaks)
    accepted = train >= 0.85 and validation >= 0.8 and not ambiguous and best not in (0, len(offsets)-1)
    return OffsetEstimate(accepted, float(offsets[best]) if accepted else None,
        float(train), float(validation), "偏移候选已通过保留段验证" if accepted else "偏移不可信，请补充变化运动或核对时钟")
