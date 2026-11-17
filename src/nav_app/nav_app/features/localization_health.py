"""Fail closed when map matching stops, even while odometry/TF keep arriving."""
import json
import math
import time


# The Localizer rejects stale live sensor observations.
MAX_INPUT_AGE_S = 1.


class LocalizationHealth:
    def __init__(self):
        self.sample = {}
        self.received = 0.
        self.error = ""

    def update(self, text, received=None):
        self.error = ""
        try:
            row = json.loads(text)
            if not isinstance(row, dict):
                raise ValueError("quality status must be an object")
            required = ("published_stamp_ns", "match_age_s", "data_age_s", "match_sequence",
                        "request_id", "completed_request_id")
            if row.get("schema_version") != 1 or any(not isinstance(row[k], (int, float)) or not math.isfinite(row[k]) for k in required):
                raise ValueError("invalid quality status")
            manual_numbers = (
                "manual_request_id", "manual_pose_stamp_ns",
                "completed_manual_request_id",
                "completed_manual_pose_stamp_ns",
            )
            if any(key in row for key in manual_numbers):
                if (
                    any(
                        not isinstance(row.get(key), (int, float))
                        or not math.isfinite(row[key])
                        for key in manual_numbers
                    )
                    or not isinstance(
                        row.get("manual_request_succeeded"), bool)
                ):
                    raise ValueError("invalid manual request status")
            self.sample = row
            self.received = time.monotonic() if received is None else received
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            self.error = f"定位质量状态格式错误：{exc}"
            self.sample = {}
            self.received = 0.

    def valid(self, ros_now_ns, now=None):
        now = time.monotonic() if now is None else now
        row = self.sample
        return (row.get("valid") is True and 0 <= now-self.received <= 1.5
                and -.1 <= (ros_now_ns-row["published_stamp_ns"])/1e9 <= 1.5
                and 0 <= row["match_age_s"] <= 5.
                and -.1 <= row["data_age_s"] <= MAX_INPUT_AGE_S)

    def request_result(self, request_id):
        if self.sample.get("completed_request_id") != request_id:
            return None
        return self.sample.get("request_succeeded") is True
