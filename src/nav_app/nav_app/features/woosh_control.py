"""Validate the existing motion guard protocol before translating to WOOSH."""
import json
import math


def decode_command(text, now_ns, previous_sequence):
    if len(text) > 2048:
        raise ValueError("WOOSH command exceeds size limit")
    row = json.loads(text)
    if not isinstance(row, dict) or row.get("schema") != 1 or row.get("frame_id") != "base_link":
        raise ValueError("Invalid WOOSH command schema/frame")
    stamp, sequence, velocity = row["stamp_ns"], row["sequence"], row["velocity"]
    if (type(stamp) is not int or not -.05 <= (now_ns - stamp) / 1e9 <= .3
            or type(sequence) is not int or sequence <= previous_sequence
            or not isinstance(velocity, list) or len(velocity) != 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in velocity)
            or abs(velocity[0]) > .5 or velocity[1] != 0 or abs(velocity[2]) > 1):
        raise ValueError("Stale, repeated, lateral or out-of-range WOOSH command")
    return sequence, tuple(velocity)
