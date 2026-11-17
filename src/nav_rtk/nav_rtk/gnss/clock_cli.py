"""CSV columns: timestamp,speed. Offset is diagnostic only, never applied."""
import argparse
from dataclasses import asdict
import json
import numpy as np
from .time_sync import estimate_offset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lio_csv")
    parser.add_argument("gnss_csv")
    parser.add_argument("--max-offset", type=float, default=2.)
    args = parser.parse_args()
    lio = np.genfromtxt(args.lio_csv, delimiter=",", names=True)
    gnss = np.genfromtxt(args.gnss_csv, delimiter=",", names=True)
    result = estimate_offset(np.atleast_1d(lio["timestamp"]), np.atleast_1d(lio["speed"]),
        np.atleast_1d(gnss["timestamp"]), np.atleast_1d(gnss["speed"]), max_offset=args.max_offset)
    print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
    return 0 if result.accepted else 2
