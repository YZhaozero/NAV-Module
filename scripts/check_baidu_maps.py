#!/usr/bin/env python3
"""Check Baidu credentials without printing them; online checks are opt-in.

Only public documentation example coordinates are transmitted. No device state,
GNSS position, rosbag, browser session or user route is read by this utility.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import sys
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, Request, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/nav_app'))
from nav_app.baidu_client import request_url


KEYS = ("NAV_BAIDU_BROWSER_AK", "NAV_BAIDU_SERVER_AK")
SIGNING_KEY = "NAV_BAIDU_SERVER_SK"
MAX_RESPONSE = 4 * 1024 * 1024


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # Do not forward an AK embedded in the query to any redirect target.
        return None


def load_keys(path: Path | None, environ=None) -> dict[str, str]:
    env = os.environ if environ is None else environ
    values = {}
    if path is not None:
        # Parse data, never source/execute the environment file as shell code.
        for line in path.read_text(encoding="utf-8-sig").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            name, sep, value = line.partition("=")
            if sep and name.strip() in (*KEYS, SIGNING_KEY):
                value = value.strip()
                if len(value) >= 2 and value[0] in "\"'" and value[-1] == value[0]:
                    value = value[1:-1]
                values[name.strip()] = value
    # An explicitly supplied file takes precedence; missing entries use env.
    result = {name: values.get(name, env.get(name, "")).strip() for name in KEYS}
    signing_key=values.get(SIGNING_KEY, env.get(SIGNING_KEY, "")).strip()
    if signing_key:
        result[SIGNING_KEY]=signing_key
    return result


def key_state(value: str) -> str:
    if not value:
        return "missing"
    return "present_unverified" if re.fullmatch(r"[A-Za-z0-9]{16,128}", value) else "invalid_format"


def request_json(path: str, params: dict, key: str, signing_key='') -> dict:
    url = request_url(path, params, key, signing_key)
    request = Request(url, headers={"Accept": "application/json"})
    with build_opener(NoRedirect()).open(request, timeout=10) as response:
        raw = response.read(MAX_RESPONSE + 1)
    if len(raw) > MAX_RESPONSE:
        raise ValueError("response_too_large")
    result = json.loads(raw)
    if not isinstance(result, dict):
        raise ValueError("invalid_response")
    return result


def finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_result(name: str, payload: dict) -> bool:
    result = payload.get("result")
    if name == "coordinate_conversion":
        if not isinstance(result, list) or len(result) != 1 or not isinstance(result[0], dict):
            return False
        x, y = result[0].get("x"), result[0].get("y")
        return finite_number(x) and finite_number(y) and -180 <= x <= 180 and -90 <= y <= 90
    routes = result.get("routes") if isinstance(result, dict) else None
    if not isinstance(routes, list) or not routes or not isinstance(routes[0], dict):
        return False
    route = routes[0]
    distance = route.get("distance")
    steps = route.get("steps")
    return (finite_number(distance) and distance > 0 and isinstance(steps, list) and bool(steps)
            and all(isinstance(step, dict) and isinstance(step.get("path"), str)
                    and bool(step["path"]) for step in steps))


def inspect(keys: dict[str, str], online=False, request=request_json) -> dict:
    states = {"browser_ak": key_state(keys[KEYS[0]]), "server_ak": key_state(keys[KEYS[1]])}
    report = {"configuration": states, "online_requested": online,
              "browser_map_verified": False, "app_integrated": False, "services": {},
              "server_auth": "sn" if keys.get(SIGNING_KEY) else "ip"}
    if not online or states["server_ak"] != "present_unverified":
        return report
    probes = {
        "coordinate_conversion": ("/geoconv/v2/", {"coords": "114.218927,29.575430", "model": 2, "output": "json"}),
        "walking": ("/directionlite/v1/walking", {
            "origin": "40.011160,116.339303", "destination": "39.936404,116.452562",
            "coord_type": "bd09ll", "ret_coordtype": "bd09ll", "steps_info": 1}),
    }
    for name, (path, params) in probes.items():
        try:
            data = (request(path, params, keys[KEYS[1]], keys[SIGNING_KEY]) if keys.get(SIGNING_KEY)
                    else request(path, params, keys[KEYS[1]]))
            status = data.get("status") if isinstance(data, dict) else None
            if type(status) is not int:
                item = {"ok": False, "error": "invalid_response"}
            elif status != 0:
                item = {"ok": False, "api_status": status}
            else:
                valid = valid_result(name, data)
                item = {"ok": valid, "api_status": 0}
                if not valid:
                    item["error"] = "invalid_result"
        except HTTPError as exc:
            item = {"ok": False, "error": "http_error", "http_status": exc.code}
        except (URLError, OSError, ValueError):
            # Exception text/URLs and upstream response messages may contain keys.
            item = {"ok": False, "error": "network_or_response_error"}
        report["services"][name] = item
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    default_file = Path.home()/".config/nav/maps.env"
    parser.add_argument("--env-file", type=Path,
                        default=default_file if default_file.is_file() else None)
    parser.add_argument("--online", action="store_true", help="Make two API calls using public example coordinates")
    args = parser.parse_args(argv)
    try:
        keys = load_keys(args.env_file)
    except (OSError, UnicodeError):
        print(json.dumps({"error": "cannot_read_configuration"}))
        return 2
    report = inspect(keys, args.online)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    configured = all(v == "present_unverified" for v in report["configuration"].values())
    passed = len(report["services"]) == 2 and all(v["ok"] for v in report["services"].values())
    return 0 if configured and (not args.online or passed) else 1


if __name__ == "__main__":
    raise SystemExit(main())
