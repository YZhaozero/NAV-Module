#!/usr/bin/env python3
"""One-time Ubuntu/GDM device setup. Does not restart the desktop or APP."""
import argparse
import configparser
import json
import os
from pathlib import Path
import re
import subprocess


CONFIG = Path("/etc/gdm3/custom.conf")
STATE = Path("/var/lib/nav/desktop-login")


def run(*args):
    result = subprocess.run(["systemctl", *args], text=True, capture_output=True, timeout=15)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "systemctl failed")
    return result.stdout.strip()


def configured_text(original, username):
    parser = configparser.ConfigParser(interpolation=None, strict=False)
    parser.read_string(original)
    values = {"AutomaticLoginEnable": "true", "AutomaticLogin": username,
              "TimedLoginEnable": "false", "WaylandEnable": "false"}
    # Keep vendor comments and all unrelated sections/settings.
    result, daemon, inserted = [], False, False
    for line in original.splitlines():
        match = re.match(r"\s*\[([^]]+)\]", line)
        if match:
            if daemon:
                result.extend(f"{key}={value}" for key, value in values.items())
                inserted = True
            daemon = match[1] == "daemon"
        if daemon and any(re.match(r"\s*" + key + r"\s*=", line) for key in values):
            continue
        result.append(line)
    if daemon:
        result.extend(f"{key}={value}" for key, value in values.items())
        inserted = True
    if not inserted:
        result.extend(["", "[daemon]", *(f"{key}={value}" for key, value in values.items())])
    return "\n".join(result) + "\n"


def atomic_write(path, content, mode=0o644):
    temporary = path.with_suffix(path.suffix + ".nav-next")
    with temporary.open("w", encoding="utf-8") as stream:
        os.chmod(temporary, mode)
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def enable(username):
    import pwd
    account = pwd.getpwnam(username)
    if account.pw_uid < 1000 or not Path(account.pw_dir).is_dir():
        raise ValueError("Select the normal device desktop user")
    manager = run("show", "display-manager.service", "--property=FragmentPath", "--value")
    if Path(manager).name not in ("gdm.service", "gdm3.service"):
        raise RuntimeError("Automatic setup supports Ubuntu GDM only")
    original = CONFIG.read_text(encoding="utf-8")
    updated = configured_text(original, username)
    STATE.mkdir(parents=True, exist_ok=True, mode=0o700)
    backup = STATE/"before.json"
    if not backup.exists():
        atomic_write(backup, json.dumps({"config": original, "default_target": run("get-default")}), 0o600)
    atomic_write(CONFIG, updated)
    try:
        run("set-default", "graphical.target")
    except Exception:
        atomic_write(CONFIG, original)
        raise
    print("Desktop auto-login configured; current session unchanged.")


def restore():
    backup = json.loads((STATE/"before.json").read_text(encoding="utf-8"))
    run("set-default", backup["default_target"])
    atomic_write(CONFIG, backup["config"])
    print("Previous desktop login configuration restored; current session unchanged.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("enable", "restore"))
    parser.add_argument("--user")
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error("Run this one-time desktop setup with sudo")
    try:
        if args.action == "enable":
            if not args.user or not re.fullmatch(r"[a-z_][a-z0-9_-]*[$]?", args.user):
                raise ValueError("A valid --user is required")
            enable(args.user)
        else:
            restore()
    except (OSError, KeyError, ValueError, RuntimeError, configparser.Error, subprocess.SubprocessError) as exc:
        print("Desktop login setup failed: " + str(exc))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
