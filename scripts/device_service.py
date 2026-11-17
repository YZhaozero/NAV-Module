#!/usr/bin/env python3
"""Install/check/roll back the user service, or find a usable local X display."""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src/nav_app"))
from nav_app.features import startup


LAUNCHER_NAME = "nav-app.desktop"


def command(args, **kwargs):
    return subprocess.run(args, text=True, capture_output=True, timeout=kwargs.pop("timeout", 5), **kwargs)


def launcher_content():
    return ("[Desktop Entry]\n"
            "Version=1.0\n"
            "Type=Application\n"
            "Name=NAV\n"
            "Name[zh_CN]=识动 NAV\n"
            "Comment=Navigation and mapping\n"
            "Comment[zh_CN]=导航与建图\n"
            'Exec=/bin/sh -c "systemctl --user reset-failed nav-app.service; exec systemctl --user start nav-app.service"\n'
            "TryExec=systemctl\n"
            "Icon=applications-science\n"
            "Terminal=false\n"
            "StartupNotify=false\n"
            "Categories=Science;\n")


def desktop_directory(home=None):
    """Return the user's configured desktop, including localized names."""
    home = Path(home or Path.home()).resolve()
    try:
        result = command(
            ["xdg-user-dir", "DESKTOP"],
            env={**os.environ, "HOME": str(home)}, timeout=2)
        configured = Path(result.stdout.strip()).expanduser()
        # xdg-user-dir returns HOME when the desktop directory is disabled.
        if result.returncode == 0 and configured.is_absolute() and configured != home:
            return configured
    except (OSError, subprocess.SubprocessError):
        pass
    for name in ("Desktop", "桌面"):
        candidate = home/name
        if candidate.is_dir():
            return candidate
    return home/"Desktop"


def _write_launcher(path, content, mode):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".next")
    with temporary.open("w", encoding="utf-8") as stream:
        os.chmod(temporary, mode)
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def install_launchers(home=None):
    """Install both the application-menu entry and a trusted desktop icon."""
    home = Path(home or Path.home()).resolve()
    content = launcher_content()
    menu_launcher = home/".local/share/applications"/LAUNCHER_NAME
    desktop_launcher = desktop_directory(home)/LAUNCHER_NAME
    _write_launcher(menu_launcher, content, 0o644)
    _write_launcher(desktop_launcher, content, 0o755)
    trusted = False
    try:
        result = command(
            ["gio", "set", str(desktop_launcher), "metadata::trusted", "true"],
            env={**os.environ, "HOME": str(home)}, timeout=5)
        trusted = result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        # Executable launchers still work on desktops without GIO metadata.
        pass
    return {"application_menu": str(menu_launcher),
            "desktop": str(desktop_launcher), "desktop_trusted": trusted}


def find_display():
    preferred = os.environ.get("NAV_DISPLAY", os.environ.get("DISPLAY", ""))
    displays = [preferred] if preferred else []
    displays += [":"+p.name[1:] for p in sorted(Path("/tmp/.X11-unix").glob("X*"))]
    authorities = [os.environ.get("NAV_XAUTHORITY", os.environ.get("XAUTHORITY", "")),
                   f"/run/user/{os.getuid()}/gdm/Xauthority", str(Path.home()/".Xauthority")]
    accessible = []
    for display in dict.fromkeys(displays):
        if not re.fullmatch(r":\d+(\.\d+)?", display):
            continue
        for authority in dict.fromkeys(authorities):
            if not authority or not Path(authority).is_file():
                continue
            env = {**os.environ, "DISPLAY": display, "XAUTHORITY": authority}
            try:
                if command(["xdpyinfo"], env=env, timeout=2).returncode != 0:
                    continue
                if preferred and display == preferred:
                    return display, authority
                accessible.append((display, authority))
                # A virtual/remote X session may also accept this user's cookie.
                # Prefer the physical HDMI/DP/DSI panel when no display was selected.
                try:
                    outputs = command(["xrandr", "--query"], env=env, timeout=2).stdout
                    if re.search(r"^(?:HDMI|DP|eDP|DSI)[-\w]* connected", outputs, re.MULTILINE):
                        return display, authority
                except (OSError, subprocess.SubprocessError):
                    pass
            except (OSError, subprocess.SubprocessError):
                pass
    if accessible:
        return accessible[0]
    raise RuntimeError("No accessible graphical session; log into the device desktop and check Xauthority/xdpyinfo")


def check(release):
    release = Path(release).resolve()
    errors = []
    for name in ("install/local_setup.bash", "scripts/start_device_app.sh", "scripts/runtime_environment.sh"):
        if not (release/name).is_file():
            errors.append("Missing " + name)
    display = None
    try:
        display = find_display()[0]
    except RuntimeError as exc:
        errors.append(str(exc))
    result = command(["systemctl", "--user", "show", "nav-app.service", "--property=LoadState,ActiveState,Result,NRestarts,ExecMainStatus"])
    try:
        boot = startup.status()
    except RuntimeError as exc:
        boot = {"error": str(exc)}
    desktop_launcher = desktop_directory()/LAUNCHER_NAME
    return {"release": str(release), "display": display, "errors": errors,
            "service": result.stdout.strip(),
            "service_available": result.returncode == 0 and "LoadState=loaded" in result.stdout.splitlines(),
            "startup": boot,
            "desktop_launcher": {"path": str(desktop_launcher),
                                 "exists": desktop_launcher.is_file(),
                                 "executable": os.access(desktop_launcher, os.X_OK)}}


def preserve_device_profiles(home=None):
    """Keep device calibration independent of a replaceable release tree."""
    home = Path(home or Path.home()).resolve()
    directory = home/".config/nav"
    pending = []
    for name in ("fast_livo_params.yaml", "camera_pinhole.yaml", "localizer.yaml"):
        target = directory/name
        if not target.is_symlink():
            continue
        source = target.resolve()
        roots = (home/"nav-current", (home/"nav-current").resolve(),
                 home/"nav-releases")
        if not any(source.is_relative_to(root) for root in roots):
            continue
        if not target.is_file():
            raise RuntimeError(f"设备配置链接已失效，请先恢复原配置：{target}")
        resolved = target.resolve()
        companion = resolved.with_suffix(".calibration.json")
        destination = target.with_suffix(".calibration.json")
        if companion.is_file():
            data = companion.read_bytes()
            if destination.exists() and destination.read_bytes() != data:
                raise RuntimeError(f"设备标定说明不一致，请先核对：{destination}")
            pending.append((destination, data))
        pending.append((target, target.read_bytes()))
    # Read every source and detect conflicts before replacing any link.
    for target, data in pending:
        temporary = target.with_name(target.name+".preserve-next")
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(target)
    return [str(target) for target, _ in pending]


def install(release, state, unit_content=None, autostart=None):
    release = Path(release).resolve()
    report = check(release)
    if report["errors"]:
        raise RuntimeError("; ".join(report["errors"]))
    preflight = command(["bash", "-c", 'source "$1/scripts/runtime_environment.sh" && python3 -c '
                         "'from nav_app.qt_nav_node import NavigationWindow; import nav_rtk; import nav_rtk'",
                         "nav-preflight", str(release)], timeout=30)
    if preflight.returncode:
        raise RuntimeError("Release Python packages are incomplete; rebuild before switching: " + preflight.stderr[-1200:])
    # Upgrading must retain a choice made in the APP. First installation defaults on.
    autostart = startup.preferred_enabled() if autostart is None else autostart
    current = Path.home()/"nav-current"
    if current.exists() and not current.is_symlink():
        raise RuntimeError("nav-current is a directory; refusing to overwrite")
    previous = str(current.resolve()) if current.is_symlink() else None
    unit = release/"deployment/nav-app.service"
    if not unit.is_file():
        raise RuntimeError("Service template missing")
    temporary = current.with_name(".nav-current.next")
    if temporary.exists() or temporary.is_symlink():
        raise RuntimeError("Pending release link exists; inspect .nav-current.next")
    unit_content = unit.read_bytes() if unit_content is None else unit_content
    # Fail before stopping the running release if the visible launchers cannot be written.
    install_launchers()
    state.mkdir(parents=True, exist_ok=True)
    journal = {"previous_release": previous, "release": str(release), "installed_unix": time.time()}
    unit_path = Path.home()/".config/systemd/user/nav-app.service"
    unit_path.parent.mkdir(parents=True, exist_ok=True)
    if previous != str(release):
        if unit_path.exists():
            (state/"previous.service").write_bytes(unit_path.read_bytes())
        (state/"last-install.json").write_text(json.dumps(journal,indent=2))
    # Device-only calibration may never have been tracked by Git. A fresh
    # package build can legitimately omit it, so retain its exact bytes before
    # nav-current changes. This also applies when rolling back a release.
    preserve_device_profiles()
    # Stop before changing the release. A recorder is allowed to flush by the unit timeout.
    if report["service_available"]:
        stopped = command(["systemctl","--user","stop","nav-app.service"],timeout=960)
        if stopped.returncode != 0:
            raise RuntimeError("Could not stop existing service: " + stopped.stderr)
    temporary.symlink_to(release, target_is_directory=True)
    temporary.replace(current)
    unit_path.write_bytes(unit_content)
    for args in (["daemon-reload"], ["enable" if autostart else "disable", "nav-app.service"],
                 ["reset-failed", "nav-app.service"], ["restart", "nav-app.service"]):
        result = command(["systemctl", "--user", *args], timeout=30)
        if result.returncode:
            raise RuntimeError("Service update failed; run rollback: " + result.stderr)
    startup.set_enabled(autostart)
    print(json.dumps(check(release), indent=2))


def install_options(config_path=None, override=None):
    config = json.loads(Path(config_path).read_text(encoding="utf-8")) if config_path else {}
    if not isinstance(config, dict) or set(config) - {"autostart", "desktop_autologin"}:
        raise ValueError("Unknown device install option")
    if any(type(value) is not bool for value in config.values()):
        raise ValueError("Device install options must be true or false")
    # A saved APP preference wins over the sample's default. CLI can explicitly override it.
    enabled = (startup.preferred_enabled() if startup.preference_path().exists()
               else config.get("autostart", True))
    return (enabled if override is None else override == "on"), config.get("desktop_autologin", False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("display", "check", "install", "rollback"))
    parser.add_argument("--release", default=str(Path.home()/"nav-current"))
    parser.add_argument("--wait", type=float, default=0.)
    parser.add_argument("--config", help="Device installation JSON, e.g. deployment/device-install.json")
    parser.add_argument("--autostart", choices=("on", "off"), help="Override the saved APP preference")
    args = parser.parse_args()
    state = Path.home()/".local/state/nav/service-manager"
    try:
        if args.action == "display":
            deadline = time.monotonic()+min(45., max(0.,args.wait))
            while True:
                try:
                    display, authority = find_display()
                    print("export DISPLAY="+shlex.quote(display))
                    print("export XAUTHORITY="+shlex.quote(authority))
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(1)
        elif args.action == "check":
            report = check(args.release)
            print(json.dumps(report, indent=2))
            return int(bool(report["errors"]))
        elif args.action == "install":
            enabled, desktop_login = install_options(args.config, args.autostart)
            report = check(args.release)
            if report["errors"]:
                raise RuntimeError("; ".join(report["errors"]))
            if desktop_login and not startup.desktop_login_ready():
                helper = Path(__file__).resolve().with_name("desktop_autologin.py")
                result = subprocess.run(["sudo", "python3", str(helper), "enable", "--user", startup.getpass.getuser()], timeout=120)
                if result.returncode:
                    raise RuntimeError("Desktop auto-login setup failed; no release was changed")
            install(args.release, state, autostart=enabled)
        else:
            previous = json.loads((state/"last-install.json").read_text()).get("previous_release")
            if not previous:
                raise RuntimeError("No previous release recorded")
            backup = state/"previous.service"
            install(previous, state, backup.read_bytes() if backup.is_file() else None)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print("NAV startup check: " + str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
