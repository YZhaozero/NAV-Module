"""Resolve an unset profile to this device's installed calibration."""
from pathlib import Path


def device_profile_path(name, saved, directory=None):
    if str(saved or '').strip():
        return str(saved)
    files = {'localizer_config': 'localizer.yaml',
             'fast_livo_params': 'fast_livo_params.yaml',
             'fast_livo_camera_params': 'camera_pinhole.yaml'}
    if name not in files:
        return ''
    path = Path(directory or Path.home()/'.config/nav')/files[name]
    return str(path) if path.is_file() else ''
