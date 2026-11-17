from nav_app.features.device_profile import device_profile_path


def test_unset_profiles_use_device_calibration_including_persisted_empty(tmp_path):
    for name, filename in [('fast_livo_params', 'fast_livo_params.yaml'),
                           ('fast_livo_camera_params', 'camera_pinhole.yaml'),
                           ('localizer_config', 'localizer.yaml')]:
        path = tmp_path/filename
        assert device_profile_path(name, '', tmp_path) == ''
        path.write_text('calibration fixture')
        for saved in ('', ' ', None):
            assert device_profile_path(name, saved, tmp_path) == str(path)
        assert device_profile_path(name, '/explicit/custom.yaml', tmp_path) == '/explicit/custom.yaml'
    assert device_profile_path('nav2_params', '', tmp_path) == ''
