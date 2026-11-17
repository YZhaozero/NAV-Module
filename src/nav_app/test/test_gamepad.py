"""Guided controller mapping and saved profile boundaries."""
import json
import pytest
from nav_app.features.gamepad import Calibration, parse_devices, load_profiles, save_profile
from nav_app.features.manual_drive import joystick_input


def test_parse_ros_device_identity_and_name():
    output = "ID : GUID : GamePad : Mapped : Joystick Device Name\n-------\n 2 : "+"a"*32+" : true : false : Xbox: USB Controller\n"
    assert parse_devices(output) == [dict(id=2, guid="a"*32, name="Xbox: USB Controller")]
    assert parse_devices("ID : GUID : GamePad : Mapped : Joystick Device Name\n-------\n") == []
    with pytest.raises(ValueError):parse_devices("driver failed")


@pytest.mark.parametrize("chassis", ["ackermann", "omni"])
def test_wizard_learns_button_polarity_and_distinct_axes(chassis, tmp_path):
    wizard = Calibration(chassis)
    now = 10.
    center = (.02, -.03, 0., 0.)
    wizard.center((center, (0, 0, 0), now), now)
    wizard.update((center, (1, 1, 0), now), now)
    assert wizard.stage == "button" and wizard.deadman is None
    wizard.update((center, (0, 0, 1), now), now)
    assert wizard.deadman == 2 and wizard.stage == "release_button"
    wizard.update((center, (0, 0, 0), now), now)
    movements = [(1, -1), (0, 1)] + ([(3, -1)] if chassis == "omni" else [])
    for index, sign in movements:
        values = list(center);values[index] = sign
        now += .1
        wizard.update((values, (0, 0, 0), now), now)
        now += .25
        wizard.update((values, (0, 0, 0), now), now)
        assert wizard.stage == "return"
        now += .1
        wizard.update((center, (0, 0, 0), now), now)
    profile = wizard.profile(dict(chassis=chassis, linear=.15, angular=.3, name="Xbox"))
    path = tmp_path/"gamepads.json"
    save_profile(path, "a"*32, profile)
    saved = load_profiles(path)["a"*32]
    assert saved["axes"] == [1, 3 if chassis == "omni" else -1, 0]
    assert saved["signs"] == [-1, -1 if chassis == "omni" else 1, 1]
    values, held = joystick_input((center, (0, 0, 0), now), now, chassis,
        saved["axes"], saved["deadman"], signs=saved["signs"], centers=saved["centers"])
    assert values == (0, 0, 0) and not held
    moved = list(center);moved[1] = -.9
    values, held = joystick_input((moved, (0, 0, 1), now), now, chassis,
        saved["axes"], saved["deadman"], signs=saved["signs"], centers=saved["centers"])
    assert values[0] > 0 and held


def test_wizard_rejects_stale_nonfinite_and_held_center():
    wizard = Calibration("ackermann")
    for sample in [None, ((0., 0.), (0,), 8.), ((float("nan"), 0.), (0,), 10.), ((0., 0.), (1,), 10.)]:
        with pytest.raises(ValueError):wizard.center(sample, 10.)
    with pytest.raises(ValueError):wizard.profile(dict(chassis="ackermann"))


def test_axis_noise_diagonals_and_reused_axis_do_not_finish_step():
    wizard = Calibration("ackermann")
    wizard.center(((0., 0.), (0,), 10.), 10.)
    wizard.update(((0., 0.), (1,), 10.), 10.)
    wizard.update(((0., 0.), (0,), 10.), 10.)
    for values in ((.1, 0.), (.8, .8)):
        wizard.update((values, (0,), 10.), 10.)
        wizard.update((values, (0,), 10.3), 10.3)
        assert wizard.stage == "axis"
    wizard.update(((1., 0.), (0,), 11.), 11.)
    wizard.update(((1., 0.), (0,), 11.3), 11.3)
    wizard.update(((0., 0.), (0,), 11.4), 11.4)
    wizard.update(((1., 0.), (0,), 11.5), 11.5)
    wizard.update(((1., 0.), (0,), 11.8), 11.8)
    assert wizard.stage == "axis" and wizard.axes == [0, -1, -1]


def test_invalid_saved_profile_is_reported_and_not_overwritten(tmp_path):
    path = tmp_path/"gamepads.json"
    path.write_text("{broken")
    with pytest.raises(ValueError):load_profiles(path)
    assert path.read_text() == "{broken"
    path.write_text(json.dumps({"bad-guid": {}}))
    with pytest.raises(ValueError):load_profiles(path)


def test_live_axis_count_must_match_saved_centers():
    with pytest.raises(ValueError, match="轴数量"):
        joystick_input(((0., 0.), (0,), 10.), 10., "ackermann", (1, -1, 0), 0, centers=(0., 0., 0.))
