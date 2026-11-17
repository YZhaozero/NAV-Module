"""Explicit installation assumptions; live UTC/freshness/interpolation remain mandatory."""
import math


def alignment_configuration_ready(data):
    if not isinstance(data, dict):
        return False
    if data.get('time_verified') is True and data.get('lever_arm_verified') is True:
        return True
    # This policy is opt-in per installation. It never turns an unknown lever
    # arm into a measured one, or accepts receive-time stamped GNSS messages.
    bound = data.get('antenna_uncertainty_m')
    return (data.get('timing_policy') == 'receiver_utc_interpolated_odom'
            and data.get('installation_policy') == 'centimetre_offset_approximation'
            and isinstance(data.get('installation_source'), str)
            and bool(data['installation_source'].strip())
            and type(bound) in (int, float) and math.isfinite(bound) and 0 < bound <= .1
            and data.get('antenna_in_body_m') == [0., 0., 0.]
            and data.get('gnss_to_odom_offset_s') == 0.
            and data.get('odom_child_frame') in ('body', 'base_link'))
