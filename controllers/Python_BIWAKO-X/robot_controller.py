def omni_control_action(diff_deg, diff_distance):
    cmd = 0
    if -45.0 <= diff_deg < 45:
        cmd = 1

    elif -180.0 <= diff_deg < -135.0 or 135.0 <= diff_deg < 180.0:
        cmd = 2

    elif 45.0 <= diff_deg < 135.0:
        cmd = 3

    elif -135.0 <= diff_deg < -45.0:
        cmd = 4
    return cmd


def fixed_head_action(diff_deg, diff_distance):
    # heading_torelance = const.heading_torelance
    heading_torelance = 10
    cmd = 0
    if abs(diff_deg) < heading_torelance:
        cmd = 1
    elif diff_deg >= heading_torelance:
        cmd = 6
    elif diff_deg < -1.0 * heading_torelance:
        cmd = 5
    return cmd
