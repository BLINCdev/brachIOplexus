def change_scale(old_min, old_max, new_min, new_max, value):
    """
    Converts value in [old_min, old_max] proportionally to [new_min, new_max]
    """
    return (((value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min


def clamp_range(min_val, max_val, value):
    """
    Clamps value within [min_val,max_val]
    """
    return max(min_val, min(value, max_val))