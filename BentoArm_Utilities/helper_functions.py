import numpy as np

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


def checksum_fcn(packet):
    """The checksum is calculated via summing up all the bytes and taking the bitwise complement (~/not operator) of
    the sum cast to an uint8 / char / 8 bits, the header (first two bytes being 0xFF) and checksum (last bytes) are not
    used for the checksum"""
    summed_packet = sum(packet)
    checksum = ~np.uint8(summed_packet)
    return checksum


def fill_state(state):
    """If a state of 4 values is passed, it will fill in all the fixed state values of 0"""
    # Default full state value
    if len(state) == 9:
        return state
    # Else append 4 joint state to nine joint state to account for fixed joints
    for i in range(3):
        state.insert(0, None)
    for i in range(2):
        state.append(None)
    return state
