"""
compact position reporting

https://shemesh.larc.nasa.gov/fm/papers/VSTTE2017-draft.pdf
"""

import numpy as np

NZ = 15
"""num of latitude zones between equator and pole in Mode S"""

DLAT_EVEN = 360 / (4 * NZ)
"""latitude zone sizes for an even message"""

DLAT_ODD = 360 / (4 * NZ - 1)
"""latitude zone sizes for an odd message"""


def get_nl(latitude) -> int:
    """
    math to get number of longitude zones
    """
    latitude = np.abs(latitude)

    # at the equator
    if latitude == 0:
        return 59

    # at a pole
    if latitude > 87:
        return 1

    # close to a pole
    if latitude == 87:
        return 2

    # TODO: have a faster lookup table?
    # everything else in between will be calculated on the spot
    numerator = 1 - np.cos(np.pi / (2 * NZ))
    denominator = (np.cos(np.deg2rad(latitude))) ** 2
    acos_input = 1 - (numerator / denominator)

    # bound values to avoid floating point math edgecases
    acos_input = max(-1, min(1, acos_input))

    # reconvert radian output back to degrees
    return np.floor((2 * np.pi) / np.arccos(acos_input)).astype(int)


def mod(x, y):
    """math variation"""
    return x - (y * np.floor(x / y))
