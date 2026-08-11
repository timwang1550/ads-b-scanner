"""
everything related to decoding raw data
"""


import numpy as np

import constants


class ADSBException(Exception):
    """Generic exception in decoding ADS-B telemetry."""


class DownLinkFormat:
    """"""
    ALLCALL_REPLY = 4  # just ICAO address for radar acquisition
    EXTENDED_SQUITTER = 17  # ADS-B from transponder-equipped aircraft


class TypeCode:
    """"""
    AIRCRAFT_ID = (
        1,
        2,
        3,
        4,
    )  # aircraft identification
    SURFACE_POS = (
        5,
        6,
        7,
        8,
    )  # surface position
    AIR_POS_BARO = (
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        18,
    )  # airborne position using barometer
    AIR_VEL = (19,)  # airborne velocities
    AIR_POS_GNSS = (
        20,
        21,
        22,
    )  # airborne position using GNSS
    AIRCRAFT_STATUS = (28,)  # aircraft status
    TARGET_STATE = (29,)  # TODO: do this
    OPERATION_STATUS = (31,)  # aircraft operation status


class AircraftCategory:
    """"""
    # 2 are surface, 3 are small amatuer
    # only looking to track large vehicles
    DECODER = {
        4: {
            1: "Light",  # <7,000 kg
            2: "Medium 1",  # 7,000-34,000 kg
            3: "Medium 2",  # 7,000-136,000 kg
            4: "High Vortex Aircraft",
            5: "Heavy",  # >136,000 kg
            6: "High Performance",  # >5g accel, >400kt
        },
    }


def iq_to_pulses(iq_block):
    """Convert a list of IQ samples to pulses by getting magnitude of each IQ sample.
    Phase information can be discarded.
    """
    return np.abs(iq_block)


def pulses_to_bits(pulse_block):
    """Convert a list of pulses to binary by comparying magnitude of first vs second slot in every 1us.
    0.5us pulse followed by empty slot = 1
    empty slot follower by 0.5us pulse = 0
    """
    if len(pulse_block) % 2 != 0:
        raise ADSBException(f"number of pulses ({len(pulse_block)}) is not a multiple of 2!")

    bits = ""
    for idx in range(0, len(pulse_block), 2):  # iterate by 2
        first_slot = pulse_block[idx]
        second_slot = pulse_block[idx+1]

        bits += "1" if (first_slot > second_slot) else "0"

    return bits


def bits_to_int(bit_block):
    """convert big endian binary string into integer"""
    bit_num = 0
    for pow_of_2, bit in enumerate(bit_block[::-1]):
        bit_num += int(bit) * (2**pow_of_2)

    return bit_num


def find_msg_start(pulse_block):
    """Cross correlate the list of pulse samples with the message preamble.
    Return the index of highest correlation
    """
    correlation = np.correlate(pulse_block, constants.PREAMBLE, mode="valid")
    return np.argmax(correlation)


def validate_crc(msg_block):
    """
    return true or false"""
    msg_block = int(msg_block)

    GENERATOR = int("1111111111111010000001001", 2)

    for _ in range(88):
        if msg_block & 1:
            msg_block ^= GENERATOR
        msg_block >>= 1

    return(msg_block == 0)