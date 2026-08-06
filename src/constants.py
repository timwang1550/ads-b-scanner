"""
u know
"""


import numpy as np

CENTER_FREQ = 1090e6  # worldwide transmit standard is 1090 MHz
SAMPLE_RATE = 2e6  # supports 1Mbit/s datarate
SAMPLES_PER_US = 2  # 2MHz = 2 samples per microsecond

PREAMBLE_US = 8  # microsec
PREAMBLE_LENGTH = PREAMBLE_US * SAMPLES_PER_US

SHORT_SQUITTER_US = 56  # microsec
LONG_SQUITTER_US = 112  # microsec

# preamble has a pulse at the 0, 1, 3.5, and 4.5 us mark
# each pulse is 0.5us wide
PREAMBLE = np.zeros(PREAMBLE_LENGTH, dtype=np.float32)
PREAMBLE = np.array([
    1.0, 0.0,  # 0 
    1.0, 0.0,  # 1
    0.0, 0.0,
    0.0, 1.0,  # 3.5
    0.0, 1.0,  # 4.5
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
])  # TODO: maybe actually use the constants? idk 1010000101000000
# pream = [1 1 x 0 0 x x x]

MSG_SIZE = (PREAMBLE_US + LONG_SQUITTER_US) * SAMPLES_PER_US  # samples
BLOCK_SIZE = 256 * 1024  # samples
