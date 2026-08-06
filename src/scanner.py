"""
Automatic Dependent Surveillance-Broadcast (ADS-B) Scanner and Decoder


resources:
- https://mode-s.org/1090mhz/content/introduction.html
- 


"""

from rtlsdr import RtlSdr

import argparse
import queue
import sys
import threading
import time

import pyModeS as pms

import numpy as np
from numpy.typing import NDArray
from scipy.signal import firwin, lfilter, lfilter_zi, resample_poly

import constants

from aircraft import Aircraft
from decode import (
    DownLinkFormat,
    TypeCode,
    iq_to_pulses,
    pulses_to_bits,
    bits_to_int,
    find_msg_start,
    validate_crc,
)
from cpr import (
    DLAT_EVEN,
    DLAT_ODD,
    mod,
    get_nl,
)


def parse_args():
    """Optional args to make testing easier."""
    p = argparse.ArgumentParser(description="ADS-B Scanner")
    p.add_argument(
        "--gain",
        default="auto",
        help="Tuner gain: 'auto' or a number in dB (default auto)",
    )
    # TODO: add a unique flag that only shows new aircrafts???
    return p.parse_args()


def sdr_reader_thread(
    sdr,
    block_size,
    buffer,
    stop_event,
):
    """Thread task to read a specified block of IQ samples, and input into provided buffer.
    Buffer input is a blocking call such that no data is ever dropped.
    """
    while not stop_event.is_set():
        try:
            iq_samples = sdr.read_samples(block_size)
        except Exception as e:
            print(f"SDR Read error: {e}")
            # TODO: break,except, write stderr, all
            break
        buffer.put(np.asarray(iq_samples, dtype=np.complex64), block=True)


def main():
    args = parse_args()

    # initialize the RTL-SDR
    sdr = RtlSdr()
    sdr.sample_rate = constants.SAMPLE_RATE
    sdr.center_freq = constants.CENTER_FREQ
    sdr.gain = "auto" if args.gain == "auto" else float(args.gain)

    # counter of message failures
    # noisy correlator, wrong crc, etc
    msg_failures = 0

    # initialize buffer for IQ samples
    iq_block_size = constants.BLOCK_SIZE
    iq_buffer = queue.Queue(maxsize=10)
    iq_read_stop_event = threading.Event()

    # start IQ buffer
    sdr_reader = threading.Thread(
        target=sdr_reader_thread,
        args=(
            sdr,
            iq_block_size,
            iq_buffer,
            iq_read_stop_event,
        ),
        daemon=True,
    )
    sdr_reader.start()
    print("SDR thread started, listening for ADS-B on 1090MHz...\n")

    # track all incoming vehicles
    tracked_aircrafts = {}

    # loop to read from IQ buffer and decode any messages
    # for x in range(100000):
    while True:
        # iq_all = iq_buffer.get()  # TODO: put back
        iq_all = np.fromfile("iq_examples/g_adsb.iq", dtype=np.complex64)
        print("new buffer...\n")
        while len(iq_all) != 0:
            iq_data = iq_all[:500]
            iq_all = iq_all[250:]
            pulse_data = iq_to_pulses(iq_data)
            msg_idx = find_msg_start(pulse_data)

            # TODO: if msg_idx+TOTAL_MSG_LENGTH discard? and mark as false positive?
            # message would be cutoff, ignore
            if msg_idx+constants.MSG_SIZE > len(pulse_data):
                continue
            msg_pulses = pulse_data[msg_idx:msg_idx+constants.MSG_SIZE]
            msg_bits = pulses_to_bits(msg_pulses)

            # discard the preamble
            int_block = bits_to_int(msg_bits[8:])
            u = pms.decode(int_block)

            if not validate_crc(int_block):
                continue

            print(msg_bits[8:])
            print(u)

            temp_aircraft = Aircraft(squitter_bits=msg_bits, timestamp=time.time())
            icao_hex = temp_aircraft.icao_hex
            if icao_hex not in tracked_aircrafts:
                print(f"new aircraft identified: {icao_hex}")
                tracked_aircrafts[icao_hex] = temp_aircraft
            else:
                print("seen aircraft before, updating aircraft object")
                aircraft = tracked_aircrafts[icao_hex]
                aircraft.update_squitter(squitter_bits=msg_bits, timestamp=time.time())
            print("\n")


        # TODO: remove when live loop
        break

    print(tracked_aircrafts)
    t = Aircraft("111111111000110101000000011011100001000001011000000101110001011000110111010011100110000111100000111101110110001101011000")
    t.decode_airborne_position()
    time.sleep(3)
    t.update_squitter("111111111000110101000000011011100001000001011000000110010100001010010100001110111011100100010001010110111010001011010111")
    t.decode_airborne_position()
    print()
    t.dump_info()








if __name__ == "__main__":
    main()
