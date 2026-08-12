"""
Automatic Dependent Surveillance-Broadcast (ADS-B) Scanner and Decoder


resources:
- https://mode-s.org/1090mhz/content/introduction.html
- 


"""

from rtlsdr import RtlSdr

import argparse
import queue
import threading
import time
import sys

import pyModeS as pms  #  TODO: delete everntually

import numpy as np

import constants

from aircraft import Aircraft
from decode import (
    iq_to_pulses,
    pulses_to_bits,
    bits_to_int,
    find_msg_start,
    validate_crc,
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

    # track aircafts by ICAO
    tracked_aircrafts = {}
    track_count = 0

    # store ICAO of non-aircraft vehicles to make filtering easier
    not_aircrafts = {}

    # loop to read from IQ buffer and decode any messages
    while True:
        # iq_all = iq_buffer.get()  # TODO: put back
        iq_all = np.fromfile("iq_examples/g_adsb.iq", dtype=np.complex64)  # imagine this as a single block
        # print("new buffer...\n")
        while len(iq_all) != 0:
            iq_data = iq_all[:500]
            iq_all = iq_all[250:]
            pulse_data = iq_to_pulses(iq_data)
            msg_idx = find_msg_start(pulse_data)

            # if message > remaining block, false positive found
            if msg_idx+constants.MSG_SIZE > len(pulse_data):
                continue

            # convert pulses to binary
            msg_pulses = pulse_data[msg_idx:msg_idx+constants.MSG_SIZE]
            msg_bits = pulses_to_bits(msg_pulses)

            # discard the preamble
            int_block = bits_to_int(msg_bits[8:])
            u = pms.decode(int_block)  # TODO: delete eventually

            # if message crc fails, too noisy to use or false positive found
            if not validate_crc(int_block):
                continue

            # not ADS-B format message, ignore
            if not Aircraft.is_adsb(msg_bits[8:]):
                continue

            # do a check if message is from aircraft, if not blacklist it

            # print(u)
            # print(Aircraft.get_icao(msg_bits[8:]))  # TODO: discard preamble
            icao_hex = Aircraft.get_icao(msg_bits[8:])
            if icao_hex not in tracked_aircrafts:
                # print(f"new aircraft identified: {icao_hex}")
                tracked_aircrafts[icao_hex] = Aircraft(squitter_bits=msg_bits, timestamp=time.time())
            else:
                # print("seen aircraft before, updating aircraft object")
                aircraft = tracked_aircrafts[icao_hex]
                aircraft.update_squitter(squitter_bits=msg_bits, timestamp=time.time())


            # print all known aircraft, deleting old data
            for _ in range(track_count):
                for _ in range(4):
                    sys.stdout.write('\x1b[1A')  # move cursor up one line
                    sys.stdout.write('\x1b[2K')  # clear the entire line
            sys.stdout.flush()

            track_count = 0
            for a in tracked_aircrafts.values():
                if a.real_lat_valid:
                    print("_______________________________")
                    print(f"ICAO:\t\t{a.icao:X}")
                    print(f"Latitude:\t{round(a.real_lat, 4) if a.real_lat_valid else ''}")
                    print(f"Longitude:\t{round(a.real_lon, 4) if a.real_lon_valid else ''}")
                    track_count += 1

        break # TODO: remove when live loop

    # for a in tracked_aircrafts.values():
    #     print("\033[F"*8)
    #     print("\033[K_______________________________")
    #     a.dump_info()
    #     time.sleep(3)









if __name__ == "__main__":
    main()
    # msg_bits = "111111111000110101001000010100000010000010011001010001000000100110010100000010000011100000010111010110110010100001001111"
    # a = Aircraft(squitter_bits=msg_bits, timestamp=time.time())
    # a.dump_info()



# TODO: make a dependency toml
# TODO: ruff check --select F401 --fix .