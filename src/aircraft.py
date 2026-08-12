""" """

import numpy as np
import time

from decode import (
    TypeCode,
    bits_to_int,
    LargeAircraftCategory,
    ADSBException,
)

from cpr_utils import (
    DLAT_EVEN,
    DLAT_ODD,
    mod,
    get_nl,
)


class Aircraft:
    """Class to represent an aircraft."""

    msg_timestamp = -1
    squitter_bits = ""
    df = ""
    ca = ""
    icao = ""
    me = ""
    tc = ""
    pi = ""

    callsign = ""
    aircraft_type = ""

    # CPR registers + flags
    cpr_even_timestamp: float = -1.0  # unit time
    cpr_lat_even: float = -1.0
    cpr_lon_even: float = -1.0
    cpr_even_valid: bool = False

    cpr_odd_timestamp: float = -1.0  # unix time
    cpr_lat_odd: float = -1.0
    cpr_lon_odd: float = -1.0
    cpr_odd_valid: bool = False

    real_lat: float = -1.0
    real_lon: float = -1.0
    real_lat_valid: bool = False
    real_lon_valid: bool = False

    # Altitude registers + flags
    altitude: float = -1.0

    # Velocity registers + flags
    vertical_velocity: float = -1.0  # ft/min
    x_velocity: float = -1.0  # kt
    y_velocity: float = -1.0  # kt
    horizontal_velocity: float = -1.0  # kt
    track_angle: float = -1.0  # deg [0,360)

    def __init__(self, squitter_bits: str, timestamp: float | None = None):
        """Squitter Bits withouthe preamble?"""
        self.squitter_bits = squitter_bits
        self.msg_timestamp = timestamp if timestamp else time.time()

        if self.squitter_bits:
            self.parse_message(self.squitter_bits)

    def update_squitter(self, squitter_bits, timestamp: float | None = None):
        """update and reparse?"""
        self.squitter_bits = squitter_bits
        self.msg_timestamp = timestamp if timestamp else time.time()

        self.parse_message(self.squitter_bits)
        # TODO: check icao matches, before updating


    @staticmethod
    def is_adsb(msg_bits: str) -> bool:
        """Pull downlink format without parsing entire message.
        Return true if ADS-B squitter.
        """
        df_format = bits_to_int(msg_bits[:5])
        return df_format == 17  # TODO: constant this?


    @staticmethod
    def is_large_aircraft(msg_bits: str) -> bool:
        """Pull aircraft type without parsing entire message.
        Return true if a large vehicle.
        """
        pass # TODO: finish

    @staticmethod
    def get_icao(msg_bits: str) -> str:
        """Pull ICAO hex without parsing entire message."""  # TODO: update
        return f"{bits_to_int(msg_bits[8:32]):X}"


    # TODO: discard preamble
    def parse_message(self, msg):
        """Split message into fundamental blocks, then decide how to process message"""
        _ = msg[0:8]  # 8 bit preamble, discarded

        self.df = bits_to_int(msg[8:13])  # 5 bit downlink format
        self.ca = bits_to_int(msg[13:16])  # 3 bit transponder capability
        self.icao = bits_to_int(msg[16:40])  # 24 bit ICAO aircraft address
        self.me = msg[40:96]  # 56 bit message payload
        self.data_bits = msg[48:96]  # 48 bit message payload without type code
        self.tc = bits_to_int(msg[40:45])  # 8 bit type code (inside payload)
        self.surveillance_bits = msg[45:47]
        # self.antenna_flag =
        self.tc_total = msg[40:48]
        self.pi = msg[96:119]  # 24 bit crc parity check

        # decode message based on type code
        self.decode_msg()


    @property  # TODO move all properties make all
    def icao_hex(self):
        """Aircraft's ICAO designator as a hexadecimal string"""
        return f"{self.icao:X}"


    def decode_msg(self):
        """Decide how to decode a message based on its typecode."""
        if self.tc in TypeCode.AIRCRAFT_ID:
            self.decode_callsign()
            self.decode_aircraft_type()
        elif self.tc in TypeCode.SURFACE_POS:
            pass  # currently ignoring surface velocity
        elif self.tc in TypeCode.AIR_POS_BARO or self.tc in TypeCode.AIR_POS_GNSS:
            self.decode_airborne_position()
        elif self.tc in TypeCode.AIR_VEL:
            self.decode_velocity()
        else:
            # print(f"not doing anything with this type code yet {self.tc}")
            pass
            # so far 28,29,31


    def dump_info(self):  # TODO eventually right shift everything to be fancy and nice
        """dump what you know"""
        print(f"\033[KICAO:\t\t{self.icao:X}")
        print(f"\033[KCallsign:\t{self.callsign}")
        print(f"\033[KAircraft Type:\t{self.aircraft_type}")
        print(f"\033[KAltitude:\t{self.altitude if self.altitude != -1 else ''}")
        print(f"\033[KLatitude:\t{round(self.real_lat, 4) if self.real_lat_valid else ''}")
        print(f"\033[KLongitude:\t{round(self.real_lon, 4) if self.real_lon_valid else ''}")


    def decode_callsign(self):
        """decode callsign"""
        CALLSIGN_CHAR_LUT = (
            "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"
        )

        # reset callsign and regenerate from char lookup table
        self.callsign = ""
        for i in range(0, 48, 6):
            char_idx = bits_to_int(self.data[i : i + 6])
            self.callsign += CALLSIGN_CHAR_LUT[char_idx]

        return self.callsign

    def decode_aircraft_type(self):
        """decode what kinda vehicle"""
        tc = bits_to_int(self.me[:5])  # first 8b its
        ca = bits_to_int(self.me[5:8])
        self.aircraft_type = LargeAircraftCategory[ca]


    def decode_airborne_position(self):
        """ """
        altitude_bits = self.data_bits[:12]
        alt_t_bit = self.data_bits[
            12
        ]  # single antenna flag # TODO: figure out how to use
        cpr_frame_bit = bits_to_int(self.data_bits[13])
        cpr_lat = bits_to_int(self.data_bits[14:31]) / 2**17
        cpr_lon = bits_to_int(self.data_bits[31:]) / 2**17

        # TODO: do something about altitude bits here
        if self.tc in TypeCode.AIR_POS_BARO:
            self.get_barometric_altitude(altitude_bits)

        if self.tc in TypeCode.AIR_POS_GNSS:
            self.get_gnss_altitude(altitude_bits)

        # even frame = 0, odd frame = 1
        if cpr_frame_bit:
            self.cpr_lat_odd = cpr_lat
            self.cpr_lon_odd = cpr_lon
            self.cpr_odd_timestamp = self.msg_timestamp
            self.cpr_odd_valid = True
            # print(f"{[self.icao]} odd frame")
        else:
            self.cpr_lat_even = cpr_lat
            self.cpr_lon_even = cpr_lon
            self.cpr_even_timestamp = self.msg_timestamp
            self.cpr_even_valid = True
            # print(f"{[self.icao]} even frame")

        # if both frames are valid, attempt to decode real position
        if self.cpr_even_valid and self.cpr_odd_valid:
            # print("new valid cpr frame, try to decode")
            self.get_latitude()
            self.get_longitude()
            # print(self.real_lat, self.real_lon) # TODO: debug
            # reset the unused lat & lon so next valid frame can be calculated

            # clear the oldest cpr message
            self.reset_oldest_cpr()


    def get_latitude(self):
        """"""
        if not self.cpr_even_valid or not self.cpr_odd_valid:
            raise ADSBException("Latitude decoding requires even and odd frames!""")

        # calculate latitude zone index
        j = np.floor(59 * self.cpr_lat_even - 60 * self.cpr_lat_odd + 0.5)
        lat_even = DLAT_EVEN * (mod(j, 60) + self.cpr_lat_even)
        lat_odd = DLAT_ODD * (mod(j, 59) + self.cpr_lat_odd)

        # check if both latitude coords are in the same zone
        if get_nl(lat_even) != get_nl(lat_odd):
            print(
                "DEBUG: latitudes are part of different index, messages are too far apart"
            )
            self.reset_oldest_cpr()
            return

        # select latitude from the more recent of the two messages
        if self.cpr_even_timestamp > self.cpr_odd_timestamp:
            # print("selecting even latitude") # TODO: turn this into debug
            self.real_lat = lat_even
        else:
            # print("selecting odd latitude")
            self.real_lat = lat_odd

        # normalize to [-90, 0] for southern hemisphere coordinates
        if self.real_lat >= 270:
            self.real_lat -= 360

        self.real_lat_valid = True
        return self.real_lat


    def get_longitude(self):
        """Requires a valid lat to make a lon?"""
        if not self.real_lat_valid:
            raise ADSBException("Longitude decoding requires valid latitude!""")

        # calculate longitude zone index
        nl = get_nl(self.real_lat)
        m = np.floor(
            self.cpr_lon_even * (nl - 1) - (self.cpr_lon_odd * nl) + 0.5
        ).astype(int)

        n_even = max(nl, 1)
        n_odd = max(nl - 1, 1)

        dlon_even = 360 / n_even
        dlon_odd = 360 / n_odd

        lon_even = dlon_even * (mod(m, n_even) + self.cpr_lon_even)
        lon_odd = dlon_odd * (mod(m, n_odd) + self.cpr_lon_odd)

        # select latitude from the more recent of the two messages
        if self.cpr_even_timestamp > self.cpr_odd_timestamp:
            # print(f"{[self.icao]}selecting even longitude")  # TODO: turn into debug
            self.real_lon = lon_even
        else:
            # print(f"{[self.icao]}selecting odd longitude")
            self.real_lon = lon_odd

        # normalize to [-180, 180] for aviation convention
        if self.real_lon >= 180:
            self.real_lon -= 360

        self.real_lon_valid = True
        return self.real_lon


    def reset_oldest_cpr(self):
        """Reset CPR valid flag on oldest message."""
        if self.cpr_even_timestamp > self.cpr_odd_timestamp:
            # print(f"{[self.icao]}reset odd frame")
            self.cpr_odd_timestamp = False
        else:
            # print(f"{[self.icao]}reset even frame")
            self.cpr_even_timestamp = False


    def get_barometric_altitude(self, altitude_bits):
        """"""
        if bits_to_int(altitude_bits) == 0:
            print("invalid altitude, skipping?")
            return

        q_bit = bits_to_int(altitude_bits[8])  # 8th bit
        if q_bit:  # Q=1, 25ft increment
            remainder = bits_to_int(altitude_bits[:8] + altitude_bits[9:])
            self.altitude = remainder * 25 - 1000
            # print(f"baro alt {self.altitude}")  # TODO: debug
            return self.altitude
        else:  # Q=0, 100ft increment
            # print("havent implemented the graycode version")
            pass # TODO: add these


    def get_gnss_altitude(self, altitude_bits):
        self.altitude = bits_to_int(altitude_bits)
        # print(f"gnss alt {self.altitude}")  # TODO: debug
        return self.altitude
        # TODO: can we valid this? lets jsut work with the baro altitude


    def decode_velocity(self):
        """assumes message is from a subsonic aircraft"""
        subtype = self.me[5:8]
        intent_flag = self.me[8]
        ifr_flag = self.me[9]
        nav_uncertainty = self.me[10:13]
        horizontal_velocity = self.me[13:35]
        vertical_source = self.me[35]  # 0 gnss 1 for baro
        vertical_direction = self.me[36]  # 0 up 1 down
        vertical_rate = bits_to_int(self.me[37:46])
        reserved = self.me[46:48]  # reserve, del
        gnss_baro_direction = self.me[48]  # 0 gnss above baro, 1 gnss below baro
        gnss_baro_diff = self.me[49:]

        # decode subtype
        subtype = bits_to_int(self.me[5:8])
        if subtype in (3,4):
            return  # ignore GNSS-based speeds for now
        subtype_factor = 1 if (subtype == 1) else 4

        # decode vertical speed if information is provided
        if vertical_rate != 0:
            sign = 1 if vertical_direction == 0 else -1
            self.vertical_velocity = (sign*64) * (vertical_rate - 1)

        # decode east-west (x) horizontal speed
        x_direction = bits_to_int(self.me[13])
        x_speed = bits_to_int(self.me[14:24])
        x_dir = -1 if x_direction else 1
        self.x_velocity = (
            (x_dir*subtype_factor) * (x_speed - 1)
        )

        # decode north-south (y) horizontal speed
        y_direction = bits_to_int(self.me[24])
        y_speed = bits_to_int(self.me[25:35])
        y_dir = -1 if y_direction else 1
        self.y_velocity = (
            (y_dir*subtype_factor) * (y_speed - 1)
        )

        # decode final ground speed
        self.horizontal_velocity = (
            np.sqrt(self.x_velocity**2 + self.y_velocity**2)
        )

        # decode final track angle (clockwise from north)
        track_angle = (np.atan2(self.x_velocity, self.y_velocity) * 360) / (2*np.pi)
        self.track_angle = mod(track_angle, 360)


        # TODO: debug msg
        # TODO, convert everything to m/s?
        # TODO: dont display vertical if its -1
        # print(f"{self.vertical_velocity} ft/min, {self.horizontal_velocity} kt")
        # print(f"final x and y components {self.x_velocity} kt, {self.y_velocity} kt")
        # print(f"{self.track_angle} deg from north")


