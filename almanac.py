"""
This file deals with almanac data.
'Almanac_Raw' is an object for raw almanac data. The 'set_data' method parses the raw UBX-AID-ALM message and extracts
the relevant fields as per the UBX documentation.
'Almanac_Parsed' is an object for almanac data extracted from the Almanac_Raw object as per the GPS documentation. The
'special_print' method prints all attributes and their values.
"""
import math

from config import twos_comp
from ephemeris import Word


class Almanac_Raw:
    def __init__(self) -> None:
        """
        Constructor.
        """

        self.svid = None
        self.week = 0
        self.sf_empty = True
        self.word3 = None
        self.word4 = None
        self.word5 = None
        self.word6 = None
        self.word7 = None
        self.word8 = None
        self.word9 = None
        self.word10 = None

    def set_data(self, raw_data) -> None:
        """
        Fills class with data received

        :param raw_data: bytes:
        """
        lenm = len(raw_data)
        lenp = raw_data[4:6]
        lenp = int.from_bytes(lenp, "little", signed=False)
        raw_data_payload = raw_data[6: lenm - 2]
        self.svid = int.from_bytes(raw_data_payload[0:4], "little", signed=False)
        raw_data_payload = raw_data_payload[4:]
        self.week = int.from_bytes(raw_data_payload[0:4], "little", signed=False)
        raw_data_payload = raw_data_payload[4:]
        if lenp > 8:
            self.sf_empty = False
            for i, attr in zip(range(8), ("word3",
                                          "word4",
                                          "word5",
                                          "word6",
                                          "word7",
                                          "word8",
                                          "word9",
                                          "word10")):
                setattr(self, attr,
                        Word(int.from_bytes(raw_data_payload[i*4:i*4+4], "little", signed=False)))
        else:
            self.sf_empty = True


class Almanac_Parsed:
    def __init__(self, alm_raw: Almanac_Raw) -> None:
        """
        Constructor.

        :param alm_raw: Almanac_Raw:
        """
        self.svid = alm_raw.svid
        self.svid_ext = int(alm_raw.word3.bin[2: 8], 2)
        self.e = int(alm_raw.word3.bin[8:], 2) * (2 ** -21)
        self.toa = int(alm_raw.word4.bin[:8], 2) * (2 ** 12)
        self.delta_i = twos_comp(int(alm_raw.word4.bin[8:], 2), 16) * (2 ** -19) * math.pi + 0.3 * math.pi
        self.omega_dot = twos_comp(int(alm_raw.word5.bin[:16], 2), 16) * (2 ** -38) * math.pi
        self.sqrt_a = int(alm_raw.word6.bin, 2) * (2 ** -11)
        self.omega0 = twos_comp(int(alm_raw.word7.bin, 2), 24) * (2 ** -23) * math.pi
        self.omega = twos_comp(int(alm_raw.word8.bin, 2), 24) * (2 ** -23) * math.pi
        self.m0 = twos_comp(int(alm_raw.word9.bin, 2), 24) * (2 ** -23) * math.pi
        self.af0 = twos_comp(int(alm_raw.word10.bin[:8] + alm_raw.word10.bin[19: -2], 2), 11) * (2 ** -20)
        self.af1 = twos_comp(int(alm_raw.word10.bin[8: 19], 2), 11) * (2 ** -38)

    def special_print(self) -> None:
        attr_list = [attr for attr in dir(self) if not attr.startswith('__') and not attr.startswith('special')]
        str_rtr = ''
        for attr in attr_list:
            str_rtr += attr + " = " + str(getattr(self, attr)) + "   "
        print(str_rtr[:-1])
