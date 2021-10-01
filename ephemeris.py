"""
This file deals with ephemeris data.
'Ephemeris_Raw' is an object for raw ephemeris data. The 'set_data' method parses the raw UBX-AID-EPH message and
extracts the relevant fields as per the UBX documentation.
'Ephemeris_Parsed' is an object for ephemeris data extracted from the Ephemeris_Raw object as per the GPS documentation.
The 'special_print' method prints all attributes and their values.
"""
import math

from config import twos_comp


class Ephemeris_Raw:
    def __init__(self) -> None:
        """
        Constructor.

        NB: u-blox receiver only transmits subframes 1 through 3.
        """

        self.svid = None
        self.how = None
        self.sf_empty = True
        self.sf1 = Subframe()
        self.sf2 = Subframe()
        self.sf3 = Subframe()

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
        self.how = Word(int.from_bytes(raw_data_payload[0:4], "little", signed=False))
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
                setattr(self.sf1, attr,
                        Word(int.from_bytes(raw_data_payload[i*4:i*4+4], "little", signed=False)))
                setattr(self.sf2, attr,
                        Word(int.from_bytes(raw_data_payload[32 + i*4:32 + i*4 + 4], "little", signed=False)))
                setattr(self.sf3, attr,
                        Word(int.from_bytes(raw_data_payload[64 + i*4:64 + i*4 + 4], "little", signed=False)))
        else:
            self.sf_empty = True
                

class Subframe:
    def __init__(self) -> None:
        """
        Constructor.

        NB: u-blox receiver only transmits words 3 through 10 in of subframes 1 through 3.
        """

        self.word3 = None
        self.word4 = None
        self.word5 = None
        self.word6 = None
        self.word7 = None
        self.word8 = None
        self.word9 = None
        self.word10 = None


class Word:
    def __init__(self, value=None) -> None:
        """
        Constructor.

        Decimal and binary (24-bit) value of the word.
        """
        self.dec = value
        self.bin = bin(value)[2:].zfill(24) if isinstance(value, int) else None


class Ephemeris_Parsed:
    def __init__(self, eph_raw: Ephemeris_Raw) -> None:
        """
        Constructor.

        :param eph_raw: Ephemeris_Raw:
        """
        self.svid = eph_raw.svid
        self.tgd = twos_comp(int(eph_raw.sf1.word7.bin[16:], 2), 8) * (2 ** -31)
        self.iodc = int(eph_raw.sf1.word3.bin[22:] + eph_raw.sf1.word8.bin[:8], 2)
        self.toc = int(eph_raw.sf1.word8.bin[8:], 2) * (2 ** 4)
        self.af2 = twos_comp(int(eph_raw.sf1.word9.bin[:8], 2), 8) * (2 ** -55)
        self.af1 = twos_comp(int(eph_raw.sf1.word9.bin[8:], 2), 16) * (2 ** -43)
        self.af0 = twos_comp(int(eph_raw.sf1.word10.bin[:22], 2), 22) * (2 ** -31)
        self.iode = int(eph_raw.sf2.word3.bin[:8], 2)  # Found in two different subframes for some reason ...
        self.crs = twos_comp(int(eph_raw.sf2.word3.bin[8:], 2), 16) * (2 ** -5)
        self.delta_n = twos_comp(int(eph_raw.sf2.word4.bin[:16], 2), 16) * (2 ** -43) * math.pi
        self.m0 = twos_comp(int(eph_raw.sf2.word4.bin[16:] + eph_raw.sf2.word5.bin, 2), 32) * (2 ** -31) * math.pi
        self.cuc = twos_comp(int(eph_raw.sf2.word6.bin[:16], 2), 16) * (2 ** -29)
        self.e = int(eph_raw.sf2.word6.bin[16:] + eph_raw.sf2.word7.bin, 2) * (2 ** -33)
        self.cus = twos_comp(int(eph_raw.sf2.word8.bin[:16], 2), 16) * (2 ** -29)
        self.sqrt_a = int(eph_raw.sf2.word8.bin[16:] + eph_raw.sf2.word9.bin, 2) * (2 ** -19)
        self.toe = int(eph_raw.sf2.word10.bin[:16], 2) * (2 ** 4)
        self.cic = twos_comp(int(eph_raw.sf3.word3.bin[:16], 2), 16) * (2 ** -29)
        self.omega0 = twos_comp(int(eph_raw.sf3.word3.bin[16:] + eph_raw.sf3.word4.bin, 2), 32) * (2 ** -31) * math.pi
        self.cis = twos_comp(int(eph_raw.sf3.word5.bin[:16], 2), 16) * (2 ** -29)
        self.i0 = twos_comp(int(eph_raw.sf3.word5.bin[16:] + eph_raw.sf3.word6.bin, 2), 32) * (2 ** -31) * math.pi
        self.crc = twos_comp(int(eph_raw.sf3.word7.bin[:16], 2), 16) * (2 ** -5)
        self.omega = twos_comp(int(eph_raw.sf3.word7.bin[16:] + eph_raw.sf3.word8.bin, 2), 32) * (2 ** -31) * math.pi
        self.omega_dot = twos_comp(int(eph_raw.sf3.word9.bin, 2), 24) * (2 ** -43) * math.pi
        self.idot = twos_comp(int(eph_raw.sf3.word10.bin[8:-2], 2), 14) * (2 ** -43) * math.pi

    def special_print(self) -> None:
        attr_list = [attr for attr in dir(self) if not attr.startswith('__') and not attr.startswith('special')]
        str_rtr = ''
        for attr in attr_list:
            str_rtr += attr + " = " + str(getattr(self, attr)) + "   "
        print(str_rtr[:-1])
