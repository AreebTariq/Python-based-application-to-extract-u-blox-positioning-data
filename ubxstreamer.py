"""
The UBX Streamer is the central piece of this app.
Its purpose is to establish a two-way communication channel, with the u-blox receiver.
The Streamer manages the serial connection via the 'connect' and 'disconnect' methods.
The Streamer manages a thread to send and receive data via the 'start_read_thread' and 'stop_read_thread' methods.
The Streamer sends data via the 'send' method and parses the data received via the '_read_thread' method (both require a
thread to be running).
"""
# Imports needed for this project
from io import BufferedReader
from threading import Thread
from time import sleep
from pyubx2.ubxreader import UBXReader
from serial import Serial, SerialException, SerialTimeoutException
import pyubx2.exceptions as ube
from almanac import Almanac_Raw, Almanac_Parsed
from config import C
from ephemeris import Ephemeris_Raw, Ephemeris_Parsed
from position import get_wgs84_sat_position, xyz_to_latlongalt, XYZPosition
from datetime import datetime
import time
from pyubx2.ubxhelpers import (
    itow2utc,
)


class UBXStreamer:
    """
    UBXStreamer class.
    """

    def __init__(self, port, baudrate, timeout=5):
        """
        Constructor.
        """

        self._serial_object = None
        self._serial_thread = None
        self._ubxreader = None
        self._connected = False
        self._reading = False
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self.almanac_raw = {i: Almanac_Raw() for i in range(32)}  # Almanac_Raw object for each GPS Satellite
        self.almanac_parsed = {i: None for i in range(32)}  # Almanac_Parsed object for each GPS Satellite
        self.ephemeris_raw = {i: Ephemeris_Raw() for i in range(32)}  # Ephemeris_Raw object for each GPS Satellite
        self.ephemeris_parsed = {i: None for i in range(32)}  # Ephemeris_Parsed object for each GPS Satellite
        self.pseudorange = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.snr = {i: None for i in range(32)}  # Signal to Noise Ration for each GPS Satellite
        self.sat_position = {i: None for i in range(32)}  # Satellite position in the sky for each GPS Satellite
        self.clockBias_dist = 0  # Clock bias (as a distance in meters)
        self.receiver_time = 0  # Receiver time in seconds
        self.pseudorange_residual = {i: None for i in range(32)}
        self.pseudorange_residual_from_svinfo = {i: None for i in range(32)}
        self.pseudorange_correction = {i: None for i in range(32)}
        self.pseudorange_correction_from_slas = {i: None for i in range(32)}
        self.pseudorange_correction_from_sbas = {i: None for i in range(32)}
        self.pseudorange_rate_correction = {i: None for i in range(32)}
        self.pseudorange_rms_error_index = {i: None for i in range(32)}
        self.svid_rawx = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.pseudorange_rawx = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.pseudorange_sd_estimate_rawx = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.carrierphase_rawx = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.carrierphase_sd_estimate_rawx = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.carrierphase_locktime = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.measurements_rawx = 0  # Pseudorange for each GPS Satellite
        self.klobuchar_alpha = {i: None for i in range(32)}  # Klobuchar alpha information
        self.klobuchar_beta = {i: None for i in range(32)}  # Klobuchar beta information
        self.x_coordinates = {i: None for i in range(32)}  # estimated_X_coordinates
        self.y_coordinates = {i: None for i in range(32)}  # estimated_Y_coordinates
        self.z_coordinates = {i: None for i in range(32)}  # estimated_Z_coordinates
        self.latitudes = {i: None for i in range(32)}  # estimated_latitudes
        self.longitudes = {i: None for i in range(32)}  # estimated_longitudes
        self.altitudes = {i: None for i in range(32)}  # estimated_altitudes
        self.france_time = 0            # GPS time in France timezone
        self.gps_time = time            # GPS time
        self.iTOW = 0                   # GPS time of the week



    def connect(self):
        """
        Open serial connection.
        """

        try:
            self._serial_object = Serial(self._port,
                                         self._baudrate,
                                         timeout=self._timeout)
            self._ubxreader = UBXReader(BufferedReader(self._serial_object), False)
            self._connected = True
        except (SerialException, SerialTimeoutException) as err:
            print(f"Error connecting to serial port {err}")

    def disconnect(self):
        """
        Close serial connection.
        """

        if self._connected and self._serial_object:
            try:
                self._serial_object.close()
            except (SerialException, SerialTimeoutException) as err:
                print(f"Error disconnecting from serial port {err}")
        self._connected = False

    def start_read_thread(self):
        """
        Start the serial reader thread.
        """

        if self._connected:
            self._reading = True
            self._serial_thread = Thread(target=self._read_thread, daemon=False)
            self._serial_thread.start()

    def stop_read_thread(self):
        """
        Stop the serial reader thread.
        """

        if self._serial_thread is not None:
            self._reading = False
            self._serial_thread.join()

    def send(self, data1, data2=None, data3=None, data4=None, data5=None, data6=None, data7=None):
        """
        Send data to serial connection.
        The number of data (i.e. UBX messages) in parameters here is arbitrary.
        There is no theoretical limit, it's up to the user to define how many messages will be sent.
        """

        for data in (data1, data2, data3, data4, data5, data6, data7):
            if data is not None:
                self._serial_object.write(data)
                sleep(1)  # Arbitrary timeout to account for delay in getting a response to requests.
                          # Could theoretically be lower

    def _read_thread(self):
        """
        THREADED PROCESS
        Reads and parses UBX message data from the stream.
        In addition to parsing messages, this method will do further processing for specific messages: UBX-NAV-CLOCK,
        UBX-RXM-RAW, UBX-AID-ALM, and UBX-AID-EPH. Theoretically, this part could be done elsewhere.
        """

        while self._reading and self._serial_object:
            if self._serial_object.in_waiting:
                try:
                    (raw_data, parsed_data) = self._ubxreader.read()
                    if parsed_data is not None:
                        print(parsed_data)
                    if parsed_data:
                        """
                        'parsed_data' is an object which takes all the fields of the UBX message 
                        (as defined in the U-blox documentation) as attributes.
                        """
                        # Store the parsed klobuchar information
                        if parsed_data.identity == "AID-HUI":
                            for i in range (0, 4):
                                klob_A = "klobA" + str(i)
                                klob_B = "klobB" + str(i)
                                self.klobuchar_alpha[i + 1] = getattr(parsed_data, klob_A)
                                self.klobuchar_beta[i + 1] = getattr(parsed_data, klob_B)

                        if parsed_data.identity == "NAV-SAT":
                            for i in range(1,parsed_data.numCh+1):
                                prRes_num = "prRes_0" + str(i) if i < 10 else "prRes_" + str(i)
                                sv_num = "svId_0" + str(i) if i < 10 else "svId_" + str(i)
                                self.pseudorange_residual[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, prRes_num)

                        if parsed_data.identity == "NAV-DGPS":
                            for i in range(1, parsed_data.numCh + 1):
                                prc_num = "prc_0" + str(i) if i < 10 else "prc_" + str(i)
                                prrc_num = "prrc_0" + str(i) if i < 10 else "prrc_" + str(i)
                                sv_num = "svid_0" + str(i) if i < 10 else "svid_" + str(i)
                                self.pseudorange_correction[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, prc_num)
                                self.pseudorange_rate_correction[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, prrc_num)

                        if parsed_data.identity == "NAV-SLAS":
                            for i in range(1, parsed_data.cnt + 1):
                                prc_num = "prc_0" + str(i) if i < 10 else "prc_" + str(i)
                                sv_num = "svId_0" + str(i) if i < 10 else "svId_" + str(i)
                                self.pseudorange_correction_from_slas[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, prc_num)

                        if parsed_data.identity == "NAV-SVINFO":
                            for i in range(1, parsed_data.numCh + 1):
                                prRes_num = "prRes_0" + str(i) if i < 10 else "prRes_" + str(i)
                                sv_num = "svid_0" + str(i) if i < 10 else "svid_" + str(i)
                                self.pseudorange_residual_from_svinfo[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, prRes_num)

                        if parsed_data.identity == "NAV-SBAS":
                            for i in range(1, parsed_data.numCh + 1):
                                prc_num = "prc_0" + str(i) if i < 10 else "prc_" + str(i)
                                sv_num = "svid_0" + str(i) if i < 10 else "svid_" + str(i)
                                self.pseudorange_correction_from_sbas[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, prc_num)

                        if parsed_data.identity == "RXM-MEASX":
                            for i in range(1, parsed_data.numSv + 1):
                                pr_rms_err_num = "pseuRangeRMSErr_0" + str(i) if i < 10 else "pseuRangeRMSErr_" + str(i)
                                sv_num = "svid_0" + str(i) if i < 10 else "svid_" + str(i)
                                self.pseudorange_rms_error_index[int(getattr(parsed_data, sv_num)) - 1] = \
                                    getattr(parsed_data, pr_rms_err_num)

                        if parsed_data.identity == "NAV-CLOCK":
                            self.clockBias_dist = parsed_data.clkB * (10 ** -9) * C  # Multiplying to account for units.
                            self.receiver_time = parsed_data.iTOW * (10 ** -3)
                            print("Receiver clock bias: ", self.clockBias_dist)
                            print("GPS System Time: ", self.receiver_time)
                            print("\n")

                        # Store the parsed GPS time information
                        if parsed_data.identity == "NAV-TIMEGPS":
                            self.iTOW = parsed_data.iTOW * (10 ** -3)
                            time_val = itow2utc(parsed_data.iTOW)
                            self.gps_time = time_val

                        if parsed_data.identity == "RXM-RAW":
                            self.receiver_time = parsed_data.iTOW * (10 ** -3)
                            numSV = parsed_data.numSV
                            """
                            The number of attributes for RXM-RAW depends on the amount of satellites detected.
                            As such, there is a repeated block of attributes indexed from 1 to numSV 
                            (= number of satellites detected). 
                            """
                            if numSV != 0:
                                for i in range(1, numSV + 1):
                                    prMes_num = "prMes_0" + str(i) if i < 10 else "prMes_" + str(i)
                                    sv_num = "sv_0" + str(i) if i < 10 else "sv_" + str(i)
                                    snr_num = "cno_0" + str(i) if i < 10 else "cno_" + str(i)
                                    """
                                    The satellite identifier 'sv' is indexed from 1, the corresponding python
                                    dictionary is indexed from 0.
                                    """
                                    self.pseudorange[int(getattr(parsed_data, sv_num)) - 1] = \
                                        getattr(parsed_data, prMes_num)
                                    self.snr[int(getattr(parsed_data, sv_num)) - 1] = getattr(parsed_data, snr_num)
                                print(self.pseudorange)

                        # Store the parsed pseudorange and carrier phase information
                        if parsed_data.identity == "RXM-RAWX":
                            self.receiver_time = parsed_data.rcvTow
                            numSV = parsed_data.numMeas
                            self.measurements_rawx = numSV
                            """
                            The number of attributes for RXM-RAW depends on the amount of satellites detected.
                            As such, there is a repeated block of attributes indexed from 1 to numSV 
                            (= number of satellites detected). 
                            """
                            if numSV != 0:
                                for i in range(1, numSV + 1):
                                    prMes_num = "prMes_0" + str(i) if i < 10 else "prMes_" + str(i)
                                    cpMes_num = "cpMes_0" + str(i) if i < 10 else "cpMes_" + str(i)
                                    locktime_num = "locktime_0" + str(i) if i < 10 else "locktime_" + str(i)
                                    sv_num = "svId_0" + str(i) if i < 10 else "svId_" + str(i)
                                    snr_num = "cno_0" + str(i) if i < 10 else "cno_" + str(i)
                                    pr_sde_num = "prStdev_0"+ str(i) if i < 10 else "prStdev_" + str(i)
                                    cp_sde_num = "cpStdev_0" + str(i) if i < 10 else "cpStdev_" + str(i)
                                    """
                                    The satellite identifier 'svId' is indexed from 1, the corresponding python
                                    dictionary is indexed from 0.
                                    """
                                    self.pseudorange_rawx[i] = \
                                        getattr(parsed_data, prMes_num)
                                    self.carrierphase_rawx[i] = \
                                        getattr(parsed_data, cpMes_num)
                                    self.svid_rawx[i] = \
                                        getattr(parsed_data, sv_num)

                                    self.pseudorange[int(getattr(parsed_data, sv_num)) - 1] = \
                                        getattr(parsed_data, prMes_num)

                                    self.carrierphase_locktime[int(getattr(parsed_data, sv_num)) - 1] = \
                                        getattr(parsed_data, locktime_num)
                                    self.pseudorange_sd_estimate_rawx[int(getattr(parsed_data, sv_num)) - 1] = \
                                        getattr(parsed_data, pr_sde_num)
                                    self.carrierphase_sd_estimate_rawx[int(getattr(parsed_data, sv_num)) - 1] = \
                                        getattr(parsed_data, cp_sde_num)

                        if parsed_data.identity == "AID-ALM":
                            self.almanac_raw[parsed_data.svid - 1].set_data(raw_data)  # Fills up the ephemeris class
                            if not self.almanac_raw[parsed_data.svid - 1].sf_empty:
                                self.almanac_parsed[parsed_data.svid - 1] = \
                                    Almanac_Parsed(self.almanac_raw[parsed_data.svid - 1])
                                self.almanac_parsed[parsed_data.svid - 1].special_print()
                                print("\n")

                        # Store the parsed Ephemeris information
                        if parsed_data.identity == "AID-EPH" and parsed_data.how != 0:
                            print('svid: ', parsed_data.svid)
                            self.ephemeris_raw[parsed_data.svid - 1].set_data(raw_data)  # Fills up the ephemeris class

                            if not self.ephemeris_raw[parsed_data.svid - 1].sf_empty:
                                self.ephemeris_parsed[parsed_data.svid - 1] = \
                                    Ephemeris_Parsed(self.ephemeris_raw[parsed_data.svid - 1])
                                # self.ephemeris_parsed[parsed_data.svid - 1].special_print()

                        # Store the estimated X Y Z coordinates, latitude, longitude, altitude information
                        if parsed_data.identity == "AID-EPH" and parsed_data.how != 0 and \
                                self.ephemeris_parsed[parsed_data.svid - 1] is not None:

                            X, Y, Z = get_wgs84_sat_position(self.ephemeris_parsed[parsed_data.svid - 1],
                                                             self.receiver_time,
                                                             self.pseudorange[parsed_data.svid - 1])

                            self.x_coordinates[parsed_data.svid - 1] = X
                            self.y_coordinates[parsed_data.svid - 1] = Y
                            self.z_coordinates[parsed_data.svid - 1] = Z

                            self.sat_position[parsed_data.svid - 1] = XYZPosition(X, Y, Z)
                            #print("XYZ: ", (X, Y, Z))
                            Lat, Long, Alt = xyz_to_latlongalt(X, Y, Z)

                            self.latitudes[parsed_data.svid - 1] = Lat
                            self.longitudes[parsed_data.svid - 1] = Long
                            self.altitudes[parsed_data.svid - 1] = Alt

                            #print("LatLongAlt: ", (Lat, Long, Alt))
                            #print("\n")

                except (ube.UBXStreamError, ube.UBXMessageError, ube.UBXTypeError,
                        ube.UBXParseError) as err:
                    print(f"Something went wrong {err}")
                    continue
