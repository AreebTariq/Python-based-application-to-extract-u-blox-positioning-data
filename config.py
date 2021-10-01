"""
This configuration file defines a set of variables and methods to be used throughout the application.
"""
MU = 3.986005 * (10 ** 14)
OMEGA_E_DOT = 7.292115 * (10 ** -5)
C = 299792458
F = -4.442807633 * (10**-10)
# Reference point (in meters) = Eiffel Tower location in ECEF
REF_X = 4201008
REF_Y = 168327
REF_Z = 4780211

# set PORT, BAUDRATE and TIMEOUT as appropriate for your system
PORT = 'COM9'
BAUDRATE = 9600
TIMEOUT = 0


def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
