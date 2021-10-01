"""
This file aggregates all methods needed to compute the position of the satellite or the receiver.
"""
from math import sqrt, atan, sin, cos, pi
from typing import Union, Optional

from config import MU, OMEGA_E_DOT, C, REF_X, REF_Y, REF_Z, F
from ephemeris import Ephemeris_Parsed

import numpy as np


class XYZPosition:
    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z


def get_wgs84_sat_position(eph: Ephemeris_Parsed, time: int, pseudorange: int):
    """
    This method computes the ECEF position of the satellite in the sky using ephemeris and pseudorange data.
    The method used here is described in the official GPS documentation.
    """
    af0 = eph.af0
    af1 = eph.af1
    af2 = eph.af2
    m0 = eph.m0
    delta_n = eph.delta_n
    a = eph.sqrt_a ** 2
    ecc = eph.e
    omega = eph.omega
    omega0 = eph.omega0
    omegadot = eph.omega_dot
    cuc = eph.cuc
    cus = eph.cus
    crc = eph.crc
    crs = eph.crs
    cic = eph.cic
    cis = eph.cis
    i0 = eph.i0
    idot = eph.idot
    toe = eph.toe
    toc = eph.toc

    """Correcting GPS time for transit time (range/speed of light) if range value was indeed received"""
    t_sv = time - pseudorange / C if pseudorange is not None else time

    # Initializing this value at 0 because Ek hasn't been computed yet
    delta_tr = 0
    # Code phase offset
    delta_t_sv = af0 + af1 * (t_sv - toc) + af2 * (t_sv - toc) ** 2 + delta_tr
    t = t_sv - delta_t_sv
    Tk = t - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    n0 = sqrt(MU / (a ** 3))

    N = n0 + delta_n

    Mk = m0 + N * Tk

    Ek = kepler_solve(Mk, ecc)

    # Relativistic correction term
    delta_tr = F * ecc * sqrt(a) * sin(Ek)

    # Computing Tk again with the real value of delta_tr
    delta_t_sv = af0 + af1 * (t_sv - toc) + af2 * ((t_sv - toc) ** 2) + delta_tr
    t = t_sv - delta_t_sv
    Tk = t - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    sinVk = (sin(Ek) * sqrt(1 - ecc ** 2)) / (1 - ecc * cos(Ek))

    cosVk = (cos(Ek) - ecc) / (1 - ecc * cos(Ek))

    phik = arctan2(sinVk, cosVk) + omega

    delta_uk = cuc * cos(2 * phik) + cus * sin(2 * phik)
    delta_rk = crc * cos(2 * phik) + crs * sin(2 * phik)
    delta_ik = cic * cos(2 * phik) + cis * sin(2 * phik)

    Uk = phik + delta_uk
    Rk = a * (1 - ecc * cos(Ek)) + delta_rk
    Ik = i0 + idot * Tk + delta_ik

    OmegaK = omega0 + (omegadot - OMEGA_E_DOT) * Tk - OMEGA_E_DOT * toe

    Xk = Rk * cos(Uk)
    Yk = Rk * sin(Uk)

    X = Xk * cos(OmegaK) - Yk * cos(Ik) * sin(OmegaK)
    Y = Xk * sin(OmegaK) + Yk * cos(Ik) * cos(OmegaK)
    Z = Yk * sin(Ik)

    return X, Y, Z


def kepler_solve(Mk, ecc):
    Ek = Mk
    while abs(Mk - Ek + ecc * sin(Ek)) > 10 ** -12:
        Ek = Ek - (Ek - ecc * sin(Ek) - Mk) / (1 - ecc * cos(Ek))
    return Ek


def arctan2(sinus, cosinus):
    if cosinus > 0:
        return atan(sinus / cosinus)
    elif sinus > 0:
        return pi - atan(sinus / (-1 * cosinus))
    elif sinus < 0:
        return (-1 * pi) - atan(sinus / (-1 * cosinus))


def get_delta_tr(ephemeris: Ephemeris_Parsed, pseudorange, receiver_time):
    af0 = ephemeris.af0
    af1 = ephemeris.af1
    af2 = ephemeris.af2
    m0 = ephemeris.m0
    delta_n = ephemeris.delta_n
    a = ephemeris.sqrt_a ** 2
    ecc = ephemeris.e
    toe = ephemeris.toe
    toc = ephemeris.toc

    t_sv = receiver_time - pseudorange / C

    # Initializing this value at 0 because Ek hasn't been calculated yet
    delta_tr = 0
    # Code phase offset
    delta_t_sv = af0 + af1 * (t_sv - toc) + af2 * (t_sv - toc) ** 2 + delta_tr
    t = t_sv - delta_t_sv
    Tk = t - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    n0 = sqrt(MU / (a ** 3))

    N = n0 + delta_n

    Mk = m0 + N * Tk

    Ek = kepler_solve(Mk, ecc)

    # Relativistic correction term
    delta_tr = F * ecc * sqrt(a) * sin(Ek)
    return delta_tr


def xyz_to_latlongalt(X, Y, Z):
    """
    This method transforms a position in the ECEF coordinate system to Latitude, Longitude and Altitude.
    """
    a = 6378137
    b = 6356752.3142
    ec = 0.00669437999014
    eprimec = 0.00673949674228

    # Longitude
    lamb_da = arctan2(Y, X)

    # Latitude
    p = sqrt(X ** 2 + Y ** 2)
    tanu = (Z / p) * (a / b)
    end_loop = False
    while not end_loop:
        cosu = (1 / (1 + tanu ** 2)) ** 0.5
        sinu = (1 - cosu ** 2) ** 0.5
        tanPhi = (Z + eprimec * b * sinu ** 3) / (p - ec * a * cosu ** 3)

        if abs(tanu - (b / a) * tanPhi) < 0.0000000001:
            end_loop = True
        else:
            tanu = (b / a) * tanPhi

    phi = atan(a / b * tanu)

    # Altitude
    N = a / sqrt(1 - ec * sin(phi) ** 2)
    h = p / cos(phi) - N

    Lat = 180 / pi * phi
    Long = 180 / pi * lamb_da
    Alt = h
    return Lat, Long, Alt


# def get_receiver_position(eph, pseudorange, satPosition, snr, clockBias_dist, receiver_time) -> \
#         Union[tuple[None, None, None], tuple[Optional[float], Optional[float], Optional[float]]]:
#     """
#     This method computes the position of the receiver using ephemeris and pseudorange data with the least-squares
#     estimation method.
#     """
#     def update_h_matrix(Hrow, sat_position: XYZPosition, receiver_position):
#         distance = sqrt((sat_position.X - receiver_position[0]) ** 2 +
#                         (sat_position.Y - receiver_position[1]) ** 2 +
#                         (sat_position.Z - receiver_position[2]) ** 2)
#         Hrow[0] = (sat_position.X - receiver_position[0]) / distance
#         Hrow[1] = (sat_position.Y - receiver_position[1]) / distance
#         Hrow[2] = (sat_position.Z - receiver_position[2]) / distance
#
#     Lat, Long, Alt = None, None, None
#
#     """
#     Taking the 5 satellites with the biggest (=best) SNR, from list of available satellites.
#     This method to filter satellites might not be optimal.
#     There could be better ways to select the satellites that will be considered.
#     """
#     pr_sv_available = list(k for k, v in pseudorange.items() if v is not None)
#     eph_sv_available = list(k for k, v in eph.items() if v is not None)
#     sv_list = list(set(pr_sv_available).intersection(eph_sv_available))
#     snr_list = [snr[x] for x in sv_list]
#     sv_list = [sv_list[x] for x in np.argpartition(snr_list, -5)[-5:].tolist()]
#     print(sv_list)
#
#     if len(sv_list) < 4:
#         print("Not enough satellite with strong signal")
#         return None, None, None
#
#     pr_measured = [pseudorange[x] for x in sv_list]
#
#     pr_measured_corrected = \
#         [x + C * (eph[y].af0 + eph[y].af1 * (receiver_time - x / C - eph[y].toc) +
#                   eph[y].af2 * ((receiver_time - x / C - eph[y].toc) ** 2) +
#                   get_delta_tr(eph[y], x, receiver_time) - eph[y].tgd)
#          for x, y in zip(pr_measured, sv_list)]
#
#     """Initial position taken from reference point"""
#     last_receiver_position = np.array([REF_X, REF_Y, REF_Z, clockBias_dist])
#
#     H = -np.ones((len(sv_list), 4))
#     delta_x = np.array([42])  # Arbitrary number to enter the loop
#     loops = 10
#     while np.linalg.norm(delta_x[:3]) > 10 ** -6 and loops > 0:
#         for i in range(len(sv_list)):
#             update_h_matrix(H[i], satPosition[sv_list[i]], last_receiver_position)
#
#         pr_calculated = [sqrt((satPosition[sv_id].X - last_receiver_position[0]) ** 2
#                               + (satPosition[sv_id].Y - last_receiver_position[1]) ** 2
#                               + (satPosition[sv_id].Z - last_receiver_position[2]) ** 2)
#                          + clockBias_dist for sv_id in sv_list]
#         delta_rho = np.array([x - y for x, y in zip(pr_calculated, pr_measured_corrected)])
#
#         try:
#             delta_x = np.dot(np.dot(np.linalg.inv(np.dot(H.transpose(), H)), H.transpose()), delta_rho)
#
#             residuals = delta_rho - np.dot(H, delta_x)
#             print("Residuals: ", residuals)
#         except np.linalg.LinAlgError:
#             print("H matrix has no inverse!")
#             return None, None, None
#
#         last_receiver_position = last_receiver_position + delta_x
#         Lat, Long, Alt = \
#             xyz_to_latlongalt(last_receiver_position[0], last_receiver_position[1], last_receiver_position[2])
#         loops -= 1
#         print("Latitude: ", Lat)
#         print("Longitude: ", Long)
#         print("Altitude: ", Alt)
#         print("\n")
#     return Lat, Long, Alt
