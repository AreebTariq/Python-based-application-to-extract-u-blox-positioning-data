###############################################################################################
"""
The getubxdata, as the name suggests, is a tool to acquire, display, and save the required data from u-blox receiver.
The main source used is pyubx2 open source library.

In the current implementation, there are four messages used to acquire information about pseudoranges, carrier phase,
klobuchar, and GPS time. However, in the future this can be extended depending upon on what information is sought.

It starts the UBX Streamer app, which initiates the reading thread. Following on, u-blox required messages are polled,
the messages are parsed and serially output. After all the messages are polled, thread is stopped and receiver is
disconnected.

Finally, all the desired information is formatted and stored in the autogenerated excel file
which is saved in the same directory as of this file.

Created by: Areeb Tariq
Project: Internship at Institut Polytechnique de Paris (IP Paris)
Date: August 2021
"""
###############################################################################################
# Imports needed for this project
from pyubx2 import UBXMessage, POLL, GET
from config import PORT, TIMEOUT, BAUDRATE
from ubxstreamer import UBXStreamer
from threading import Lock
import xlwt
from xlwt import Workbook

###################################################################################
# Global variable to control the number of recordings (number of polling) for each message
# Just change the variable value here and everything will be managed accordingly automatically
number_of_recordings = 20

# Excel Workbook is created
wb = Workbook()

###################################################################################
# This section is for settings of background color and font alignment of excel worksheets' cell.#
style = xlwt.XFStyle()                  # Style for column titles
style_2 = xlwt.XFStyle()                # Style for rest of the data
style_3 = xlwt.XFStyle()                # Style for long column titles as used for ephemeris data
style_4 = xlwt.XFStyle()                # Style for GPS time iTOW column title

bg = xlwt.Pattern()
bg.pattern = bg.SOLID_PATTERN           # NO_PATTERN, SOLID_PATTERN, or 0x00 through 0x12
bg.pattern_fore_colour = 5              # Here '5' indicates the yellow color. Can be modified.

bg_2 = xlwt.Pattern()
bg_2.pattern = bg.SOLID_PATTERN           # NO_PATTERN, SOLID_PATTERN, or 0x00 through 0x12
bg_2.pattern_fore_colour = 3              # Here '3' indicates the yellow color. Can be modified.

style.pattern = bg
style_3.pattern = bg
style_4.pattern = bg_2

aligment = xlwt.Alignment()             # Alignment for cells
aligment_2 = xlwt.Alignment()           # Alignment for cells with large text as in the ephemeris data titles

aligment.horz = aligment.HORZ_CENTER    # Horizontal alignment
aligment_2.horz = aligment.HORZ_CENTER  # Horizontal alignment
aligment_2.vert = aligment.VERT_CENTER  # Vertical alignment

style.alignment = aligment
style_2.alignment = aligment
style_3.alignment = aligment_2
style_4.alignment = aligment
style_3.alignment.wrap = 1              # Wrap the long text in a cell
#######################################################################################
# Here are the required sheets created.

# 1st sheet for Pseudorange and Carrier phase data, title : "psr_cp_data"
sheet_1 = wb.add_sheet('psr_cp_data')

# 2nd sheet for Klobuchar data, title : "Klobuchar_data"
sheet_2 = wb.add_sheet('klobuchar_data')

# 3rd sheet for Ephemeris data, title : "ephemeris_data"
sheet_3 = wb.add_sheet('ephemeris_data')

# 4th sheet for GPS time information, title : "time_information"
sheet_4 = wb.add_sheet('time_information')
#######################################################################################

def initialize_worksheets():
    """initialize_worksheets.

    This function initializes all the sheets in the workbook.
    It adds columns titles, their background color and size.

    """

    # Add 1st sheet column titles and widths
    for entry_counter in range(0,number_of_recordings*4, 4):
        sheet_1.write(0, entry_counter, 'GPS Time - iTOW (s)', style_4)
        sheet_1.write(0, entry_counter+1, 'SV ID', style)
        sheet_1.write(0, entry_counter+2, 'Pseudorange measurement', style)
        sheet_1.write(0, entry_counter+3, 'Carrierphase measurement', style)

        sheet_1.col(entry_counter).width = 6520
        sheet_1.col(entry_counter+1).width = 3120
        sheet_1.col(entry_counter+2).width = 6520
        sheet_1.col(entry_counter+3).width = 6520

    # Add 2nd sheet column titles and widths
    sheet_2.write(0, 0, 'GPS Time - iTOW (s)', style_4)
    sheet_2.write(0, 1, 'Klobuchar name', style)
    sheet_2.write(0, 2, 'Value', style)

    sheet_2.col(entry_counter).width = 6520
    sheet_2.col(entry_counter + 1).width = 5120
    sheet_2.col(entry_counter + 2).width = 6520

    # Add 3rd sheet column titles and widths
    sheet_3.write(0, 0, 'SV ID (svid)', style_3)
    sheet_3.write(0, 1, 'Mean Anomaly at reference time (m0)', style_3)
    sheet_3.write(0, 2, 'Mean Motion Difference From Computed Value (delta_n)', style_3)
    sheet_3.write(0, 3, 'Eccentricity (e)', style_3)
    sheet_3.write(0, 4, 'Square Root of the Semi-Major Axis (sqrt_a)', style_3)
    sheet_3.write(0, 5, 'Longitude of Ascending Node of Orbit Plane at Weekly Epoch (omega0)', style_3)
    sheet_3.write(0, 6, 'Inclination Angle at Reference Time (i0)', style_3)
    sheet_3.write(0, 7, 'Argument of Perigee (omega)', style_3)
    sheet_3.write(0, 8, 'Rate of Right Ascension (omega_dot)', style_3)
    sheet_3.write(0, 9, 'Rate of Inclination Angle (idot)', style_3)
    sheet_3.write(0, 10, 'Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (cuc)', style_3)
    sheet_3.write(0, 11, 'Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (cus)', style_3)
    sheet_3.write(0, 12, 'Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius (crc)', style_3)
    sheet_3.write(0, 13, 'Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (crs)', style_3)
    sheet_3.write(0, 14, 'Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (cic)', style_3)
    sheet_3.write(0, 15, 'Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (cis)', style_3)
    sheet_3.write(0, 16, 'Reference Time Ephemeris (toe)', style_3)
    sheet_3.write(0, 17, 'Issue of Data - Ephemeris (iode)', style_3)
    sheet_3.write(0, 18, 'Issue of Data - Clock (iodc)', style_3)
    sheet_3.write(0, 19, 'Clock Data reference time (toc)', style_3)
    sheet_3.write(0, 20, 'Polynomial coefficient af0', style_3)
    sheet_3.write(0, 21, 'Polynomial coefficient af1', style_3)
    sheet_3.write(0, 22, 'Polynomial coefficient af2', style_3)
    sheet_3.write(0, 23, 'Group delay differential (tgd)', style_3)
    sheet_3.write(0, 24, 'Estimated X coordinate', style_3)
    sheet_3.write(0, 25, 'Estimated Y coordinate', style_3)
    sheet_3.write(0, 26, 'Estimated Z coordinate', style_3)
    sheet_3.write(0, 27, 'Estimated Latitude', style_3)
    sheet_3.write(0, 28, 'Estimated Longitude', style_3)
    sheet_3.write(0, 29, 'Estimated Altitude', style_3)

    for i in range(0, 30):
        sheet_3.col(i).width = 6520

    # Add 4th sheet column titles and widths
    sheet_4.write(0, 0, 'GPS current UTC time', style)

    sheet_4.col(0).width = 8120
    sheet_4.col(1).width = 6520

    return


def write_recorded_data(psr_cp_index, klob_index, to_write_psr_cp_info):
    """write_recorded_data.

    The function write the recorded data from ubx receiver to the excel file.

    :param int psr_cp_index: variable to manage the column index of pseudorange and carrier phase sheet.
    :param int klob_index: variable to manage the column index of Klobuchar information sheet.
    :param bool to_write_ephemeris_and_time: indicates whether to write ephemeris and time information to sheet.
                                              (False (don't write - default), True (write)).

    """
    if to_write_psr_cp_info == True:
        # Loop to write all pseudorange and carrier phase measurements against respective sv id in excel sheet
        for i in range(1, ubxs.measurements_rawx + 1):
            sheet_1.write(i, psr_cp_index, ubxs.receiver_time, style_2)
            sheet_1.write(i, psr_cp_index + 1, ubxs.svid_rawx[i], style_2)
            sheet_1.write(i, psr_cp_index + 2, ubxs.pseudorange_rawx[i], style_2)
            sheet_1.write(i, psr_cp_index + 3, ubxs.carrierphase_rawx[i], style_2)
    else:
        # Loop to write all Klobuchar measurements in excel sheet
        for i in range(1, 5):
            klobuchar_alpha_name = "klobA" + str(i - 1)
            klobuchar_beta_name = "klobB" + str(i - 1)

            sheet_2.write(i, klob_index, ubxs.iTOW, style_2)
            sheet_2.write(i, klob_index + 1, klobuchar_alpha_name, style_2)
            sheet_2.write(i, klob_index + 2, ubxs.klobuchar_alpha[i], style_2)

            sheet_2.write(i + 4, klob_index, ubxs.iTOW, style_2)
            sheet_2.write(i + 4, klob_index + 1, klobuchar_beta_name, style_2)
            sheet_2.write(i + 4, klob_index + 2, ubxs.klobuchar_beta[i], style_2)

            # Local variable for the count of ephemeris information
            ephemeris_counter = 1

        # Loop to write all Ephemeris measurements in excel sheet
        for i in range(1, 32):
            if ubxs.ephemeris_parsed[i] is not None:
                sheet_3.write(ephemeris_counter, 0, ubxs.ephemeris_parsed[i].svid, style_2)
                sheet_3.write(ephemeris_counter, 1, ubxs.ephemeris_parsed[i].m0, style_2)
                sheet_3.write(ephemeris_counter, 2, ubxs.ephemeris_parsed[i].delta_n, style_2)
                sheet_3.write(ephemeris_counter, 3, ubxs.ephemeris_parsed[i].e, style_2)
                sheet_3.write(ephemeris_counter, 4, ubxs.ephemeris_parsed[i].sqrt_a, style_2)
                sheet_3.write(ephemeris_counter, 5, ubxs.ephemeris_parsed[i].omega0, style_2)
                sheet_3.write(ephemeris_counter, 6, ubxs.ephemeris_parsed[i].i0, style_2)
                sheet_3.write(ephemeris_counter, 7, ubxs.ephemeris_parsed[i].omega, style_2)
                sheet_3.write(ephemeris_counter, 8, ubxs.ephemeris_parsed[i].omega_dot, style_2)
                sheet_3.write(ephemeris_counter, 9, ubxs.ephemeris_parsed[i].idot, style_2)
                sheet_3.write(ephemeris_counter, 10, ubxs.ephemeris_parsed[i].cuc, style_2)
                sheet_3.write(ephemeris_counter, 11, ubxs.ephemeris_parsed[i].cus, style_2)
                sheet_3.write(ephemeris_counter, 12, ubxs.ephemeris_parsed[i].crc, style_2)
                sheet_3.write(ephemeris_counter, 13, ubxs.ephemeris_parsed[i].crs, style_2)
                sheet_3.write(ephemeris_counter, 14, ubxs.ephemeris_parsed[i].cic, style_2)
                sheet_3.write(ephemeris_counter, 15, ubxs.ephemeris_parsed[i].cis, style_2)
                sheet_3.write(ephemeris_counter, 16, ubxs.ephemeris_parsed[i].toe, style_2)
                sheet_3.write(ephemeris_counter, 17, ubxs.ephemeris_parsed[i].iode, style_2)
                sheet_3.write(ephemeris_counter, 18, ubxs.ephemeris_parsed[i].iodc, style_2)
                sheet_3.write(ephemeris_counter, 19, ubxs.ephemeris_parsed[i].toc, style_2)
                sheet_3.write(ephemeris_counter, 20, ubxs.ephemeris_parsed[i].af0, style_2)
                sheet_3.write(ephemeris_counter, 21, ubxs.ephemeris_parsed[i].af1, style_2)
                sheet_3.write(ephemeris_counter, 22, ubxs.ephemeris_parsed[i].af2, style_2)
                sheet_3.write(ephemeris_counter, 23, ubxs.ephemeris_parsed[i].tgd, style_2)
                sheet_3.write(ephemeris_counter, 24, ubxs.x_coordinates[i], style_2)
                sheet_3.write(ephemeris_counter, 25, ubxs.y_coordinates[i], style_2)
                sheet_3.write(ephemeris_counter, 26, ubxs.z_coordinates[i], style_2)
                sheet_3.write(ephemeris_counter, 27, ubxs.latitudes[i], style_2)
                sheet_3.write(ephemeris_counter, 28, ubxs.longitudes[i], style_2)
                sheet_3.write(ephemeris_counter, 29, ubxs.altitudes[i], style_2)

                ephemeris_counter = ephemeris_counter + 1

        # Convert data into time format to write in excel sheet
        xf = xlwt.easyxf(num_format_str='HH:MM:SS')

        # Write GPS time to excel file
        sheet_4.write(0, 1, ubxs.gps_time, xf)

    return


# Creating mutex to avoid overrunning of multiple u-blox messages
mutex = Lock()

######################################################################
# Here's the main function body
if __name__ == "__main__":

    msg1 = None
    msg2 = None
    msg3 = None
    msg4 = None

    # Start UBXStreamter which starts the read thread to intake the information #
    print("Instantiating UBXStreamer class...")

    # The settings of port, baudrate, and timeout are defined in the file config.py (present in the same folder).
    # These can be modified there.
    ubxs = UBXStreamer(PORT, BAUDRATE, TIMEOUT)

    print(f"Connecting to serial port {PORT} at {BAUDRATE} baud...")
    ubxs.connect()

    # Initialize the excel worksheets
    initialize_worksheets()

    print("Starting reader thread...")
    ubxs.start_read_thread()

    print("Polling receiver...\n\n")

    # Local variable to indicate whether to write the ephemeris and time information to the excel file
    # First we are reading all info (klobuchar, ephemeris, GPS time) other than pseudorange and carrier phase
    to_write_psr_cp_info = False

    mutex.acquire()

    if msg1 is None:
        msg1 = UBXMessage('AID', 'AID-HUI', POLL)               # Gives Klobuchar information
    if msg2 is None:
        msg2 = UBXMessage('AID', 'AID-EPH', POLL)               # Gives Ephemeris information
    if msg3 is None:
        msg3 = UBXMessage('NAV', 'NAV-TIMEGPS', POLL)           # Gives GPS time information

    # Print the serialize input on the console/terminal. Helpful for debugging and quick look.
    # The same results with better formatting are stored in the excel file as well.
    if msg1 is not None and msg2.identity is not None and msg3.identity is not None:
        ubxs.send(msg1.serialize(), msg2.serialize(), msg3.serialize())

        # Messages have been obtained, now release the mutex.
        mutex.release()

        # Write the recorded information from all the messages in this iteration to the excel file
        write_recorded_data(1, 0, to_write_psr_cp_info)

        # Update the flag for next operation: reading of pseudorange and carrier phase
        to_write_psr_cp_info = True

        # Reset the local msg variable for the next iteration
        msg1 = None
        msg2 = None
        msg3 = None
        msg4 = None

    for loop_counter in range(0, number_of_recordings):

        # Start Mutex Lock
        mutex.acquire()

        if msg1 is None:
            msg1 = UBXMessage('RXM', 'RXM-RAWX', POLL)  # Gives Pseudorange, Carrier Phase information, and receiver time

        # Print the serialize input on the console/terminal. Helpful for debugging and quick look.
        # The same results with better formatting are stored in the excel file as well.
        if msg1 is not None:
            ubxs.send(msg1.serialize())

            # Messages have been obtained, now release the mutex.
            mutex.release()

            # Write the recorded information from all the messages in this iteration to the excel file
            write_recorded_data(loop_counter * 4, 0, to_write_psr_cp_info)

            # Reset the local msg variable for the next iteration
            msg1 = None
            msg2 = None
            msg3 = None
            msg4 = None

    print("\n\nStopping reader thread...")

    # Stop the reading thread
    ubxs.stop_read_thread()

    print("Disconnecting from serial port...")
    ubxs.disconnect()

    # Finally save the excel file with multiple respective sheets in the same directory as of this file
    wb.save('ubx data.xls')