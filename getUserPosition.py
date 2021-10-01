from pyubx2 import UBXMessage, POLL

from config import PORT, TIMEOUT, BAUDRATE
from position import get_receiver_position
from ubxstreamer import UBXStreamer

if __name__ == "__main__":

    print("Instantiating UBXStreamer class...")
    ubxs = UBXStreamer(PORT, BAUDRATE, TIMEOUT)
    print(f"Connecting to serial port {PORT} at {BAUDRATE} baud...")
    ubxs.connect()
    print("Starting reader thread...")
    ubxs.start_read_thread()

    print("\nPolling receiver...\n\n")
    msg1 = UBXMessage('NAV', 'NAV-CLOCK', POLL)
    msg2 = UBXMessage('RXM', 'RXM-RAW', POLL)
    msg3 = UBXMessage('AID', 'AID-EPH', POLL)
    ubxs.send(msg1.serialize(), msg2.serialize(), msg3.serialize())

    Lat, Long, Alt = \
        get_receiver_position(ubxs.ephemeris_parsed, ubxs.pseudorange,
                              ubxs.sat_position, ubxs.snr, ubxs.clockBias_dist, ubxs.receiver_time)

    print("\n\nStopping reader thread...")
    ubxs.stop_read_thread()
    print("Disconnecting from serial port...")
    ubxs.disconnect()
    print("Done")
