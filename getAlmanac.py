from pyubx2 import UBXMessage, POLL

from config import PORT, TIMEOUT, BAUDRATE
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
    msg2 = UBXMessage('AID', 'AID-ALM', POLL)
    ubxs.send(msg1.serialize(), msg2.serialize())

    print("\n\nStopping reader thread...")
    ubxs.stop_read_thread()
    print("Disconnecting from serial port...")
    ubxs.disconnect()
    print("Done")
