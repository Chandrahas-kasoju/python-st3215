import serial
import threading
import time

from st3215.values import *

#from .values import *

from STservo_sdk import *

class ST3215:

    def __init__(self, device, baudrate = DEFAULT_BAUDRATE):

        if baudrate not in BAUDRATES:
            raise ValueError(f"{baudrate} must be in: {BAUDRATES}")

        self.ser = serial.Serial(
            port=device,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            timeout=0)
    

        self.ser.reset_input_buffer()
        self.tx_time_per_byte = (1000.0 / baudrate) * 10.0

        self.lock = threading.Lock()








    # Timeout definition when sending a message

    # Concurrency

    # Set Servo ID

    # Init bus 

    # Ping

    # Read Byte

    def tx(self, packet):

        checksum = 0
        packet_length = packet[PKT_LENGTH] + 4

        if packet_length > TXPACKET_MAX_LEN:
            raise ValueError(f"TX Packet len > {TXPACKET_MAX_LEN}")


        txpacket[PKT_HEADER_0] = 0xFF
        txpacket[PKT_HEADER_1] = 0xFF

        for idx in range(2, packet_length - 1):
            checksum += packet[idx]

        packet[packet_length - 1] = ~checksum & 0xFF

        with self.lock:
            self.ser.flush()
            written_bytes = self.ser.write(packet)
            if written_bytes != packet_length:
                raise ValueError("Could not transmit packet")

        return True

    def rx(self):
        rxpacket = []

        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 6  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        while True:
            rxpacket.extend(self.portHandler.readPort(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (rxpacket[PKT_ID] > 0xFD) or (rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN) or (
                            rxpacket[PKT_ERROR] > 0x7F):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != (rxpacket[PKT_LENGTH] + PKT_LENGTH + 1):
                        wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
                        continue

                    if rx_length < wait_length:
                        # check timeout
                        if self.portHandler.isPacketTimeout():
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0: idx]
                    rx_length -= idx

            else:
                # check timeout
                if self.portHandler.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

        self.portHandler.is_using = False
        return rxpacket, result



    def rx(self):

        packet = []
        checksum = 0
        packet_length = 0

        with self.lock:

            now = round(time.time() * 1000000000) / 1000000.0

            self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + LATENCY_TIMER

            # First read the initial packet (length: 6): HEADER0, HEADER1, ID, LENGTH, ERROR, CHECKSUM
            data = self.ser.read(6)
            if len(data) < 6:
                raise ValueError("Invalid initial packet")




        # Then, now we can read the lenght value

        # self.ser.read(length)

    def close(self):
        self.ser.close()


#### SETUP CODE ####
#from STservo_sdk import *
#
#
#
## Default setting
#BAUDRATE                = 1000000
#DEVICENAME              = '/dev/ttyACM0'
#
#MAX_SPEED = 3000
#MAX_ACCEL = 50
#MIN_POSITION  = 0
#MAX_POSITION  = 4095
#
#
#portHandler = PortHandler(DEVICENAME)
#
#packetHandler = sts(portHandler)
#
#if not portHandler.openPort():
#  raise ValueError(f"Fail to open Servo board port: {DEVICENAME}")
#
#
#if not portHandler.setBaudRate(BAUDRATE):
#  raise ValueError("Failed to set the baudrate")
#
#
#sts_model_number, sts_comm_result, sts_error = packetHandler.ping(1)
#if sts_comm_result != COMM_SUCCESS or sts_error != 0:
#  raise ValueError("Could not find servo: %d" % 1)
#else:
#  print(sts_model_number)
#
#value, comm_result, erro = packetHandler.read1ByteTxRx(1, 0x5)
#print(value)
#
#result, error = packetHandler.write1ByteTxRx(1, 0x37, 0)
#result, error = packetHandler.write1ByteTxRx(1, 0x5, 12)
#print(result)
#
#value, comm_result, erro = packetHandler.read1ByteTxRx(12, 0x5)
#print(value)

