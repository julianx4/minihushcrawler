import machine
import utime

DEFAULT_BAUDRATE = 1000000
LATENCY_TIMER = 10  # ms, adjust based on your specific requirements

class PortHandler:
    def __init__(self, tx_pin, rx_pin, baudrate=DEFAULT_BAUDRATE):
        self.baudrate = baudrate
        self.port_name = "UART0"
        self.is_open = False
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0
        self.is_using = False

        self.uart = machine.UART(0, baudrate=self.baudrate)
        self.uart.init(baudrate=self.baudrate, bits=8, parity=None, stop=1, tx=machine.Pin(tx_pin), rx=machine.Pin(rx_pin))
        self.is_open = True
        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

    def closePort(self):
        self.uart.deinit()
        self.is_open = False

    def clearPort(self):
        self.uart.init(baudrate=self.baudrate, bits=8, parity=None, stop=1)

    def readPort(self, length):
        data = self.uart.read(length)
        if data is None:  # Check if data is None
            return []  # Return an empty list if no data is available
        else:
            return list(data)  # Convert to list and return if data is available

    def writePort(self, packet):
        return self.uart.write(bytes(packet))

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (LATENCY_TIMER / 1000.0)

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec / 1000.0

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True
        return False

    def getCurrentTime(self):
        return utime.ticks_ms() / 1000.0

    def getTimeSinceStart(self):
        return self.getCurrentTime() - self.packet_start_time

    def getBytesAvailable(self):
        return self.uart.any()
