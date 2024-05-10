import threading
import time
import serial
import serial.tools.list_ports
import packet as packet
import util as util
import fnmatch
import glob

class Uart():
    uart = None
    uart_connected = False
    def __init__(self, transmit_queue, receive_queue) -> None:
        self.transmit_queue = transmit_queue
        self.receive_queue = receive_queue
        available_ports = []

        while(self.uart_connected == False):
            available_ports = self.auto_detect_serial_unix()
            print(available_ports)
            if len(available_ports) == 0:
                print("No Comports available")
                # time.sleep(0.5)
                continue
            else:
                print("uart connected")
                try:
                    self.uart = serial.Serial(available_ports[0], 115200,timeout=1)
                except:
                    print("access denied")
                break
        self.uart_connected = True


    def auto_detect_serial_unix(self, preferred_list=['*']):
    # '''try to auto-detect serial ports on win32'''
        glist = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        ret = []

        # try preferred ones first
        for d in glist:
            for preferred in preferred_list:
                if fnmatch.fnmatch(d, preferred):
                    ret.append(d)
        if len(ret) > 0:
            return ret
        # now the rest
        for d in glist:
            ret.append(d)
        return ret
    
    def handle_event(self, command, data):
        if command == packet.MODE_DEFAULT:
            left_reading = (data[0] << 8) | data[1]
            right_reading = (data[2] << 8) | data[3]
            self.transmit_queue.put([command, left_reading, right_reading])
    
    def rx_thread(self):
        while self.uart_connected == False:
            time.sleep(0.5)
        
        command = 0
        length = 0
        data = []
        expected = packet.START_BYTE

        while True:
            try:
                receivedByte = int.from_bytes(self.uart.read(1), byteorder='big')
            except:
                self.uart_connected = False
                break

            if (expected == packet.START_BYTE and receivedByte == packet.START_BYTE):
                expected = packet.COMMAND_BYTE
                continue

            if (expected == packet.COMMAND_BYTE):
                command = receivedByte
                expected = packet.LENGTH_BYTE
                continue
                
            if (expected == packet.LENGTH_BYTE):
                length = receivedByte
                expected = packet.DATA_BYTE

                if (length == 0):
                    #  All data received
                    self.handle_event(command, data)
                    command = 0
                    length = 0
                    expected = packet.START_BYTE
                    data = []
                continue

            if expected == packet.DATA_BYTE:
                data.append(receivedByte)

                if (len(data) == length):
                    # all bytes received
                    self.handle_event(command, data)
                    command = 0
                    length = 0
                    expected = packet.START_BYTE
                    data = []
                continue
            time.sleep(0.5)



    def tx_thread(self):
        while True:
            while self.uart_connected == False:

                available_ports = self.auto_detect_serial_unix()
                if len(available_ports) == 0:
                    print("No Comports available")
                    time.sleep(0.5)
                    continue
                else:
                    print("uart connected")
                    try:
                        self.uart = serial.Serial(available_ports[0], 115200,timeout=1)
                    except:
                        print("access denied")
                    self.uart_connected = True
        
            while True:
                if (self.uart_connected == False):
                    break
                packet = []
                data = util.get_queue_data(self.receive_queue)
                if data is not None:
                    packet.append(data[1])
                    packet.append(data[2])
                    self.transmit_uart(data[0], packet)
                time.sleep(0.1)
    
    def transmit_uart(self, command, data):
        if (self.uart_connected == False):
            print("Uart not connected")
            return
        buffer = bytearray([packet.START_BYTE, command, len(data)])
        buffer.extend(data)

        try:
            self.uart.write(buffer)
        except:
            print("UART ERROR: uart write timeout\n")

    def run(self):
        tx = threading.Thread(target=self.tx_thread)
        tx.daemon = True
        tx.start()

        rx = threading.Thread(target=self.rx_thread)
        rx.daemon = True
        rx.start()

