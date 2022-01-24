import struct

class MainBoardComm():
    def __init__(self, ser):
        self.ser = ser
        self.MCU_ERRORS = ["MCU_OK", "MCU_BAD_USB_DATA"]

        crc32_poly = 0x04C11DB7
        self.crc_table = self.generate_crc32_table(crc32_poly)

    
    def t_to_bytes_rev(self, i, t):
        for b in range(t):
            yield (i >> 8*b) & 0xFF

    def generate_crc32_table(self, poly):
        crc_table = []
        for i in range(256):
            c = i << 24

            for _ in range(8):
                c = (c << 1) ^ poly if (c & 0x80000000) else c << 1

            crc_table.append(c & 0xffffffff)
        return crc_table

    def crc32(self, buf, t, crc=0xffffffff):
        for num in buf:
            for byte in self.t_to_bytes_rev(num, t):
                crc = ((crc << 8) & 0xffffffff) ^ self.crc_table[(crc >> 24) ^ byte]

        return crc

    def send_data(self, *data):
        #print("sent", data)
        crc = self.crc32(list(data) + [0], 2)
        crc &= 0xFFFF
        try:
            packed_bytes = struct.pack('<HHHHHHHH', *data, crc)
            self.ser.write(packed_bytes)
            #print("Sent:", packed_bytes)
        except Exception as e:
            print("Error in output data!", e)
            return True
        return False

    def receive_data(self):
        try:
            s = self.ser.read(20)
            if s == b'':
                return None
            #print("Received:", s)
            recv = struct.unpack('<llllHH', s)
        except Exception as e:
            print("Exception in read data:", e)
            return 1

        *to_crc, recv_crc = recv

        # Change incoming CRC to 0 to calculate data CRC
        to_crc.append(0)

        calculated_crc = self.crc32(to_crc[0:-2], 4)
        calculated_crc = self.crc32(to_crc[-2:], 2, crc=calculated_crc) & 0xffff
        if(recv_crc ^ calculated_crc):
            print("CRC mismatch!\nReceived CRC: ",recv_crc,"\nCalculated CRC: ", calculated_crc)
            return None

        #print("MCU_STATUS", self.MCU_ERRORS[recv[0]])
        return to_crc[:-1]

if __name__ == "__main__":
    import random
    import time
    import serial

    # Generates a mock array of 3 main engine speeds, thrower speed and a command
    def generate_send_moc():
        return (random.randint(-2**15 + 1, 2**15),
                random.randint(-2**15 + 1, 2**15),
                random.randint(-2**15 + 1, 2**15),
                random.randint(0, 2**16),
                0)

    with serial.Serial('/dev/ttyACM1', 115200, timeout=3) as ser:
        comm = MainBoardComm(ser)
        while(1):
            speeds = generate_send_moc()
            if(comm.send_data(*speeds)):
                print("Failed to send data!")
            data = comm.receive_data()
            print(data)
            time.sleep(0.5)
