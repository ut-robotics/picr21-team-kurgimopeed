###
#
# Code for debugging and testing communication with Kurkkumopo mainboard
#
# Currently you can set motor speeds, thrower speed and a specific command
# Commands are yet undefined, but will add something like detect_ball there
# Kurkkumopo will respond with status, encoder values and some extra data eg. ball_detected
#
###

import serial
import struct
import time
import random

crc_table = {}
MCU_ERRORS = ["MCU_OK", "MCU_BAD_USB_DATA"]

def int_to_bytes_rev(i):
    return [i & 0xFF, (i >> 8) & 0xFF, (i >> 16) & 0xFF, (i >> 24) & 0xFF]


def short_to_bytes_rev(i):
    return [i & 0xFF, (i >> 8) & 0xFF]


def generate_crc32_table(poly):
    global crc_table

    for i in range(256):
        c = i << 24

        for j in range(8):
            c = (c << 1) ^ poly if (c & 0x80000000) else c << 1

        crc_table[i] = c & 0xffffffff


def crc32(buf, t, crc=0xffffffff):
    global crc_table
    b = []

    for num in buf:
        if t == 2:
            b = short_to_bytes_rev(num)
        if t == 4:
            b = int_to_bytes_rev(num)

        for byte in b:
            crc = ((crc << 8) & 0xffffffff) ^ crc_table[(crc >> 24) ^ byte]

    return crc


def send_data(data):
    crc = crc32(speeds + [0], 2);
    crc &= 0xFFFF
    try:
        packed_bytes = struct.pack('<hhhHHH', *speeds, crc)
    except:
        print("Error in output data!")
        return 1
    try:
        ser.write(packed_bytes)
        print("Sent:", packed_bytes)
        return 0
    except:
        return 1


def receive_data():
    try:
        s = ser.read(20)
    except:
        print("Failed to get data!")
        return 1

    try:
        print("Received:", s)
        recv = struct.unpack('<llllHH', s)
    except:
        print("Failed to unpack data!")
        return 1

    recv_crc = recv[-1]

    # Change incoming CRC to 0 to calculate data CRC
    to_crc = list(recv[0:-1])
    to_crc.append(0)

    calculated_crc = crc32(to_crc[0:-2], 4)
    calculated_crc = crc32(to_crc[-2:], 2, crc=calculated_crc) & 0xffff
    if(recv_crc ^ calculated_crc):
        print("CRC mismatch!\nReceived CRC:", recv_crc, "\nCalculated CRC:", calculated_crc)
        return 1
    return recv


# Generates a mock array of 3 main engine speeds, thrower speed and a command
def generate_send_moc():
    return [random.randint(-2**15 + 1, 2**15),
            random.randint(-2**15 + 1, 2**15),
            random.randint(-2**15 + 1, 2**15),
            random.randint(0, 2**16),
            0]


crc32_poly = 0x04C11DB7
generate_crc32_table(crc32_poly)


if __name__ == "__main__":
    with serial.Serial('/dev/ttyACM1', 115200, timeout=3) as ser:
        while(1):
            speeds = generate_send_moc()
            if(send_data(speeds)):
                print("Failed to send data!")
            serial_data = receive_data()
            mcu_status = serial_data[0]
            print("STATUS:", MCU_ERRORS[mcu_status])
            time.sleep(0.5)
