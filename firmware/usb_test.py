import serial
import struct
import zlib
import time

custom_crc_table = {}

def int_to_bytes(i):
    return [(i >> 24) & 0xFF, (i >> 16) & 0xFF, (i >> 8) & 0xFF, i & 0xFF]


def generate_crc32_table(_poly):

    global custom_crc_table

    for i in range(256):
        c = i << 24

        for j in range(8):
            c = (c << 1) ^ _poly if (c & 0x80000000) else c << 1

        custom_crc_table[i] = c & 0xffffffff


def custom_crc32(buf):

    global custom_crc_table
    crc = 0xffffffff

    for integer in buf:
        b = int_to_bytes(integer)

        for byte in b:
            crc = ((crc << 8) & 0xffffffff) ^ custom_crc_table[(crc >> 24) ^ byte]

    return crc

poly = 0x04C11DB7
generate_crc32_table(poly)

speed1 = speed2 = speed3 = 30
thrower_speed = 2000
speeds = [30, 30, 30, 2000]

with serial.Serial('/dev/ttyACM1', 115200, timeout=3) as ser:
    while True:
        crc = 0
        for speed in speeds:
            crc = zlib.crc32(speed.to_bytes(2, 'big'), crc)
        crc &= 0xFF
        send = struct.pack('<hhhHB', *speeds, crc)
        print("Sent:", send)
        ser.write(send)
        s = ser.read(20)
        recv = struct.unpack('<lllll', s)
        print("Received:", recv)
        to_crc = list(recv[0:-1])
        to_crc.append(0)
        custom_crc = custom_crc32(to_crc)
        print(custom_crc & 0xFF)
        error, actual_speed1, actual_speed2, actual_speed3, crc = recv
        time.sleep(0.5)
