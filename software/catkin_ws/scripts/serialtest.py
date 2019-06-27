import serial
import time
import binascii
import os

COMMAND_ECHO = bytearray.fromhex("62")
COMMAND_PING = bytearray.fromhex("61")
COMMAND_STEER = bytearray.fromhex("63")
START_BYTE = bytearray.fromhex("41")
END_BYTE = bytearray.fromhex("5A")

port = "/dev/ttyACM0"


# This function composes and transmits each command frame. 
# It must be provided with the command byte, data payload,
# and length of expected reply (if appropriate)
def send_frame(command, data, reply_length = 0):
    teensy = serial.Serial(port)
    frame = START_BYTE
    frame = frame + command + data
    frame = frame + bytearray.fromhex("00") # add checksum
    frame = frame + END_BYTE
    

    teensy.write(frame)
    if reply_length != 0:
        return [True, (binascii.hexlify(teensy.read(reply_length)))]

def request_echo(payload):
    responce = send_frame(COMMAND_ECHO, payload, 2)
    return responce 


count = 0
while True: 
    print(count)
    count += 1

    token = os.urandom(2)
    responce = request_echo(token)
    if count == responce: print("Good")
    time.sleep(1/ 50.0)
