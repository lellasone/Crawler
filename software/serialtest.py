import serial
import codecs
import sys

COMMAND_STEERING = bytearray.fromhex("62")

def set_steering(direction):
	if direction > 0 and direction < 256:
		
		message = bytes([direction]) + bytearray.fromhex("0000")
		send_frame(COMMAND_STEERING, message)
	else: 
		print("invalid direction request: " + str(direction))

def send_frame(command, data):
	with open("/dev/ttyACM2", 'wb') as teensy:
		frame = bytearray.fromhex("41")
		frame = frame + command + data
		frame = frame + bytearray.fromhex("5A")	
		print(frame)
		teensy.write(frame)

message = bytearray.fromhex("41620000005A")
print(message)
with open("/dev/ttyACM2", 'wb') as teensy:
	teensy.write(message)

set_steering(int(sys.argv[1]))

