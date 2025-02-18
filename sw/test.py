import serial
import struct
import time
import random

# Use https://freevirtualserialports.com/ to emulate serial port
ser = serial.Serial("COM2", 115200)  # COM8 is the other virtual port

def calculate_crc8(data):
  crc = 0x00
  poly = 0x07  # Standard CRC-8 polynomial
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF  # Ensure 8-bit value
  return crc

while True:
  start_bt = 0xA5
  voltage = random.randint(2250, 2750)  # Example ADC value
  current = random.randint(10000, 50000)  # Example ADC value
  frame = struct.pack('<BHH', 0xA5, voltage, current)  # Frame without CRC
  crc = calculate_crc8(frame[0:])  # Compute CRC over voltage and current bytes
  data = frame + struct.pack('<B', crc)  # Append CRC byte
  ser.write(data)
  print(f"Sent: Voltage={voltage}, Current={current}")
  time.sleep(0.1)