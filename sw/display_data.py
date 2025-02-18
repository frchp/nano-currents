import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation

########## CONFIGURATION ##########

COM_PORT = "COM6"

######## END CONFIGURATION ########

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

def read_serial(port= COM_PORT, baudrate=115200):
  ser = serial.Serial(port, baudrate, timeout=1)
  return ser

def update(frame, ser, currents, voltages, ax1, ax2):
  data = ser.read(5)  # Read 5 bytes
  if len(data) == 5:
    current = struct.unpack('<H', data[0:2])[0]  # Little-endian 16-bit
    voltage = struct.unpack('<H', data[2:4])[0]  # Little-endian 16-bit
    received_crc = data[4]
    computed_crc = calculate_crc8(data[:4])

    if received_crc == computed_crc:
      currents.append(current)
      voltages.append(voltage)
      if len(currents) > 100:
        currents.pop(0)
        voltages.pop(0)

      ax1.clear()
      ax2.clear()
      ax1.plot(currents, label='Current (mA)')
      ax2.plot(voltages, label='Voltage (mV)', color='r')
      ax1.legend()
      ax2.legend()
      ax1.set_ylim(0, max(currents) + 50)
      ax2.set_ylim(0, max(voltages) + 50)

def main():
  ser = read_serial()
  currents, voltages = [], []

  fig, (ax1, ax2) = plt.subplots(2, 1)
  ani = animation.FuncAnimation(fig, update, fargs=(ser, currents, voltages, ax1, ax2), interval=100)
  plt.show()
  ser.close()

if __name__ == "__main__":
  main()
