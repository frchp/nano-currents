import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation

########## CONFIGURATION ##########

COM_PORT = "COM6"
START_BYTE = b'\xA5'
FRAME_LENGTH = 6

# Constants for calculations
ADC_REF_MV = 3300  # Reference voltage in mV
ADC_REF_LSB = 0xFFF  # Max ADC value (12-bit resolution)
VOLTAGE_AMP_IN_RATIO = 2  # Amplification ratio for voltage
CURRENT_AMP_GAIN = 500  # Current amplifier gain
CURRENT_AMP_SHUNT = 10  # Shunt resistor value in ohms

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
  while True:
    start_byte = ser.read(1)
    if start_byte == START_BYTE:
      data = ser.read(FRAME_LENGTH - 1)  # Read remaining 5 bytes
      if len(data) == (FRAME_LENGTH - 1):
        frame_data = start_byte + data
        raw_current = struct.unpack('<H', frame_data[3:5])[0]  # Little-endian 16-bit
        raw_voltage = struct.unpack('<H', frame_data[1:3])[0]  # Little-endian 16-bit
        received_crc = frame_data[5]
        computed_crc = calculate_crc8(frame_data[0:5])

        if received_crc == computed_crc:
          # Convert raw ADC values to actual measurements
          voltage = (raw_voltage * ADC_REF_MV / ADC_REF_LSB) * VOLTAGE_AMP_IN_RATIO
          current = (raw_current * ADC_REF_MV / ADC_REF_LSB) / (CURRENT_AMP_GAIN * CURRENT_AMP_SHUNT)

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
        break

def main():
  ser = read_serial()
  currents, voltages = [], []

  fig, (ax1, ax2) = plt.subplots(2, 1)
  ani = animation.FuncAnimation(fig, update, fargs=(ser, currents, voltages, ax1, ax2), interval=100)
  plt.show()
  ser.close()

if __name__ == "__main__":
  main()
