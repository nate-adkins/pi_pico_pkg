import serial
import time

# Define the serial port and baud rate
serial_port = '/dev/ttyACM0'  # Replace with the correct serial port of your Raspberry Pi Pico
baud_rate = 9600

# Open the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Define the string to send
string_to_send = "Hello, Raspberry Pi Pico!"

try:
    # Send the string
    ser.write(string_to_send.encode('utf-8'))
    print(f"Sent: {string_to_send}")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    ser.close()
