# import select
# import sys
# import time

# BATTERY_MONITOR_DRIVER = False
# PAYLOAD_DRIVER = True

# READ_POLL_TIMEOUT_MS = 2

# # Set up the poll object
# poll_obj = select.poll()
# poll_obj.register(sys.stdin, select.POLLIN)


# def pico_print(text: str):
#     print(text)
#     sys.stdout.buffer.write(text + "\r")
#     sys.stdout.buffer.flush()  # Ensure immediate output


# pico_print("Selecting operating mode")


# def battery_monitor():
#     pass

# def payload_driver():
#     pass

# def main():

#     if BATTERY_MONITOR_DRIVER and PAYLOAD_DRIVER:
#         pico_print(f"ERROR, two operating modes selected: BATTERY_MONITOR_DRIVER = {BATTERY_MONITOR_DRIVER}, PAYLOAD_DRIVER = {PAYLOAD_DRIVER}")
#         battery_monitor()
#     elif BATTERY_MONITOR_DRIVER:
#         pico_print("Operating mode selected: BATTERY_MONITOR_DRIVER")

#     elif PAYLOAD_DRIVER:
#         pico_print("Operating mode selected: PAYLOAD_DRIVER")

#         n = 0
#         while True:
#             n += 1
#             # poll_results = poll_obj.poll(READ_POLL_TIMEOUT_MS)  # poll function already expects milliseconds
#             # if poll_results:
#             pico_print(f'counter: {n}')
#             # else:
#                 # time.sleep(0.001)  # Wait a bit to avoid CPU hogging

#     else:
#         pico_print(f"ERROR, no operating mode selected: BATTERY_MONITOR_DRIVER = {BATTERY_MONITOR_DRIVER}, PAYLOAD_DRIVER = {PAYLOAD_DRIVER}")
#         sys.exit()


import machine
import time

# Define the serial port
uart = machine.UART(0, baudrate=9600)

# Function to read from serial port
def read_serial():
    if uart.any():
        return uart.readline().decode().strip()

# Main loop
while True:
    received_string = read_serial()
    if received_string:
        print("Received:", received_string)
    time.sleep(0.1)





