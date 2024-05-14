import serial


def main():
    s = serial.Serial(port="/dev/ttyACM1", parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=1)
    s.flush()

    s.write("data\r".encode())
    mes = s.read_until().strip()
    print(mes.decode())

if __name__ == "__main__":
    main()