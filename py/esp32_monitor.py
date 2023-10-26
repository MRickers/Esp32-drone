import serial

# Define the serial port and baud rate for your ESP32
serial_port = '/dev/ttyUSB0'  # Replace with your ESP32's serial port (e.g., COM3 on Windows)
baud_rate = 115200  # Set to match your ESP32's baud rate (commonly 115200)

try:
    # Create a serial connection
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Connected to {serial_port} at {baud_rate} baud")

    while True:
        # Read data from the serial port (change the number of bytes to read as needed)
        data = ser.readline()  # Read a line (terminated by newline character)
        # Check if the line contains a printf message (you can change this condition)
        if b'printf' in data:
            # Print the received data
            print(f"Received: {data.decode('utf-8').strip()}")  # Remove newline characters

except serial.SerialException as e:
    print(f"An error occurred: {e}")
finally:
    # Close the serial connection
    if 'ser' in locals() and ser is not None:
        ser.close()
