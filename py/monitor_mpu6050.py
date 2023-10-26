import serial
import matplotlib.pyplot as plt
import time

# Define the serial port and baud rate
serial_port = '/dev/ttyUSB0'  # Replace with your ESP32's serial port (e.g., COM3 on Windows)
baud_rate = 115200  # Set to match your ESP32's baud rate (commonly 115200)

# Initialize lists to store the data
timestamps = []
linear_acceleration = {'x': [], 'y': [], 'z': []}
angular_velocity = {'x': [], 'y': [], 'z': []}

# Create a figure for plotting
plt.figure(figsize=(10, 6))

try:
    # Create a serial connection
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Connected to {serial_port} at {baud_rate} baud")

    while True:
        data = ser.readline().decode('utf-8').strip()  # Read a line and decode it
        values = data.split(',')  # Split the data by semicolon

        # Ensure that the received data contains six values
        if len(values) == 6:
            t = time.time()  # Capture the current timestamp
            ax, ay, az, wx, wy, wz = map(float, values)
            timestamps.append(t)
            linear_acceleration['x'].append(ax)
            linear_acceleration['y'].append(ay)
            linear_acceleration['z'].append(az)
            angular_velocity['x'].append(wx)
            angular_velocity['y'].append(wy)
            angular_velocity['z'].append(wz)

            # Update the plot
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.title('Linear Acceleration')
            plt.plot(timestamps, linear_acceleration['x'], label='x')
            plt.plot(timestamps, linear_acceleration['y'], label='y')
            plt.plot(timestamps, linear_acceleration['z'], label='z')

            plt.subplot(2, 1, 2)
            plt.title('Angular Velocity')
            plt.plot(timestamps, angular_velocity['x'], label='x')
            plt.plot(timestamps, angular_velocity['y'], label='y')
            plt.plot(timestamps, angular_velocity['z'], label='z')

            plt.tight_layout()
            plt.pause(0.1)

except serial.SerialException as e:
    print(f"An error occurred: {e}")
finally:
    # Close the serial connection
    if 'ser' in locals() and ser is not None:
        ser.close()

plt.show()
