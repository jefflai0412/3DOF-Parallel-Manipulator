import serial
import numpy as np
import matplotlib.pyplot as plt

# Set up serial port parameters (make sure to match Arduino's port and baud rate)
ser = serial.Serial('COM6', 115200, timeout=0)  # Reduced timeout for faster reads

# Function to convert Roll, Pitch, Yaw (in degrees) to rotation matrix
def rpy_to_rotation_matrix(roll, pitch, yaw):
    # Convert degrees to radians
    roll_rad = np.deg2rad(roll)
    pitch_rad = np.deg2rad(pitch)
    yaw_rad = np.deg2rad(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll_rad), -np.sin(roll_rad)],
                   [0, np.sin(roll_rad), np.cos(roll_rad)]])

    Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                   [0, 1, 0],
                   [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])

    Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                   [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                   [0, 0, 1]])

    return Rz @ Ry @ Rx

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Origin and axes for the plot
origin = np.array([[0], [0], [0]])
x_axis = np.array([[1], [0], [0]])
y_axis = np.array([[0], [1], [0]])
z_axis = np.array([[0], [0], [1]])

# Create initial lines for the axes
x_line, = ax.plot([], [], [], color='r', lw=2, label="X-axis")
y_line, = ax.plot([], [], [], color='g', lw=2, label="Y-axis")
z_line, = ax.plot([], [], [], color='b', lw=2, label="Z-axis")

# Set limits for the axes
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Initialize text labels for axis tips
x_text = ax.text(0, 0, 0, 'X', color='red', fontsize=12)
y_text = ax.text(0, 0, 0, 'Y', color='green', fontsize=12)
z_text = ax.text(0, 0, 0, 'Z', color='blue', fontsize=12)

# Function to update the plot
def update_plot(roll, pitch, yaw):
    R = rpy_to_rotation_matrix(roll, pitch, yaw)

    # Rotated axes
    x_rotated = R @ x_axis
    y_rotated = R @ y_axis
    z_rotated = R @ z_axis

    # Update line data for X, Y, Z axes
    x_line.set_data([origin[0, 0], x_rotated[0, 0]], [origin[1, 0], x_rotated[1, 0]])
    x_line.set_3d_properties([origin[2, 0], x_rotated[2, 0]])

    y_line.set_data([origin[0, 0], y_rotated[0, 0]], [origin[1, 0], y_rotated[1, 0]])
    y_line.set_3d_properties([origin[2, 0], y_rotated[2, 0]])

    z_line.set_data([origin[0, 0], z_rotated[0, 0]], [origin[1, 0], z_rotated[1, 0]])
    z_line.set_3d_properties([origin[2, 0], z_rotated[2, 0]])

    # Update the positions of axis labels (only occasionally, not every update)
    x_text.set_position((x_rotated[0, 0], x_rotated[1, 0]))
    x_text.set_3d_properties(x_rotated[2, 0])

    y_text.set_position((y_rotated[0, 0], y_rotated[1, 0]))
    y_text.set_3d_properties(y_rotated[2, 0])

    z_text.set_position((z_rotated[0, 0], z_rotated[1, 0]))
    z_text.set_3d_properties(z_rotated[2, 0])

    plt.draw()
    plt.pause(0.001)  # Allow some time for the GUI to update

# Real-time data reading and updating the plot
try:
    count = 0
    while True:
        line = ser.readline().decode('utf-8', errors='replace').strip()  # 'replace' to handle invalid bytes
        if line:
            try:
                # Parse serial data (in degrees)
                roll, pitch, yaw = map(float, line.split(','))
                print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw} (in degrees)")

                x = np.sin(pitch)
                y = np.sin(roll)

                joystick_command = f"joystick: {x:.2f},{y:.2f}\n"
                ser.write(joystick_command.encode())

                # Update the plot every 5 readings to reduce the frequency of updates
                count += 1
                if count % 7 == 0:
                    update_plot(roll, pitch, yaw)

            except ValueError as e:
                print(f"Error parsing data: {line}, Error: {e}")  # Print line that caused the error

except KeyboardInterrupt:
    ser.close()  # Close the serial port
    plt.close()  # Close the plot
