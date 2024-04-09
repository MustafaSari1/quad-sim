import socket
import time
import struct
import json

SIM_RATE = 400
TIME_STEP_S = 1.0 / SIM_RATE
GRAVITY_M_SS = 9.80665

time_now = 0
connected = False

# Initialize the simulation / Reset physics
def init():
    global time_now
    time_now = 0
    
    # Reset position and orientation

def calculate_new_state(pwm_values):
    # Calculate the new state of the physics
    # Return timestamp, gyro, accel, position, attitude, velocity
    return 0

# Create a socket object for UDP communication and bind it to port 9002 on the local machine
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(('', 9002))
udp_socket.settimeout(0.01)

last_SITL_frame_count = -1
frame_count = 0
frame_time = time.time()
print_frame_count = 1000

# Specify the format of the data
parse_format = 'HHI16H'

# Magic value to verify the data
magic_value = 18458

while True:
    try:
        # Receive data from the SITL
        data, address = udp_socket.recvfrom(100)
    except socket.timeout:
        print('No data received')
        continue

    # Check if the data is appropriate size
    if len(data) != struct.calcsize(parse_format):
        print('Data is not the correct size')
        continue

    # Unpack the data
    unpacked_data = struct.unpack(parse_format, data)

    # Check if the magic value is correct
    if unpacked_data[0] != magic_value:
        print('Incorrect magic value %d, it should be %d' % (unpacked_data[0], magic_value))
        continue

    # Extract the frame rate, frame count, and PWM values
    frame_rate_hz = unpacked_data[1]
    frame_count = unpacked_data[2]
    pwm_values = unpacked_data[3:]

    # Check if the frame rate is different from the current frame rate and update
    if frame_rate_hz != SIM_RATE:
        print('New frame rate %d' % frame_rate_hz)
        SIM_RATE = frame_rate_hz
        TIME_STEP_S = 1.0 / SIM_RATE

    if frame_count < last_SITL_frame_count:
        print('Frame count decreased, resetting physics...')
        init()
    elif frame_count == last_SITL_frame_count:
        print('Duplicate frame count %d' % frame_count)
        continue
    elif frame_count != last_SITL_frame_count + 1 and connected:
        print('Missed %d input frames' % (frame_count - last_SITL_frame_count - 1))
    
    # Keep track of the last frame count
    last_SITL_frame_count = frame_count

    # Check if the simulation is connected
    if not connected:
        connected = True
        print('Connected to %s', str(address))
    
    # Increase the frame count
    frame_count += 1

    # Get outputs from the physics
    physics_time_s, gyro_body, accel_body, position_earth, attitude, velocity_earth = calculate_new_state(pwm_values)

    # Convert outputs to JSON
    IMU_format = {
        "gyro": gyro_body,
        "accel": accel_body
    }

    JSON_format = {
        "timestamp": physics_time_s,
        "imu": IMU_format,
        "position": position_earth,
        "attitude": attitude,
        "velocity": velocity_earth
    }

    JSON_string = "\n" + json.dumps(JSON_format, separators=(',', ':')) + "\n"

    # Send outputs to the SITL/ArduPilot
    udp_socket.sendto(bytes(JSON_string, "ascii"), address)

    # Track performance
    if frame_count % print_frame_count == 0:
        now = time.time()
        step_time = now - frame_time
        print("fps=%.2f  timestamp=%.3f dt=%.3f" % (print_frame_count/step_time, physics_time_s, step_time))
        frame_time = now