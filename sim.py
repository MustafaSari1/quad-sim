import socket
import time
import struct
import json
from QuadCopter import QuadCopter
from parameters import *

time_now = 0
step_time = 0
connected = False

quad = QuadCopter()

# Initialize the simulation / Reset physics
def init():
    global time_now
    time_now = 0

    global quad
    quad.__reset__()


# Calculate the new state of the physics
# Return timestamp, gyro(angular vel), accel, position, attitude, velocity
def calculate_new_state(pwm_values):

    global step_time
    global time_now
    time_now += DT

    rotor_velocities = pwm_values[0:4]

    if DEBUG_PWM and DEBUG_TIME:
        print(rotor_velocities)
    gyro_body_rad_s, accel_body_m_ss, position_m, attitude_rad, velocity_m_s = quad.step(rotor_velocities, step_time)
    
    return time_now, gyro_body_rad_s, accel_body_m_ss, position_m, attitude_rad, velocity_m_s

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
    if (time.time() * 1000) % DEBUG_PERIOD_MS < 1:
        DEBUG_TIME = True
    else:
        DEBUG_TIME = False

    try:
        # Receive data from the SITL
        data, address = udp_socket.recvfrom(1024)
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
        DT = 1.0 / SIM_RATE

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
    physics_time_s, gyro, accel, pos, att, vel = calculate_new_state(pwm_values)

    # Convert outputs to JSON
    IMU_format = {
        "gyro": [gyro[0][0], gyro[1][0], gyro[2][0]],
        "accel_body": [accel[0][0], accel[1][0], -accel[2][0] - GRAVITY_M_S2]
    }

    JSON_format = {
        "timestamp": physics_time_s,
        "imu": IMU_format,
        "position": [pos[0][0], pos[1][0], -pos[2][0]],
        "attitude": [att[0][0], att[1][0], att[2][0]],
        "velocity": [vel[0][0], vel[1][0], -vel[2][0]]
    }

    JSON_string = "\n" + json.dumps(JSON_format, separators=(',', ':')) + "\n"
    if DEBUG_JSON and DEBUG_TIME:
        print(JSON_string)
    # time.sleep()

    # Send outputs to the SITL/ArduPilot
    udp_socket.sendto(bytes(JSON_string, "ascii"), address)

    # Track performance
    now = time.time()
    step_time = now - frame_time
    # print("fps=%.2f  timestamp=%.3f dt=%.3f" % (print_frame_count/step_time, physics_time_s, step_time))
    frame_time = now
