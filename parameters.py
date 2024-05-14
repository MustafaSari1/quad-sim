# Simulation parameters
SIM_RATE = 400
DT = 1.0 / SIM_RATE
GRAVITY_M_S2 = 9.81

# Quadcopter parameters
MASS_KG = 10

IXX = 1
IYY = 1
IZZ = 1

K = 1e-5 # Thrust constants
A = 0.5 
B = 1e-6 # Drag constant 
L = 1 # Arm length (m)

DEBUG_PWM = True
DEBUG_JSON = True
DEBUG_ANGULAR_ACCEL = True
DEBUG_PERIOD_MS = 500