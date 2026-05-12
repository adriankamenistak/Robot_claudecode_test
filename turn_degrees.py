import hub
import motor_pair

# =============================================
#  SET YOUR TURN ANGLE HERE
#  Positive = right turn, Negative = left turn
# =============================================
TURN_DEGREES = 90

# --- Config (don't need to change these) ---
LEFT_PORT  = hub.port.A
RIGHT_PORT = hub.port.E
MAX_SPEED  = 400   # max turn speed (degrees/sec)
MIN_SPEED  = 150   # min speed so motors don't stall near target
KP         = 4.0   # raise if it undershoots, lower if it overshoots
TOLERANCE  = 3     # stop when within this many degrees of target

# --- Setup ---
motor_pair.pair(motor_pair.PAIR_1, LEFT_PORT, RIGHT_PORT)
hub.motion_sensor.reset_yaw()

# --- Turn loop ---
# P-controller: speed is proportional to remaining error.
# Left forward + right backward = turn right (positive yaw on SPIKE Prime).
# If your robot turns the WRONG direction, flip the sign of TURN_DEGREES.
while True:
    yaw   = hub.motion_sensor.tilt_angles()[0] / 10  # decidegrees -> degrees
    error = TURN_DEGREES - yaw

    if abs(error) < TOLERANCE:
        break

    speed = int(error * KP)
    speed = max(-MAX_SPEED, min(MAX_SPEED, speed))

    # Enforce minimum speed so motors don't stall
    if   speed > 0: speed = max(speed,  MIN_SPEED)
    elif speed < 0: speed = min(speed, -MIN_SPEED)

    motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)

motor_pair.stop(motor_pair.PAIR_1)
