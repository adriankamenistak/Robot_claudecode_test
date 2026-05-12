import hub
import motor_pair

# =============================================
#  SET YOUR TURN ANGLE HERE
#  Positive = right,  Negative = left
# =============================================
TURN_DEGREES = 90

LEFT_PORT  = hub.port.A
RIGHT_PORT = hub.port.E
FAST_SPEED = 400   # speed during phase 1 (far from target)
SLOW_SPEED = 100   # speed during phase 2 (last 30 degrees)

motor_pair.pair(motor_pair.PAIR_1, LEFT_PORT, RIGHT_PORT)
hub.motion_sensor.reset_yaw()

# --- Unwrapped yaw tracker ---
# tilt_angles()[0] wraps at ±180°, which breaks turns near 0°, 30°, or >170°.
# Instead we accumulate small deltas each loop so wrap-around is never an issue.
prev_raw    = hub.motion_sensor.tilt_angles()[0] / 10
total_yaw   = 0.0

def read_yaw():
    global prev_raw, total_yaw
    raw   = hub.motion_sensor.tilt_angles()[0] / 10
    delta = raw - prev_raw
    if   delta >  180: delta -= 360   # crossed +180 → -180 boundary
    elif delta < -180: delta += 360   # crossed -180 → +180 boundary
    total_yaw += delta
    prev_raw   = raw
    return total_yaw

direction  = 1 if TURN_DEGREES > 0 else -1
abs_target = abs(TURN_DEGREES)

# Phase 1: fast, until 30° before target
while True:
    turned = read_yaw()
    if direction * turned >= abs_target - 30:
        break
    motor_pair.move_tank(motor_pair.PAIR_1,
                          direction * FAST_SPEED,
                         -direction * FAST_SPEED)

# Phase 2: slow, until 1° before target
while True:
    turned = read_yaw()
    if direction * turned >= abs_target - 1:
        break
    motor_pair.move_tank(motor_pair.PAIR_1,
                          direction * SLOW_SPEED,
                         -direction * SLOW_SPEED)

motor_pair.stop(motor_pair.PAIR_1)
