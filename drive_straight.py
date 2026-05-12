# SPIKE Prime - Drive Straight Until Left Button Pressed
# Robot: M4A3 Sherman | Left motor: port A | Right motor: port E
#
# Uses the hub's built-in gyroscope (yaw axis) with a P-controller
# to continuously correct the heading and keep the robot going straight.

import hub
import motor_pair

# --- Configuration ---
LEFT_PORT  = hub.port.A
RIGHT_PORT = hub.port.E

BASE_SPEED = 500   # degrees/sec — increase for faster, decrease for slower
KP         = 3.0   # proportional gain — raise if robot still drifts, lower if it wobbles

# --- Setup ---
motor_pair.pair(motor_pair.PAIR_1, LEFT_PORT, RIGHT_PORT)

# Reset yaw to 0 at the start so "straight" = 0 degrees
hub.motion_sensor.reset_yaw()
target_yaw = 0

# Show a go-arrow on the hub display
hub.display.show(hub.Image.GO_RIGHT)

# --- Drive straight until left button is pressed ---
while not hub.button.left.is_pressed():
    # tilt_angles() returns (yaw, pitch, roll) in decidegrees
    # Divide by 10 to convert to degrees
    yaw = hub.motion_sensor.tilt_angles()[0] / 10

    # Positive yaw = drifted right  →  steer left (increase right, decrease left)
    correction = int((target_yaw - yaw) * KP)
    correction = max(-200, min(200, correction))  # clamp to safe range

    motor_pair.move_tank(
        motor_pair.PAIR_1,
        BASE_SPEED + correction,   # left wheel
        BASE_SPEED - correction,   # right wheel
    )

# --- Stop ---
motor_pair.stop(motor_pair.PAIR_1, stop=motor_pair.BRAKE)
hub.display.show(hub.Image.SQUARE)
