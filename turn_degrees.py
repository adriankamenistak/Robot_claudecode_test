# =============================================================================
#  SPIKE Prime - Turn by Degrees
#  Robot: M4A3 Sherman  |  Left motor: port A  |  Right motor: port E
#
#  Uses the hub gyroscope to turn exactly the number of degrees you set.
#  Two-phase approach: fast for most of the turn, slow for the last 30°.
#  Unwrapped yaw tracking fixes the gyro wrap-around bug at ±180°.
# =============================================================================

import hub
import motor_pair

# -----------------------------------------------------------------------------
#  SETTINGS — change these
# -----------------------------------------------------------------------------

TURN_DEGREES = 90   # how far to turn | positive = right, negative = left

LEFT_PORT  = hub.port.A
RIGHT_PORT = hub.port.E

FAST_SPEED  = 400   # speed during the main part of the turn
SLOW_SPEED  = 100   # speed for the final 30 degrees
SLOW_ZONE   = 30    # degrees before target where we switch to slow speed
STOP_EARLY  = 1     # stop this many degrees before target to compensate for momentum


# -----------------------------------------------------------------------------
#  YAW TRACKING
#  tilt_angles()[0] wraps at ±180°, which breaks turns near 0°, 30°, >170°.
#  We fix this by accumulating small deltas each loop call instead of reading
#  the raw value directly.
# -----------------------------------------------------------------------------

_prev_raw  = 0.0
_total_yaw = 0.0

def reset_yaw():
    """Reset the cumulative yaw tracker and the hub gyro to zero."""
    global _prev_raw, _total_yaw
    hub.motion_sensor.reset_yaw()
    _prev_raw  = hub.motion_sensor.tilt_angles()[0] / 10
    _total_yaw = 0.0

def read_yaw():
    """Return total degrees turned since last reset_yaw(). No wrap-around limit."""
    global _prev_raw, _total_yaw
    raw   = hub.motion_sensor.tilt_angles()[0] / 10
    delta = raw - _prev_raw
    # Correct for wrap-around crossing ±180°
    if   delta >  180: delta -= 360
    elif delta < -180: delta += 360
    _total_yaw += delta
    _prev_raw   = raw
    return _total_yaw


# -----------------------------------------------------------------------------
#  MOVEMENT FUNCTIONS
# -----------------------------------------------------------------------------

def setup():
    """Pair the drive motors and reset the yaw tracker."""
    motor_pair.pair(motor_pair.PAIR_1, LEFT_PORT, RIGHT_PORT)
    reset_yaw()

def drive_turn(speed, direction):
    """Spin the robot in place.
    direction: +1 = right,  -1 = left
    speed: always a positive number
    """
    motor_pair.move_tank(motor_pair.PAIR_1,
                          direction * speed,
                         -direction * speed)

def stop():
    """Stop both drive motors."""
    motor_pair.stop(motor_pair.PAIR_1)

def turn(degrees):
    """Turn the robot exactly 'degrees' degrees then stop.
    Positive = right turn,  negative = left turn.

    Keeps turning until abs(yaw) exceeds the target — no complex conditions.
    Switches from fast to slow speed in the final SLOW_ZONE degrees.
    """
    direction  = 1 if degrees > 0 else -1
    abs_target = abs(degrees)

    reset_yaw()

    # Phase 1 — turn fast until we are SLOW_ZONE degrees away from target
    while abs(read_yaw()) < abs_target - SLOW_ZONE:
        drive_turn(FAST_SPEED, direction)

    # Phase 2 — turn slow, stop STOP_EARLY degrees before target to account for momentum
    while abs(read_yaw()) < abs_target - STOP_EARLY:
        drive_turn(SLOW_SPEED, direction)

    stop()


# -----------------------------------------------------------------------------
#  MAIN — runs when you press the play button
# -----------------------------------------------------------------------------

setup()
turn(TURN_DEGREES)
