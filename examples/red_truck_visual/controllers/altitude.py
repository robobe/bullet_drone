"""Altitude controller tuning for the red-truck mission."""
from bullet_drone.controllers import PidSettings


ALTITUDE_POSITION_PID = PidSettings(1.5, 0.0, 0.0, 8.0, 0.0)
ALTITUDE_VELOCITY_PID = PidSettings(2.0, 1.0, 0.0, 5.0, 2.0)
