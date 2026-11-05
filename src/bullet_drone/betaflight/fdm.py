"""FDM encoder for the upstream Betaflight Gazebo bridge."""
from math import isfinite
import struct

FDM_FORMAT = '<18d'
SEA_LEVEL_PRESSURE_PA = 101325.0
ATMOSPHERE_EXPONENT = 5.25588
ATMOSPHERE_ALTITUDE_FACTOR = 2.25577e-5


def pack_fdm(observation):
    state = observation.ground_truth
    imu, barometer = observation.sensors.imu, observation.sensors.barometer
    if imu is None or barometer is None:
        raise ValueError('Betaflight requires IMU and barometer readings')
    for reading in (imu, barometer):
        if (reading.episode_id, reading.step_index, reading.simulation_time_s) != (
                state.episode_id, state.step_index, state.simulation_time_s):
            raise ValueError('FDM sensors must belong to the current observation')
    gx, gy, gz = imu.angular_velocity_rad_s
    x, y, z, w = state.orientation_xyzw
    pressure = barometer.pressure_pa
    if not isfinite(pressure) or pressure <= 0:
        raise ValueError('Invalid FDM pressure')
    altitude = (1.0 - (pressure / SEA_LEVEL_PRESSURE_PA) ** (1.0 / ATMOSPHERE_EXPONENT)) / ATMOSPHERE_ALTITUDE_FACTOR
    values = (state.simulation_time_s, gx, -gy, -gz,
              *(-value for value in imu.specific_force_m_s2),
              w, x, -y, -z, *state.linear_velocity_m_s, 0.0, 0.0, altitude, pressure)
    if not all(isfinite(value) for value in values) or state.simulation_time_s < 0:
        raise ValueError('Invalid FDM values')
    return struct.pack(FDM_FORMAT, *values)
