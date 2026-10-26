"""Pure encoder for the pinned IMU-only, external-pressure SITL build."""
from math import isfinite
import struct

FDM_FORMAT = '<18d'
RESERVED = (0.,) * 6
UNUSED_QUATERNION = (1., 0., 0., 0.)


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
    values = (state.simulation_time_s, gx, -gy, -gz,
              *(-value for value in imu.specific_force_m_s2),
              *UNUSED_QUATERNION, *RESERVED, barometer.pressure_pa)
    if not all(isfinite(value) for value in values) or state.simulation_time_s < 0 or barometer.pressure_pa <= 0:
        raise ValueError('Invalid FDM time, sensor or pressure value')
    return struct.pack(FDM_FORMAT, *values)
