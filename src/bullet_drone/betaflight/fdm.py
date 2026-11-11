"""FDM encoder for the upstream Betaflight Gazebo bridge."""
from math import isfinite
import struct

from bullet_drone.config import _number, _vector

FDM_FORMAT = '<18d'
SEA_LEVEL_PRESSURE_PA = 101325.0
ATMOSPHERE_EXPONENT = 5.25588
ATMOSPHERE_ALTITUDE_FACTOR = 2.25577e-5
GPS_MAX_VELOCITY_M_S = 327.67
GPS_MIN_ALTITUDE_M = -21474836.48
GPS_MAX_ALTITUDE_M = 21474836.47


def pack_fdm(observation, *, gps_enabled=False):
    state = observation.ground_truth
    imu, barometer = observation.sensors.imu, observation.sensors.barometer
    if imu is None or barometer is None:
        raise ValueError('Betaflight requires IMU and barometer readings')
    for reading in (imu, barometer):
        if (reading.episode_id, reading.step_index, reading.simulation_time_s) != (
                state.episode_id, state.step_index, state.simulation_time_s):
            raise ValueError('FDM sensors must belong to the current observation')
    gps = observation.sensors.gps
    if gps_enabled and gps is None:
        raise ValueError('GPS-enabled FDM requires a GPS reading')
    if gps is not None and not gps_enabled:
        raise ValueError('GPS reading requires explicitly enabled GPS FDM')
    if gps_enabled:
        if (gps.episode_id, gps.step_index, gps.simulation_time_s) != (
                state.episode_id, state.step_index, state.simulation_time_s):
            raise ValueError('FDM GPS must belong to the current observation')
        latitude = _number(gps.latitude_deg, 'gps.latitude_deg')
        longitude = _number(gps.longitude_deg, 'gps.longitude_deg')
        gps_altitude = _number(gps.altitude_m, 'gps.altitude_m')
        gps_velocity = _vector(gps.velocity_enu_m_s, 'gps.velocity_enu_m_s')
        if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
            raise ValueError('Invalid FDM GPS coordinates')
        if not GPS_MIN_ALTITUDE_M <= gps_altitude <= GPS_MAX_ALTITUDE_M:
            raise ValueError('FDM GPS altitude exceeds firmware int32 centimetres')
        if any(abs(value) > GPS_MAX_VELOCITY_M_S for value in gps_velocity):
            raise ValueError('FDM GPS velocity exceeds firmware int16 centimetres/second')
    gx, gy, gz = imu.angular_velocity_rad_s
    x, y, z, w = state.orientation_xyzw
    pressure = barometer.pressure_pa
    if not isfinite(pressure) or pressure <= 0:
        raise ValueError('Invalid FDM pressure')
    # Default upstream Gazebo ignores pressure, so encode it through position Z.
    altitude = (1.0 - (pressure / SEA_LEVEL_PRESSURE_PA) ** (1.0 / ATMOSPHERE_EXPONENT)) / ATMOSPHERE_ALTITUDE_FACTOR
    position = (longitude, latitude, gps_altitude) if gps_enabled else (0.0, 0.0, altitude)
    velocity = gps_velocity if gps_enabled else state.linear_velocity_m_s
    values = (state.simulation_time_s, gx, -gy, -gz,
              *(-value for value in imu.specific_force_m_s2),
              w, x, -y, -z, *velocity, *position, pressure)
    if not all(isfinite(value) for value in values) or state.simulation_time_s < 0:
        raise ValueError('Invalid FDM values')
    return struct.pack(FDM_FORMAT, *values)
