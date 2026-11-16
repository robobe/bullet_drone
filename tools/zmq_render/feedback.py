"""Map immutable simulation observations into process-neutral telemetry."""
from bullet_simulation_protocol.feedback import (
    TimeStamp, Header, GroundTruth, Imu, Barometer, Rangefinder, Gps,
    AttitudeEstimate, Sensors, FlightFeedback,
)


def from_observation(observation):
    state, sensors = observation.ground_truth, observation.sensors
    body_frame = f'{state.drone_id}/base_link'

    def header(reading, frame='world'):
        return Header(frame, TimeStamp.from_seconds(reading.simulation_time_s))

    ground = GroundTruth(header(state), state.position_m, state.orientation_xyzw,
                         state.position_m[2], state.linear_velocity_m_s,
                         state.angular_velocity_rad_s, state.rotor_speeds_rad_s)
    imu, barometer, rangefinder, gps = sensors.imu, sensors.barometer, sensors.rangefinder, sensors.gps
    estimate = observation.attitude_estimate
    return FlightFeedback(header(state), state.drone_id, ground,
        Sensors(
            None if imu is None else Imu(header(imu, body_frame), imu.specific_force_m_s2, imu.angular_velocity_rad_s),
            None if barometer is None else Barometer(header(barometer), barometer.pressure_pa, barometer.altitude_m),
            None if rangefinder is None else Rangefinder(header(rangefinder, body_frame), rangefinder.distance_cm, rangefinder.valid),
            None if gps is None else Gps(header(gps, 'earth'), gps.latitude_deg, gps.longitude_deg,
                                        gps.altitude_m, gps.velocity_enu_m_s)),
        None if estimate is None else AttitudeEstimate(header(estimate), estimate.orientation_xyzw))
