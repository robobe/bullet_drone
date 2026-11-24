"""Convert normalized camera target error into attitude and altitude targets."""
from math import radians

from examples.red_truck_visual.controllers.altitude import ALTITUDE_POSITION_PID
from examples.red_truck_visual.vision import TargetOffset


LOW_ALTITUDE_APPROACH_DESCENT_M_S = 4.0
FULL_ALTITUDE_APPROACH_DESCENT_M_S = 4.5
IMAGE_DESCENT_GAIN_M_S = 4.0
FULL_IMAGE_GUIDANCE_ALTITUDE_M = 40.0
MIN_IMAGE_GUIDANCE_SCALE = 0.5
MAX_TILT_RAD = radians(30.0)
MAX_FORWARD_PITCH_RAD = radians(22.0)
IMAGE_ROLL_GAIN_RAD = radians(24.0)
IMAGE_PITCH_GAIN_RAD = radians(10.0)
IMAGE_ROLL_D_GAIN_RAD_S = radians(30.0)
IMAGE_D_FILTER_S = 0.2
PITCH_SLEW_RAD_S = radians(40.0)
GROUND_ALTITUDE_M = 0.0
ZERO_PITCH_RAD = 0.0
ZERO_IMAGE_ERROR_RATE_S_INV = 0.0
MAX_IMAGE_GUIDANCE_SCALE = 1.0
NORMALIZED_IMAGE_ERROR_RANGE = 1.0
MIN_FORWARD_PITCH_FACTOR = 0.2
MIN_APPROACH_PITCH_RAD = radians(5.0)
IMAGE_GUIDANCE_SCALE_RANGE = MAX_IMAGE_GUIDANCE_SCALE - MIN_IMAGE_GUIDANCE_SCALE


class ImageGuidanceController:
    def __init__(self):
        self.last_dx = None
        self.last_frame_step = None
        self.dx_rate = ZERO_IMAGE_ERROR_RATE_S_INV
        self.last_pitch_rad = ZERO_PITCH_RAD

    def reset(self):
        self.last_dx = None
        self.last_frame_step = None
        self.dx_rate = ZERO_IMAGE_ERROR_RATE_S_INV
        self.last_pitch_rad = ZERO_PITCH_RAD

    def level_pitch(self, dt_s):
        max_change = PITCH_SLEW_RAD_S * dt_s
        pitch = max(self.last_pitch_rad - max_change,
                    min(self.last_pitch_rad + max_change, ZERO_PITCH_RAD))
        self.last_pitch_rad = pitch
        return pitch

    def approach(self, target: TargetOffset, altitude_m, dt_s, frame_step=None):
        if frame_step is not None and (
                self.last_frame_step is None or frame_step > self.last_frame_step):
            if self.last_frame_step is not None:
                frame_dt_s = (frame_step - self.last_frame_step) * dt_s
                rate = (target.dx - self.last_dx) / frame_dt_s
                self.dx_rate += frame_dt_s / (IMAGE_D_FILTER_S + frame_dt_s) * (rate - self.dx_rate)
            self.last_dx = target.dx
            self.last_frame_step = frame_step
        guidance_scale = max(MIN_IMAGE_GUIDANCE_SCALE,
                             min(MAX_IMAGE_GUIDANCE_SCALE,
                                 altitude_m / FULL_IMAGE_GUIDANCE_ALTITUDE_M))
        descent_m_s = LOW_ALTITUDE_APPROACH_DESCENT_M_S + (
            (guidance_scale - MIN_IMAGE_GUIDANCE_SCALE) / IMAGE_GUIDANCE_SCALE_RANGE
            * (FULL_ALTITUDE_APPROACH_DESCENT_M_S - LOW_ALTITUDE_APPROACH_DESCENT_M_S))
        roll = max(-MAX_TILT_RAD, min(MAX_TILT_RAD,
            target.dx * IMAGE_ROLL_GAIN_RAD + self.dx_rate * IMAGE_ROLL_D_GAIN_RAD_S))
        pitch = max(MIN_APPROACH_PITCH_RAD, min(MAX_TILT_RAD,
            MAX_FORWARD_PITCH_RAD * max(
                MIN_FORWARD_PITCH_FACTOR, NORMALIZED_IMAGE_ERROR_RANGE - abs(target.dx))
            + target.dy * IMAGE_PITCH_GAIN_RAD * guidance_scale))
        max_change = PITCH_SLEW_RAD_S * dt_s
        pitch = max(self.last_pitch_rad - max_change,
                    min(self.last_pitch_rad + max_change, pitch))
        self.last_pitch_rad = pitch
        vertical_target = max(GROUND_ALTITUDE_M, altitude_m -
                              (descent_m_s + target.dy * IMAGE_DESCENT_GAIN_M_S)
                              / ALTITUDE_POSITION_PID.kp)
        return roll, pitch, vertical_target
