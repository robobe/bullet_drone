"""RC channel encoding/decoding and episode translation; no IO or physics."""
from math import pi

from bullet_drone.commands import HighLevelCommand, InvalidCommandError, RcInput, _finite, _values
from bullet_drone.config import ConfigurationError, RcSettings, _number, _vector
from bullet_drone.contracts import LifecycleError
from bullet_drone.models import DroneObservation
from bullet_drone.vehicle import load_racer_parameters


class RcCodec:
    """AETR to SI: positive body roll/pitch/yaw, throttle in body +Z newtons.

    Angle R/P use radians, Acro R/P and yaw in either mode use body rad/s.
    Calculated SI outputs are clipped and rounded (Python ties-to-even); external
    frames are strictly validated. Unused channels remain at 1000.
    """

    def __init__(self, max_thrust_n: float, max_rates_rad_s=(2., 2., 2.),
                 settings: RcSettings = RcSettings()):
        if not isinstance(settings, RcSettings):
            raise ConfigurationError('settings must be RcSettings')
        self.settings = settings
        self.max_thrust_n = _number(max_thrust_n, 'rc.max_thrust_n', positive=True)
        self.max_rates_rad_s = _vector(max_rates_rad_s, 'rc.max_rates_rad_s')
        if any(v <= 0 for v in self.max_rates_rad_s):
            raise ConfigurationError('RC rate limits must be positive')

    def encode(self, mode: str = 'angle', rpy=(0., 0., 0.), thrust_n: float = 0.,
               armed: bool = False) -> RcInput:
        """Here rpy[2] is always a yaw rate, never an absolute heading."""
        if mode not in ('angle', 'acro') or type(armed) is not bool:
            raise InvalidCommandError('RC encoding requires angle/acro and boolean armed')
        roll, pitch, yaw_rate = _values(rpy, 3, 'rpy')
        thrust = _finite(thrust_n, 'thrust_n')
        limits = ((self.settings.max_angle_rad,) * 2 + (self.max_rates_rad_s[2],)
                  if mode == 'angle' else self.max_rates_rad_s)
        sticks = [round(1500 + 500 * max(-1., min(1., value / limit)))
                  for value, limit in zip((roll, pitch, yaw_rate), limits)]
        channels = [1000] * 18
        a, e, t, r = (v - 1 for v in self.settings.aetr_channels)
        channels[a], channels[e], channels[r] = sticks
        channels[t] = round(1000 + 1000 * max(0., min(1., thrust / self.max_thrust_n)))
        channels[self.settings.arm_channel - 1] = 2000 if armed else 1000
        channels[self.settings.mode_channel - 1] = 2000 if mode == 'angle' else 1000
        return RcInput(tuple(channels))

    def decode(self, frame: RcInput, episode_id: str, step_index: int) -> HighLevelCommand:
        if not isinstance(frame, RcInput):
            raise InvalidCommandError('RC requires RcInput')
        channels = RcInput(frame.channels).channels
        a, e, t, r = (v - 1 for v in self.settings.aetr_channels)
        angle = channels[self.settings.mode_channel - 1] >= self.settings.angle_threshold
        limits = ((self.settings.max_angle_rad,) * 2 + (self.max_rates_rad_s[2],)
                  if angle else self.max_rates_rad_s)
        roll, pitch, yaw = ((channels[index] - 1500) / 500 * limit
                            for index, limit in zip((a, e, r), limits))
        return HighLevelCommand(
            'angle' if angle else 'acro', (roll, pitch, 0. if angle else yaw),
            (channels[t] - 1000) / 1000 * self.max_thrust_n, episode_id, step_index,
            channels[self.settings.arm_channel - 1] >= self.settings.arm_threshold,
            yaw if angle else None,
        )


def heading_rate(target_yaw_rad: float, measured_yaw_rad: float, limit_rad_s: float,
                 gain_s_inv: float = 4.) -> float:
    """Wrapped heading error to bounded body-Z rate for nearly level flight."""
    target = _finite(target_yaw_rad, 'target_yaw_rad')
    measured = _finite(measured_yaw_rad, 'measured_yaw_rad')
    limit = _finite(limit_rad_s, 'limit_rad_s')
    gain = _finite(gain_s_inv, 'gain_s_inv')
    if limit <= 0 or gain < 0:
        raise InvalidCommandError('Heading control needs positive rate limit and nonnegative gain')
    error = (target - measured + pi) % (2 * pi) - pi
    return max(-limit, min(limit, gain * error))


class SoftwareRc:
    """Reset/command/close source translating RC frames into stamped SI commands."""

    def __init__(self, codec: RcCodec | None = None) -> None:
        self.codec = codec if codec is not None else RcCodec(4 * load_racer_parameters().max_thrust_per_rotor_n)
        self._input = RcInput()
        self._episode_id: str | None = None
        self._next_step = 0
        self._closed = False
        self.last_command: HighLevelCommand | None = None

    def reset(self, observation: DroneObservation) -> None:
        if self._closed:
            raise LifecycleError('Cannot reset a closed RC source')
        self._input = RcInput()
        self.last_command = None
        self._episode_id = observation.ground_truth.episode_id
        self._next_step = observation.ground_truth.step_index + 1

    def set_input(self, value: RcInput) -> None:
        self._require_ready()
        if not isinstance(value, RcInput):
            raise InvalidCommandError('RC requires RcInput')
        self._input = RcInput(value.channels)

    def command(self, observation: DroneObservation, dt_s: float) -> HighLevelCommand:
        self._require_ready()
        if _finite(dt_s, 'dt_s') <= 0:
            raise InvalidCommandError('dt_s must be positive')
        state = observation.ground_truth
        if state.episode_id != self._episode_id or state.step_index + 1 != self._next_step:
            raise InvalidCommandError('RC observation is stale or belongs to another episode/step')
        command = self.codec.decode(self._input, state.episode_id, self._next_step)
        self._next_step += 1
        self.last_command = command
        return command

    def _require_ready(self) -> None:
        if self._closed or self._episode_id is None:
            raise LifecycleError('RC source must be reset before use')

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._episode_id = None
        self._input = RcInput()
        self.last_command = None
