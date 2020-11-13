from abc import ABC as AbstractBaseClass
from abc import abstractmethod
import dataclasses
from dataclasses import dataclass

import random

from pid_controller.utils import clamp


@dataclass
class StepOutput:
    system_state: float
    error_signal: float = None
    controller_output: float = None


@dataclass
class ErrorSignal:
    proportional: float = None
    differential: float = None
    integral: float = None


@dataclass
class Gain:
    proportional: float = 0.0
    differential: float = 0.0
    integral: float = 0.0


class Controller:
    def __init__(self, gains: Gain = Gain()) -> None:
        self._gains = gains

    @staticmethod
    @abstractmethod
    def apply(error_signal: ErrorSignal) -> float:
        """Calculate the actual control command."""
        raise NotImplementedError

    def set_gains(self, gains: Gain) -> None:
        """Overwrite all gains with a full Gain object."""
        self._gains = gains

    def set_gain(self, gain_type: str, value: float) -> None:
        """Set only a single gain value by key."""
        if gain_type not in dataclasses.asdict(self._gains).keys():
            print(f'WARNING: Provided key for gain value {gain_type} invalid. Ignoring.')
        else:
            self._gains.__setattr__(gain_type, value)


class ProportionalController(Controller):
    def apply(self, error_signal: ErrorSignal) -> float:
        return self._gains.proportional * error_signal.proportional


class ProportionalDifferentialController(Controller):
    def apply(self, error_signal: ErrorSignal) -> float:
        return self._gains.proportional * error_signal.proportional + \
               self._gains.differential * error_signal.differential


class PidController(Controller):
    def __init__(self, gains: Gain = Gain()) -> None:
        super().__init__(gains)
        self.integral = 0.0

    def apply(self, error_signal: ErrorSignal) -> float:
        self.integral += self._gains.integral * error_signal.integral
        self.integral = clamp(self.integral, -1, 1)
        p_part = self._gains.proportional * error_signal.proportional
        d_part = self._gains.differential * error_signal.differential
        i_part = self.integral
        return clamp(p_part + d_part + i_part, -1, 1)


class System(AbstractBaseClass):
    def __init__(self, init_position: float, system_noise_std: float,
                 init_velocity: float = None, delta_time: float = None) -> None:
        self.position = init_position
        self.velocity = init_velocity
        self._system_noise_std = system_noise_std
        self._delta_time = delta_time

    @abstractmethod
    def update(self, control: float,) -> None:
        raise NotImplementedError


class LinearSystem(System):
    def update(self, control: float) -> None:
        """The control here is the direct setting of velocity."""
        noise = random.gauss(mu=0.0, sigma=self._system_noise_std)
        delta_position = control * self._delta_time + noise
        self.velocity = delta_position / self._delta_time
        self.position = self.position + delta_position


class MassSystem(System):
    def __init__(self, init_position: float, init_velocity: float, system_noise_std: float,
                 mass: float, delta_time: float) -> None:
        super().__init__(init_position, system_noise_std, init_velocity, delta_time=delta_time)
        self._mass = mass

    def update(self, control: float) -> None:
        """Control stands for force applied to increase or decrease velocity."""
        noise = random.gauss(mu=0.0, sigma=self._system_noise_std)
        # F = m * a -> a = F / m
        # a = dv / dt -> dv = a * dt = (F / m) * dt
        self.velocity += (control / self._mass) * self._delta_time + noise
        self.position += self.velocity * self._delta_time
        # print(f'(pos/vel): -> {(self.position, self.velocity)}')


class Sensor:
    def __init__(self, noise_std: float = 0.1) -> None:
        self._noise_std = noise_std

    def measure(self, state: float) -> float:
        """Simply add some noise to a measured state."""
        noise = random.gauss(mu=0.0, sigma=self._noise_std)
        return state + noise
