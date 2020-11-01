from abc import ABC as AbstractBaseClass
from abc import abstractmethod
from dataclasses import dataclass

import random


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
    proportional: float = None
    differential: float = None
    integral: float = None


class Controller:
    @staticmethod
    @abstractmethod
    def apply(error_signal: ErrorSignal, gains: Gain = None) -> float:
        raise NotImplementedError


class ProportionalController:
    def __init__(self, p_gain: float = 0.1) -> None:
        self._p_gain = p_gain

    def apply(self, error_signal: ErrorSignal) -> float:
        return self._p_gain * error_signal.proportional


class ProportionalDifferentialController:
    def __init__(self, p_gain: float = 0.1, d_gain: float = 0.1) -> None:
        self._p_gain = p_gain
        self._d_gain = d_gain

    def apply(self, error_signal: ErrorSignal) -> float:
        return self._p_gain * error_signal.proportional + self._d_gain * error_signal.differential


class PidController:
    def __init__(self, p_gain: float, d_gain: float, i_gain: float) -> None:
        self._p_gain = p_gain
        self._d_gain = d_gain
        self._i_gain = i_gain
        self.integral = 0.0

    def apply(self, error_signal: ErrorSignal) -> float:
        self.integral += self._i_gain * error_signal.integral
        p_part = clip(self._p_gain * error_signal.proportional, -1, 1)
        d_part = clip(self._d_gain * error_signal.differential, -1, 1)
        i_part = clip(self.integral, -1, 1)
        return p_part + d_part + i_part


def clip(value, lower, upper):
    return lower if value < lower else upper if value > upper else value


class System(AbstractBaseClass):
    def __init__(self, init_state: float, system_noise_std: float, init_velocity: float = None) -> None:
        self.state = init_state
        self._velocity = init_velocity
        self._system_noise_std = system_noise_std

    @abstractmethod
    def update(self, control: float) -> None:
        raise NotImplementedError

    def update_velocity(self, velocity: float) -> None:
        self._velocity = velocity


class LinearSystem(System):
    def update(self, control: float) -> None:
        noise = random.gauss(mu=0.0, sigma=self._system_noise_std)
        self.state = self.state + control + noise


class MassSystem(System):
    def __init__(self, init_state: float, init_velocity: float, system_noise_std: float, delta_time: float) -> None:
        super().__init__(init_state, system_noise_std, init_velocity)
        self.delta_time = delta_time

    def update(self, control: float) -> None:
        noise = random.gauss(mu=0.0, sigma=self._system_noise_std)
        self.state += self._velocity * self.delta_time + control + noise


class Sensor:
    def __init__(self, noise_std: float = 0.1) -> None:
        self._noise_std = noise_std

    def measure(self, state: float) -> float:
        noise = random.gauss(mu=0.0, sigma=self._noise_std)
        return state + noise
