from pid_controller.controller import PidController, System, Sensor, ErrorSignal, MassSystem

from typing import Generator, Tuple


def closed_loop(system: System, controller: PidController, sensor: Sensor,
                desired_state: float, eps: float = 0.01, delta_time: float = 0.1,
                max_time: float = None, max_steps: int = None,
                no_early_stop: bool = True) -> Generator[Tuple[float], None, None]:
    previous_error = 0.0

    step_count = 0
    time = 0.0

    continue_running = True
    while continue_running:
        measurement = sensor.measure(system.position)

        proportional_error = desired_state - measurement
        differential_error = (proportional_error - previous_error) / delta_time if step_count > 0 else 0.0
        integral_error = proportional_error * delta_time

        error_signal = ErrorSignal(proportional_error, differential_error, integral_error)
        controller_output = controller.apply(error_signal)

        previous_error = proportional_error

        yield time, system.position, system.velocity, error_signal, controller.integral, controller_output
        system.update(control=controller_output)

        time += delta_time
        step_count += 1

        # Determine stopping criteria
        should_stop_by_steps = stop_due_to_max_steps_reached(time, step_count, max_time, max_steps)
        if no_early_stop:
            continue_running = not should_stop_by_steps
        else:
            should_stop_by_eps = stop_due_to_eps(system.position, desired_state, eps)
            continue_running = not (should_stop_by_eps or should_stop_by_steps)


def stop_due_to_max_steps_reached(time_step: float, steps: int, max_time: float = None, max_steps: int = None) -> bool:
    if max_steps is not None and steps > max_steps:
        # print(f'Max time steps {max_steps} reached. Stop.')
        return True
    if max_time is not None and time_step > max_time:
        # print(f'Max simulation time {max_time:.2f} reached. Stop.')
        return True
    return False


def stop_due_to_eps(system_state: float, desired_state: float, eps: float) -> bool:
    do_stop = abs(system_state - desired_state) < eps
    if do_stop:
        # print(f'Desired state reached within eps={eps}')
        pass
    return do_stop
