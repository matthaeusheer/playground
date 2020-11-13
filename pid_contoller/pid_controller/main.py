from ipywidgets import interact
import matplotlib.pyplot as plt

from pid_controller.visualization import setup_control_widgets
from pid_controller.controller import LinearSystem, Sensor, MassSystem, PidController, Gain
from pid_controller.loop import closed_loop
from pid_controller.visualization import plot_control_loop_output

PRINT_DEBUG = False


def run_pid_control(init_state, init_velocity, desired_position, system_noise_std, sensor_noise_std, delta_time,
                    p_gain, d_gain, i_gain, mass, eps, max_steps, max_time):
    system = MassSystem(init_state, init_velocity, system_noise_std, mass=mass, delta_time=delta_time)
    # system = LinearSystem(init_state, system_noise_std, delta_time=delta_time)
    sensor = Sensor(noise_std=sensor_noise_std)

    gains = Gain(p_gain, d_gain, i_gain)
    pid_controller = PidController(gains)

    output_generator = closed_loop(system, pid_controller, sensor, desired_position, init_velocity,
                                   eps, delta_time, max_steps, max_time, print_debug=PRINT_DEBUG)

    plot_control_loop_output(output_generator, pid_controller)
    plt.show()
