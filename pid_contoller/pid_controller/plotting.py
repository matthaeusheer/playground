from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt

from pid_controller.controller import Controller, ProportionalDifferentialController

from typing import Generator, List


def plot_control_loop_output(output_generator: Generator, controller: Controller, x_lim: List[int] = None) -> plt.Figure:
    """Plot the output of a closed control loop run."""
    system_states = []
    velocities = []
    error_signals = defaultdict(list)
    controller_outputs = []
    steps = []

    for output in output_generator:
        step, system_state, velocity, error_signal, controller_output = output
        steps.append(step)
        system_states.append(system_state)
        velocities.append(velocity)
        for key, value in error_signal.__dict__.items():
            if value is not None:
                error_signals[key].append(value)
        controller_outputs.append(controller_output)

    fig, ax = setup_plt_figure(figsize=(12, 5), title=f'{type(controller).__name__}',
                               xlabel='Time [s]', ylabel='State and Controls')
    linewidth = 1.5

    ax.plot(steps, system_states, color='green', linewidth=linewidth, alpha=0.8, label='Position')
    ax.plot(steps, velocities, color='cyan', linewidth=linewidth, alpha=0.8, label='Velocity')
    ax.plot(steps, controller_outputs, color='purple', linewidth=linewidth, alpha=0.8, label='Controller Output')
    ax.plot(steps, error_signals['proportional'], color='red', linewidth=linewidth, alpha=0.8, label='Proportional Error')
    if isinstance(controller, ProportionalDifferentialController):
        ax.plot(steps, error_signals['differential'], color='orange', linewidth=linewidth, alpha=0.8, label='Differential Error')
    if x_lim is not None:
        ax.set_xlim(x_lim)
    ax.grid()
    ax.legend()
    return fig


def setup_plt_figure(**kwargs) -> (plt.Figure, plt.Axes):
    """Create a default matplotlib figure and return the figure and ax object.
    Possible kwargs which are handled by this function (matplotlib uses kwargs internally so there is not really
    a way around this):
        figsize: Tuple(float, float), width and height in inches, defaults to (6.4, 4.8)
        title: str, sets the center title for the figures axes
    """
    font = {'family': 'normal',
            'weight': 'bold',
            'size': 12}
    matplotlib.rc('font', **font)
    if 'figsize' in kwargs:
        fig = plt.figure(figsize=kwargs.get('figsize'))
    else:
        fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    if 'title' in kwargs:
        ax.set_title(kwargs.get('title'), fontweight='bold')
    if 'aspect' in kwargs:
        ax.set_aspect(kwargs.get('aspect'))
    else:
        ax.set_aspect('auto')
    if 'axis' in kwargs:
        ax.axis(kwargs.get('axis'))
    if 'xlabel' in kwargs:
        ax.set_xlabel(kwargs.get('xlabel'), fontweight='bold')
    if 'ylabel' in kwargs:
        ax.set_ylabel(kwargs.get('ylabel'), fontweight='bold')
    return fig, ax
