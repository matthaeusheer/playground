from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt
import ipywidgets as widgets
from ipywidgets import Layout

from typing import Generator, List


def setup_control_widgets() -> dict:
    layout = Layout(width='50%', description_width='initial', align_content='center')
    style = {'description_width': '30%'}

    widget_sliders = {
        'init_state': widgets.FloatSlider(value=1.0, min=-10, max=10, step=0.1,
                                          description='Initial Position',
                                          layout=layout, style=style),
        'init_velocity': widgets.FloatSlider(value=0.0, min=-10, max=10, step=0.1,
                                             description='Initial Velocity',
                                             layout=layout, style=style),
        'desired_position': widgets.FloatSlider(value=5.0, min=-10, max=10, step=0.1,
                                                description='Desired Position',
                                                layout=layout, style=style),
        'system_noise_std': widgets.FloatSlider(value=0.0, min=0.0, max=0.05, step=0.005,
                                                readout_format='.3f', description='System Noise',
                                                layout=layout, style=style),
        'sensor_noise_std': widgets.FloatSlider(value=0.0, min=0.0, max=0.05, step=0.005,
                                                readout_format='.3f', description='Sensor Noise',
                                                layout=layout, style=style),
        'delta_time': widgets.FloatSlider(value=1.0, min=0.001, max=5.0, step=0.01,
                                          description='Time Step', layout=layout, style=style),
        'p_gain': widgets.FloatSlider(value=0.0, min=0.0, max=3.0, step=0.01,
                                      description='P Gain',
                                      layout=layout, style=style),
        'd_gain': widgets.FloatSlider(value=0.0, min=0.0, max=3.0, step=0.01,
                                      description='D Gain', layout=layout, style=style),
        'i_gain': widgets.FloatSlider(value=0.0, min=0.0, max=3.0, step=0.01,
                                      description='I Gain', layout=layout, style=style),
        'mass': widgets.FloatSlider(value=1.0, min=0.01, max=19.0, step=0.01,
                                    description='Mass', layout=layout, style=style),
        'eps': widgets.FloatSlider(value=0.001, min=0.0001, max=0.1, step=0.001,
                                   readout_format='.4f', description='eps', layout=layout, style=style),
        'max_steps': widgets.FloatSlider(value=200, min=1, max=1000, step=10,
                                         description='Max Steps', layout=layout, style=style),
        'max_time': widgets.FloatSlider(value=200, min=1, max=1000, step=10,
                                        description='Max Time', layout=layout, style=style),
    }
    return widget_sliders


def plot_control_loop_output(output_generator: Generator, x_lim: List[int] = None, plot_errors: bool = True) -> plt.Figure:
    """Plot the output of a closed control loop run."""

    # TODO: Make this an actual animation! :-)

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

    fig, ax = setup_plt_figure(figsize=(25, 6), xlabel='Time [s]', ylabel='State and Controls')
    linewidth = 1.5

    ax.plot(steps, system_states, color='green', linewidth=linewidth, alpha=0.8, label='Position')
    ax.plot(steps, velocities, color='cyan', linewidth=linewidth, alpha=0.8, label='Velocity')
    ax.plot(steps, controller_outputs, color='purple', linewidth=linewidth, alpha=0.8, label='Controller Output')
    if plot_errors:
        ax.plot(steps, error_signals['proportional'], color='red', linewidth=linewidth, alpha=0.8, label='Proportional Error')
        ax.plot(steps, error_signals['differential'], color='orange', linewidth=linewidth, alpha=0.8, label='Differential Error')
        ax.plot(steps, error_signals['integral'], color='magenta', linewidth=linewidth, alpha=0.8, label='Integral Error')

    if x_lim is not None:
        ax.set_xlim(x_lim)
    ax.grid()
    ax.legend(loc='upper right')
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
