import numpy as np
import matplotlib.pyplot as plt
import periodicity_detection as pyd
from collections.abc import Callable
from dataclasses import dataclass

@dataclass
class PlotSettings:
    """Settings for plotters.
    
    title (str): The title shown in the plot window.
    x_log_scale (bool): Whether the x axis should use logarithmic scale (True) of linear (False, default).
    y_log_scale (bool): Whether the y axis should use logarithmic scale (True) of linear (False, default).
    """
    title: str
    x_log_scale: bool = False
    y_log_scale: bool = False

class BasePlotter:
    """Base class for plotters. Handles common plot setup and settings.
    
    Args:
        settings: Instance of `PlotSettings`.
    """
    def __init__(self, settings: PlotSettings):
        self._settings = settings
        
    def _plot_setup(self):
        plt.suptitle(self._settings.title)
        plt.xscale("log" if self._settings.x_log_scale else "linear")
        plt.yscale("log" if self._settings.y_log_scale else "linear")
        
        

class FunctionPlotter(BasePlotter):
    """Plotter that plots a function in an interval.
    
    Args:
        settings: Instance of `PlotSettings`.
        func: The periodic function to plot.
        start_x (float): The start of the interval to plot.
        stop_x (float): The end of the interval to plot.
        step (float): The step size of the values plotted.
    """
    def __init__(self, settings: PlotSettings, func: Callable[[float], float], start_x: float, stop_x: float, step: float = 1e-5):
        super().__init__(settings)
        self._func = func
        self._start_x = start_x
        self._stop_x = stop_x
        self._step = step
    
    def plot(self):
        """Generates the plot"""
        self._plot_setup()
        x, y = make_arrays(self._func, self._start_x, self._stop_x, self._step)
        plt.xlabel("x")
        plt.ylabel("func(x)")
        plt.plot(x, y)
        plt.show()
        
class PeriodicFunctionPlotter(FunctionPlotter):
    """Plotter that plots a number of periods of a periodic function. The periodicity of the function is estimated in the interval `start_x` to `stop_x`.
    The plot then starts at `start_x` and ends after the specified number of periods. This class assumes the function is periodic.

    Args:
        settings: Instance of `PlotSettings`.
        periodic_func: The periodic function to plot.
        start_x (float): The start of the graph and the interval which is checked for periodicity.
        stop_x (float): The end of the interval which is checked for periodicity. (The graph will stop after `period` number of periods and will not reach stop_x)
        periods (int): Number of periods to plot.
        step (float): The step size of both the graph and the periodicity estimate.
    """
    
    def __init__(self, settings: PlotSettings, periodic_func: Callable[[float], float], start_x: float, stop_x: float,  periods: int = 2, step: float = 1e-2):
        self.periods = periods
        
        # Make arrays in order to check periodicity. Has potential for optimization since this has to be done again when plotting
        _, y = make_arrays(periodic_func, start_x, stop_x, step)
        
        # Gets estimate of period length in `step` units
        period_length = pyd.estimate_periodicity(y)
        
        # Set new end point after `periods` number of periods
        end = start_x + periods * step * period_length
        
        super().__init__(settings, periodic_func, start_x, end, step)
        
    def plot(self):
        """Generates the plot"""
        plt.title(f"Showing {self.periods} periods")
        super().plot()
        
        
def make_arrays(func: Callable[[float], float], start_x: float, stop_x: float, step: float) -> tuple[np.ndarray, np.ndarray]:
    """Makes a Numpy array of `func` evaluated at every point in the range from `start_x` to `stop_x` with step size `step`.

    Args:
        func: Function to evaluate.
        start_x (float): Start of range.
        stop_x (float): End of range.
        step (float): Step size.

    Returns:
        tuple[np.ndarray, np.ndarray]: The first array is the range. The second array is `func` evaluated at every point in the range.
    """
    x = np.arange(start_x, stop_x, step)
    y = func(x)
    return x, y
        
def h(t: float) -> float:
    """Given function to visualize."""
    return 3 * np.pi * np.exp(-lmbda(t))
    
def lmbda(t: float) -> float:
    """Part of given function to visualize."""
    return 5 * np.sin(2*np.pi*t)

def main():
    """Generate a plot of 2 periods of `h`. When it is closed a plot of """
    plot = PeriodicFunctionPlotter(PlotSettings("Plot of h(x)"), h, 0, 1000, 2)
    plot.plot()
    
if __name__ == "__main__":
    main()