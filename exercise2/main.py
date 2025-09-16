import numpy as np
import matplotlib.pyplot as plt
import periodicity_detection as pyd
from collections.abc import Callable

class BasePlotter:
    """Base class for plotters. Handles common plot setup and settings
    
    Args:
        title: The title of the plot window.
    """
    def __init__(self, title: str):
        self.title = title
        
    def _plot_setup(self):
        plt.suptitle(self.title)

class FunctionPlotter(BasePlotter):
    """Plotter that plots a function in an interval.
    
    Args:
        title (str): Title of plot window.
        func: The periodic function to plot.
        start_x (float): The start of the interval to plot.
        stop_x (float): The end of the interval to plot.
        step (float): The step size of the values plotted.
    """
    def __init__(self, title: str, func: Callable[[float], float], start_x: float, stop_x: float, step: float = 1e-5):
        super().__init__(title)
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
    """Plotter that plots a number of periods of a periodic function. The periodicity of the function is estimated.

    Args:
        periodic_func: The periodic function to plot.
        start_x (float): The function value the graph starts at. This is also the start of the interval which is checked for periodicity.
        stop_x (float): The end of the interval which is checked for periodicity. (The graph will stop after a number of periods and will not reach stop_x)
        periods (int): Number of periods to plot.
        step (float): The step size of both the graph and the periodicity estimate.
    """
    
    def __init__(self, periodic_func: Callable[[float], float], start_x: float, stop_x: float,  periods: int = 2, step: float = 1e-2):
        # Make arrays in order to check periodicity. Has potential for optimization since this has to be done again when plotting
        _, y = make_arrays(periodic_func, start_x, stop_x, step)
        
        # Gets estimate of period length in `step` units
        period_length = pyd.estimate_periodicity(y)
        
        # Set new end point after `periods` number of periods
        end = start_x + periods * step * period_length
        
        super().__init__(f"Showing {periods} periods", periodic_func, start_x, end, step)
        
        
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
    """Generate a plot of 2 periods of `h`"""
    plot = PeriodicFunctionPlotter(h, 0, 1000, 2)
    plot.plot()
    
if __name__ == "__main__":
    main()