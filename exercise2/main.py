import numpy as np
import matplotlib.pyplot as plt
import math
from collections.abc import Callable

def h(t: float) -> float:
    return 3 * math.pi * math.exp(-lmbda(t))
    
def lmbda(t: float) -> float:
    return 5 * math.sin(2*math.pi*t)

class PlotSettings:
    def __init__(self, step_size=1):
        self.step_size
        
class Plotter:
    def __init__(self, x: np.ndarray, y: np.ndarray, settings: PlotSettings):
        self.settings = settings
        self.x = x
        self.y = y
        
    def plot(self):
        plt.plot(self.x, self.y)

class FunctionPlotter:
    def __init__(self, func: Callable[[float], float], settings: PlotSettings):
        self.func = func
        self.settings = settings
        
    def plot():
        plt.plot()
        
class PeriodicFunctionPlotter(FunctionPlotter):
    def plot():
        pass
    
    def get_period(self):
        
        
        for x in range():
            pass
        

def main():
    np.fft.fft()
    settings = PlotSettings()
    plotter = Plotter(np.array([1,2]), np.array([1,2]))
    plotter.plot()
    
    
if __name__ == '__main__':
    main()