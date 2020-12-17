from time import sleep
import numpy
import matplotlib.pyplot as plt
import random


class CommPlotter:
    def __init__(self, x=[], y=[]):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_autoscale_on(True)
        self.line, = self.ax.plot(x, y, '-o', alpha=.3)
        plt.show()

    
    def update(self, x, y):
        self.line.set_xdata(numpy.append(self.line.get_xdata(), x))
        self.line.set_ydata(numpy.append(self.line.get_ydata(), y))
        
        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(.01)
        plt.draw()


p = CommPlotter()
x = 0
y = 0

for i in range(1000):
    p.update([x], [y])
    x += random.random() * 2 - 1
    y += random.random() * 2 - 1
