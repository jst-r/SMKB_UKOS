from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import random


class CommPlotter:
    def __init__(self):
        plt.ion()
        self.fig, (self.ax_score, self.ax_signal) = plt.subplots(2)
        self.ax_score.set_autoscale_on(True)
        self.ax_signal.set_autoscale_on(True)
        self.line_score, = self.ax_score.plot([], [], '-o')
        self.line_signal, = self.ax_signal.plot([], [], '-o')
        plt.show()

    
    def update_score(self, x, y):
        self.line_score.set_xdata(np.append(self.line_score.get_xdata(), x))
        self.line_score.set_ydata(np.append(self.line_score.get_ydata(), y))
        
        self.ax_score.relim()
        self.ax_score.autoscale_view()

        plt.pause(.01)
        plt.draw()

    def update_signal(self, x, y):
        self.line_signal.set_xdata(np.append(self.line_signal.get_xdata(), x))
        self.line_signal.set_ydata(np.append(self.line_signal.get_ydata(), y))
        
        self.ax_signal.relim()
        self.ax_signal.autoscale_view()

        plt.pause(.01)
        plt.draw()


p = CommPlotter()
x1 = 0
y1 = 0
x2 = 0
y2 = 0

for i in range(1000):
    p.update_score([x1], [y1])
    x1 += random.random() * 2 - 1
    y1 += random.random() * 2 - 1

    p.update_signal([x2], [y2])
    x2 += random.random() * 2 - 1
    y2 += random.random() * 2 - 1
