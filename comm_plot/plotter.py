import numpy as np
import matplotlib.pyplot as plt

import re

import serial


class CommPlotter:
    def __init__(self):
        self.n_points = 20
        plt.ion()
        self.fig, (self.ax_score, self.ax_signal) = plt.subplots(2)
        self.ax_score.set_autoscale_on(True)
        self.ax_signal.set_autoscale_on(True)
        self.line_score, = self.ax_score.plot([], [], '-')
        self.line_signal, = self.ax_signal.plot([], [], '-')
        plt.show()
        plt.draw()

    
    def update_score(self, x, y):
        self.line_score.set_xdata(np.append(self.line_score.get_xdata(), x)[-self.n_points:])
        self.line_score.set_ydata(np.append(self.line_score.get_ydata(), y)[-self.n_points:])
        
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

ser = serial.Serial('COM7', 115200, stopbits=serial.STOPBITS_ONE)

t = 0
i = 0

while True:
    line = ser.readline().decode('utf-8')
    print(line)
    if line.startswith('resetLength = '):
        rsl = int(re.search(r'(\-{0,1}[0-9]+)', line)[0])
        t += rsl
        p.update_signal([t, t], [0, 1])
    elif line.startswith('setLength = '):
        sl = int(re.search(r'(\-{0,1}[0-9]+)', line)[0])
        t += sl
        p.update_signal([t, t], [1, 0])
    elif line.startswith('score = '):
        score = int(re.search(r'(\-{0,1}[0-9]+)', line)[0])
        i += 1
        p.update_score([i], [score])
    
