import numpy as np
import matplotlib.pyplot as plt

import re

import serial


class CommPlotter:
    def __init__(self):
        self.n_points = 1000
        plt.ion()
        self.fig, self.ax_score = plt.subplots(1)
        self.ax_score.set_autoscale_on(True)
        self.line_score, = self.ax_score.plot([], [], '-')
        plt.show()
        plt.draw()

    
    def update_score(self, x, y):
        self.line_score.set_xdata(np.append(self.line_score.get_xdata(), x)[-self.n_points:])
        self.line_score.set_ydata(np.append(self.line_score.get_ydata(), y)[-self.n_points:])
        
        self.ax_score.relim()
        self.ax_score.autoscale_view()

        plt.pause(.01)
        plt.draw()


p = CommPlotter()

ser = serial.Serial('COM5', 115200, stopbits=serial.STOPBITS_ONE)

t = 0
i = 0

with open('temp.txt', 'w') as f:
    while True:
        ind = []
        score = []
        for j in range(50):
            line = ser.readline().decode('utf-8')
            print(line, i)
            score.append(int(line))
            i += 1
            ind.append(i)
        p.update_score(ind, score)
        
