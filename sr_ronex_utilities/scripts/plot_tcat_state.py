#!/usr/bin/env python

# ####################################################################
# Copyright (c) 2013, Shadow Robot Company, All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library.
# ####################################################################

import rospy
from sr_ronex_msgs.msg import TCATState
#from PyQt4.QtCore import *
from PyQt4.QtGui import QMainWindow, QApplication, QWidget, QVBoxLayout

import numpy
from std_msgs.msg import Int8
import sys

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as  NavigationToolbar
from matplotlib.figure import Figure

import ctypes
from math import sqrt

class PlotWindow(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('Impulse Responses')
        self.create_main_frame()

        self.setMinimumSize(1600,900)

        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        for ax in self.axes:
            ax.clear()
            ax.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes = []
        for i in range(1,5):
            self.axes.append(self.fig.add_subplot(4,1,i))
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)



class TCATPlot(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=20

    rospy.init_node('visualizer', anonymous=True)
    self.subscriber = rospy.Subscriber("/ronex/tcat/1394812611/state", TCATState, self.plotResults, queue_size = 1 )

  def plotResults(self, data):
    receiver_index = 0

    print
    base=[r.timestamp_ns - (r.FPI - r.first_sample_number) for r in data.received_data]
    base_min = min(base)
    print base, base_min

    base = [b - base_min for b in base]
    print base

    for receiver, ax in zip(data.received_data, self.axes):
        ax.clear()
        ax.set_xlim([0, 64])
        #ax.set_ylim([-32768,32768])
        y1 = []
        y2 = []
        y3 = []
        x  = []
        x_value = base[receiver_index]

        for impulse in receiver.impulse_response:
            y1.append(impulse.real)
            y2.append(impulse.imaginary)
            y3.append(sqrt(impulse.real**2 + impulse.imaginary**2))
            x.append(x_value)
            x_value = x_value + 1

        #x = range(receiver*10, len(y1)+(receiver*10))
        #x = range(0, len(y1))
        ax.set_ylim([min(min(y2),min(y1)), max(y3)])
        self.line1 = ax.plot(x, y1, c='#A0A0FF', label="real")
        self.line1 = ax.plot(x, y2, c='#A0FFA0', label="imag")
        self.line1 = ax.plot(x, y3, c='#A00000', label="magn")
        ax.legend(loc=0, scatterpoints = 1)
        #ax.set_title("Impulse Response["+str(receiver_index)+"]")
        receiver_index += 1

    self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TCATPlot()
    window.show()
    app.exec_()
