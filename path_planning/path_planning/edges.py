import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import numpy as np

app = pg.mkQApp()

win = pg.GraphicsLayoutWidget(show=True)

p1 = win.addPlot()
data1 = np.random.normal(size=300)
connected = np.round(np.random.rand(300))
curve1 = p1.plot(data1, connect=connected)


def update1():
    global data1, connected
    data1[:-1] = data1[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    connected = np.roll(connected, -1)
    data1[-1] = np.random.normal()
    curve1.setData(data1, connect=connected)


timer = pg.QtCore.QTimer()
timer.timeout.connect(update1)
timer.start(50)

app.exec_()