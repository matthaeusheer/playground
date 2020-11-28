from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import random

# Create the main application instance
app = pg.mkQApp()

view = pg.PlotWidget()
view.resize(800, 600)
view.setWindowTitle('Scatter plot using pyqtgraph with PyQT5')
view.setAspectLocked(True)
view.show()

n = 1000
print('Number of points: ' + str(n))
data = np.random.normal(size=(2, n))

# Create the scatter plot and add it to the view
scatter = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='d', size=2)
view.setXRange(-10, 10)
view.setYRange(-10, 10)
view.addItem(scatter)

pos = [{'pos': data[:, i]} for i in range(n)]

now = pg.ptime.time()
scatter.setData(pos)
print("Plot time: {} sec".format(pg.ptime.time() - now))


def update():
    global scatter, data
    data = data = np.random.normal(size=(2, n))
    pos = [{'pos': data[:, i]} for i in range(n)]
    scatter.setData(pos)
    app.processEvents()


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)


if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
