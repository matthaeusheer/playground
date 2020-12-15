from PyQt5 import QtGui
from PyQt5 import QtCore
import pyqtgraph as pg


class RectangleItem(pg.GraphicsObject):
    def __init__(self, topLeft, size):
        pg.GraphicsObject.__init__(self)
        self.topLeft = topLeft
        self.size = size
        self.generatePicture()

    def generatePicture(self):
        self.picture = QtGui.QPicture()
        qp = QtGui.QPainter(self.picture)
        qp.setPen(QtCore.Qt.NoPen)
        qp.setBrush(QtGui.QColor(200, 0, 0))
        qp.drawRect(*self.topLeft, *self.size)
        qp.end()

    def paint(self, p, *args):
        p.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())