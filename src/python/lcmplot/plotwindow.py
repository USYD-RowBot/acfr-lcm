from PyQt4.QtGui import QMainWindow
from PyQt4.QtCore import Qt, pyqtSlot

from window import Ui_DataPlotMain

class PlotWindow(QMainWindow):
    def __init__(self, data):
        super(PlotWindow, self).__init__()
        self.data = DataModel(data, parent=self)
        self.ui = Ui_DataPlotMain()

        self.ui.setupUi(self)

        self.ui.dataTree.setModel(self.data)

        self.ui.pushAdd.clicked.connect(self.graph_addition_pressed)
        self.ui.pushRemove.clicked.connect(self.graph_removal_pressed)

    @pyqtSlot()
    def graph_addition_pressed(self):
        for idx in self.ui.dataTree.selectedIndexes():
            name = self.data.data(idx, Qt.DisplayRole).toString()
            data = self.data.data(idx, Qt.UserRole).toPyObject()

            print name

            self.ui.graphicsView.plot(data.time, data.data)

    @pyqtSlot()
    def graph_removal_pressed(self):
        print "Button pressed!"
