from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import Qt, pyqtSlot

from window import Ui_DataPlotMain
from datamodel import DataModel


class PlotWindow(QMainWindow):
    def __init__(self, data):
        super(PlotWindow, self).__init__()
        self.data = DataModel(data, parent=self)
        self.ui = Ui_DataPlotMain()

        self.ui.setupUi(self)

        self.ui.dataTree.setModel(self.data)

        self.ui.graphicsView.getPlotItem().addLegend()

        self.ui.pushAdd.clicked.connect(self.graph_addition_pressed)
        self.ui.pushRemove.clicked.connect(self.graph_removal_pressed)

        self.ui.graphicsView.dropped_data.connect(self.add_graph)

        n = 20
        self.next_pen_idx = 0
        self.pens = [(i, n) for i in xrange(n)]


    @pyqtSlot()
    def graph_addition_pressed(self):
        for idx in self.ui.dataTree.selectedIndexes():
            self.plot_data(self.data.data(idx, Qt.UserRole).value())

    def next_pen(self):
        self.next_pen_idx += 1
        return self.pens[self.next_pen_idx - 1]

    def plot_data(self, data):
        self.ui.graphicsView.plot(data.time, data.data, pen=self.next_pen(), name=data.name)

    @pyqtSlot()
    def graph_removal_pressed(self):
        print "Button pressed!"

    def add_graph(self, data_name):
        entry = self.data.get_by_name(data_name)
        if entry is not None:
            self.plot_data(entry)
