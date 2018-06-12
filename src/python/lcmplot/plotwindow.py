from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import Qt, pyqtSlot

from window import Ui_DataPlotMain
from datamodel import DataModel


class PlotWindow(QMainWindow):
    def __init__(self, data):
        super(PlotWindow, self).__init__()
        self.data = DataModel(parent=self)

        self.data.insert_data(data)
        self.ui = Ui_DataPlotMain()

        self.ui.setupUi(self)

        self.ui.dataTree.setModel(self.data)

        self.ui.graphicsView.getPlotItem().addLegend()

        self.ui.pushAdd.clicked.connect(self.graph_addition_pressed)
        self.ui.pushRemove.clicked.connect(self.graph_removal_pressed)

        self.ui.dataTree.activated.connect(self.add_by_index)

        self.ui.graphicsView.dropped_data.connect(self.add_graph)

        n = 20
        self.next_pen_idx = 0
        self.pens = [(i, n) for i in xrange(n)]

        self.plots = dict()


    @pyqtSlot()
    def graph_addition_pressed(self):
        for idx in self.ui.dataTree.selectedIndexes():
            print idx
            self.plot_data(self.data.data(idx, Qt.UserRole).value())

    def next_pen(self):
        self.next_pen_idx += 1
        return self.pens[self.next_pen_idx - 1]

    def plot_data(self, data):
        if data.name not in self.plots:
            dataitem = self.ui.graphicsView.plot(data.time, data.data, pen=self.next_pen(), name=data.name)
            self.plots[data.name] = dataitem

    @pyqtSlot()
    def graph_removal_pressed(self):
        for idx in self.ui.dataTree.selectedIndexes():
            data = self.data.data(idx, Qt.UserRole).value()

            if data.name in self.plots:
                # remove the line plot
                self.ui.graphicsView.removeItem(self.plots[data.name])
                # remove the legend entry separately
                self.ui.graphicsView.plotItem.legend.removeItem(data.name)
                # remove the entry in the local plot
                del self.plots[data.name]


    def add_by_index(self, idx):
        if idx.parent().isValid():
            self.plot_data(self.data.data(idx, Qt.UserRole).value())

    def add_graph(self, data_name):
        entry = self.data.get_by_name(data_name)
        if entry is not None:
            self.plot_data(entry)
        else:
            print "Unknown data: {}".format(data_name)

