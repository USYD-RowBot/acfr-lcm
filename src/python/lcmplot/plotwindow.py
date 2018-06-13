from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import Qt, pyqtSlot

from window import Ui_DataPlotMain
from offsetaxis import OffsetAxis


class PlotWindow(QMainWindow):
    def __init__(self, data_model):
        super(PlotWindow, self).__init__()
        self.data = data_model

        self.ui = Ui_DataPlotMain()

        self.ui.setupUi(self)

        self.ui.dataTree.setModel(self.data)

        self.ui.graphicsView.getPlotItem().addLegend()

        self.ui.pushAdd.clicked.connect(self.graph_addition_pressed)
        self.ui.pushRemove.clicked.connect(self.graph_removal_pressed)

        self.ui.dataTree.activated.connect(self.add_by_index)

        self.ui.graphicsView.dropped_data.connect(self.add_graph)

        self.offset_axis = OffsetAxis('bottom', 0.0)
        self.minimum_time = 1e24  # TODO: clean this up to be float max or something similar

        self.offset_axis.linkToView(self.ui.graphicsView.plotItem.getViewBox())
        self.ui.graphicsView.getPlotItem().layout.addItem(self.offset_axis, 4, 1)

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
            if len(data.time) > 0 and data.time[0] < self.minimum_time:
                self.minimum_time = data.time[0]
                self.offset_axis.update_offset(self.minimum_time)
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

