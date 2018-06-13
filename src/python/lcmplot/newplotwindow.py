from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import Qt, pyqtSlot

from multiplotwindow import Ui_MainWindow
from offsetaxis import OffsetAxis


class NewPlotWindow(QMainWindow):
    def __init__(self, data_model):
        super(NewPlotWindow, self).__init__()
        self.data = data_model

        self.ui = Ui_MainWindow()

        self.ui.setupUi(self)

        self.ui.sourceView.setModel(self.data)
        self.ui.plotView.getPlotItem().addLegend()

        self.ui.sourceView.activated.connect(self.timeplot_by_index)
        self.ui.sourceView.sel
        #self.ui.plotView.dropped_data.connect(self.graph_data_drop)

