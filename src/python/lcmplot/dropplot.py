from pyqtgraph import PlotWidget
from PyQt5.QtCore import pyqtSignal


class DropPlotWidget(PlotWidget):
    dropped_data = pyqtSignal(str)

    def __init__(self, *args, **kwargs):
        super(DropPlotWidget, self).__init__(*args, **kwargs)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat("text/plain"):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        if ev.mimeData().hasFormat("text/plain"):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragLeaveEvent(self, ev):
        ev.accept()

    def dropEvent(self, ev):
        ev.acceptProposedAction()

        self.dropped_data.emit(ev.mimeData().text())
