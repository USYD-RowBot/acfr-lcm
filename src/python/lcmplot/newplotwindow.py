from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import Qt, pyqtSlot, QItemSelection, QAbstractListModel, QVariant, QItemSelectionModel

from multiplotwindow import Ui_MainWindow


class ListModel(QAbstractListModel):
    def __init__(self, channel_data, *args, **kwargs):
        super(ListModel, self).__init__(*args, **kwargs)
        self.channel_data = channel_data

    def rowCount(self, QModelIndex_parent=None, *args, **kwargs):
        if QModelIndex_parent is None or not QModelIndex_parent.isValid():
            return len(self.channel_data.data_elements)
        else:
            if QModelIndex_parent.column() > 0:
                return 0

            # we have either channel or element data
            parent = QModelIndex_parent.internalPointer()

            if hasattr(parent, 'parent'):
                return 0
            else:
                return len(parent.data_elements)

    def data(self, index, int_role=None):
        # need to be able to numerically index the data
        if not index.isValid():
            return QVariant()

        if int_role == Qt.DisplayRole:
            return QVariant(self.channel_data.data_elements[index.row()].name)
        elif int_role == Qt.UserRole:
            return QVariant(self.channel_data.data_elements[index.row()])


class NewPlotWindow(QMainWindow):
    def __init__(self, data_model):
        super(NewPlotWindow, self).__init__()
        self.data = data_model

        self.ui = Ui_MainWindow()

        self.ui.setupUi(self)

        # load in the data model
        self.ui.sourceView.setModel(self.data)

        # add a legend
        self.ui.plotView.getPlotItem().addLegend()

        # these are the double click/selection commands to add a plot
        self.ui.sourceView.activated.connect(self.timeplot_by_index)
        self.ui.potentialView.activated.connect(self.xy_by_indices)

        self.ui.sourceView.selection_update.connect(self.update_source_selected)

        n = 20
        self.next_pen_idx = 0
        self.pens = [(i, n) for i in xrange(n)]

    @pyqtSlot(QItemSelection)
    def update_source_selected(self, selected):
        for idx in selected.indexes():
            parent = idx.parent()

            if parent.isValid():
                lm = ListModel(self.data.data(parent, Qt.UserRole).value())
                self.ui.potentialView.setModel(lm)

                # auto select the time index
                utime_idx = None
                for ii in xrange(lm.rowCount()):
                    if lm.data(lm.index(ii), Qt.UserRole).value().name.endswith('->utime'):
                        utime_idx = lm.index(ii)
                        break

                if utime_idx is not None:
                    self.ui.potentialView.selectionModel().select(utime_idx, QItemSelectionModel.ClearAndSelect)

    def timeplot_by_index(self, idx):
        if idx.parent().isValid():
            d = self.data.data(idx, Qt.UserRole).value()
            self.plot_data(d.time, d.data, d.name)

    def xy_by_indices(self, idx):
        # the idx refers to our x
        # we still need to get our y (dependent) from sourceView

        x = idx.model().data(idx, Qt.UserRole).value().data
        xname = idx.model().data(idx, Qt.UserRole).value().name

        for idx in self.ui.sourceView.selectedIndexes():
            y = self.data.data(idx, Qt.UserRole).value().data
            yname = self.data.data(idx, Qt.UserRole).value().name

        name = "{} vs {}".format(yname, xname)

        self.plot_data(x, y, name)

    def plot_data(self, x, y, name):
        print name, x, y
        self.ui.plotView.plot(list(x), list(y), pen=self.next_pen(), name=name)

    def next_pen(self):
        self.next_pen_idx += 1
        return self.pens[self.next_pen_idx - 1]