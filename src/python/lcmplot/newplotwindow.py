from PyQt5.QtCore import Qt, QAbstractListModel, QVariant, QItemSelectionModel, QAbstractTableModel, QModelIndex
from PyQt5.QtWidgets import QMainWindow

from .newlcmplotwindow import Ui_MainWindow


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


class PlotData(object):
    def __init__(self, xdata, ydata, plotitem, xlabel, ylabel, legend):
        self.xdata = xdata
        self.ydata = ydata
        self.plotitem = plotitem
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.legend_name = legend


class PlotModel(QAbstractTableModel):
    def __init__(self, *args, **kwargs):
        super(PlotModel, self).__init__(*args, **kwargs)
        self.plots = list()

    def add_plot(self, plot_item):
        row = len(self.plots)
        self.beginInsertRows(QModelIndex(), row, row)
        self.plots.append(plot_item)
        self.endInsertRows()

    def delete_plot(self, row):
        self.beginRemoveRows(QModelIndex(), row, row)
        print row, len(self.plots)
        del self.plots[row]
        self.endRemoveRows()

    def swap_axes(self, row):
        pd = self.plots[row]

        pd.xlabel, pd.ylabel = pd.ylabel, pd.xlabel
        pd.xdata, pd.ydata = pd.ydata, pd.xdata

        if pd.xlabel.endswith('->utime'):
            # this is a vs time
            pd.legend_name = pd.ylabel
        else:
            # this is vs a not-time
            pd.legend_name = "{} vs {}".format(pd.xlabel, pd.ylabel)

        self.dataChanged.emit(self.index(row, 0), self.index(row, 1))


    def rowCount(self, QModelIndex_parent=None, *args, **kwargs):
        return len(self.plots)

    def columnCount(self, QModelIndex_parent=None, *args, **kwargs):
        return 2

    def headerData(self, section, orientation, int_role=None):
        if int_role == Qt.DisplayRole:
            if section == 0:
                return QVariant("X")
            elif section == 1:
                return QVariant("Y")

        return QVariant()

    def data(self, index, int_role=None):
        if not index.isValid():
            return QVariant()

        if int_role == Qt.DisplayRole:
            col = index.column()
            plot = self.plots[index.row()]
            if col == 0:
                return QVariant(plot.xlabel)
            elif col == 1:
                return QVariant(plot.ylabel)
            else:
                return QVariant()

        if int_role == Qt.UserRole:
            plot = self.plots[index.row()]

            return QVariant(plot)


class NewPlotWindow(QMainWindow):
    def __init__(self, data_model):
        super(NewPlotWindow, self).__init__()
        self.data = data_model

        self.plot_model = PlotModel()

        self.ui = Ui_MainWindow()

        self.ui.setupUi(self)

        # load in the data model
        self.ui.sourceView.setModel(self.data)
        self.ui.activeView.setModel(self.plot_model)

        # add a legend
        self.ui.plotView.getPlotItem().addLegend()

        # these are the double click/selection commands to add a plot
        self.ui.sourceView.activated.connect(self.timeplot_by_index)
        self.ui.potentialView.activated.connect(self.xy_by_indices)

        # these look up the currently selected plots and do the action
        self.ui.addplotbutton.clicked.connect(self.add_plot)
        self.ui.deleteplotbutton.clicked.connect(self.delete_plot)
        self.ui.swapaxesbutton.clicked.connect(self.swap_plot)

        self.ui.sourceView.selection_update.connect(self.update_source_selected)

        n = 20
        self.next_pen_idx = 0
        self.pens = [(i, n) for i in xrange(n)]

    def update_source_selected(self, selected):
        for idx in selected:
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
            self.plot_data(d.time, d.data, '->utime', d.name)

    def xy_by_indices(self, idx):
        # the idx refers to our x
        # we still need to get our y (dependent) from sourceView

        x = idx.model().data(idx, Qt.UserRole).value().data
        xname = idx.model().data(idx, Qt.UserRole).value().name

        for idx in self.ui.sourceView.selectedIndexes():
            y = self.data.data(idx, Qt.UserRole).value().data
            yname = self.data.data(idx, Qt.UserRole).value().name

        self.plot_data(x, y, xname, yname)

    def plot_data(self, x, y, xlabel, ylabel):
        if xlabel.endswith('->utime'):
            # this is a vs time
            legend = ylabel
        else:
            # this is vs a not-time
            legend = "{} vs {}".format(xlabel, ylabel)

        plotitem = self.ui.plotView.plot(list(x), list(y), pen=self.next_pen(), name=legend)

        pd = PlotData(x, y, plotitem, xlabel, ylabel, legend)

        self.plot_model.add_plot(pd)

    def next_pen(self):
        self.next_pen_idx += 1
        return self.pens[self.next_pen_idx - 1]

    def add_plot(self):
        idx = self.ui.potentialView.selectedIndexes()[0]
        self.xy_by_indices(idx)

    def delete_plot(self):
        indices = self.ui.activeView.selectedIndexes()
        # the indices list each column in a single row...
        # so to avoid repeat deletions
        rows = set([x.row() for x in indices])
        for row in rows:
            idx = self.plot_model.index(row, 0, QModelIndex())
            pd = self.plot_model.data(idx, Qt.UserRole).value()
            self.ui.plotView.removeItem(pd.plotitem)
            self.ui.plotView.plotItem.legend.removeItem(pd.legend_name)
            self.plot_model.delete_plot(row)

    def swap_plot(self):
        indices = self.ui.activeView.selectedIndexes()
        # the indices list each column in a single row...
        # so to avoid repeat deletions
        rows = set([x.row() for x in indices])
        for row in rows:
            idx = self.plot_model.index(row, 0, QModelIndex())
            pd = self.plot_model.data(idx, Qt.UserRole).value()
            pd.plotitem.setData(x=pd.ydata, y=pd.xdata)
            self.ui.plotView.plotItem.legend.removeItem(pd.legend_name)

            self.plot_model.swap_axes(row)

            self.ui.plotView.plotItem.legend.addItem(pd.plotitem, pd.legend_name)
