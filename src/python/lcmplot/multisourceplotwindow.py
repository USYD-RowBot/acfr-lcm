from PyQt5.QtCore import Qt, QAbstractListModel, QVariant, QItemSelectionModel, QAbstractTableModel, QModelIndex, QTimer
from PyQt5.QtWidgets import QMainWindow, QFileDialog

from .multiplotwindow import Ui_MainWindow

from .multisourcemodel import NumericElementData, TypeData, DataModel, MessageSource

from .plotdialogwindow import PlotDialogWindow

import ConfigParser

import os


def get_leaf_nodes(model_element):
    leaves = []

    for node in model_element.entries:
        if isinstance(node, NumericElementData):
            leaves.append((node.full_name(), node))
        else:
            leaves.extend(get_leaf_nodes(node))

    return leaves


class ListModel(QAbstractListModel):
    def __init__(self, data_descriptor, *args, **kwargs):
        super(ListModel, self).__init__(*args, **kwargs)
        # the descriptor is a little vague here - it may be a sub-entry
        # not the actual channel itself
        found = False
        while not found:
            while not hasattr(data_descriptor, 'data_type'):
                data_descriptor = data_descriptor.parent()

            if data_descriptor.data_type is None:
                data_descriptor = data_descriptor.parent()
                continue

            found = True

        # now we know we have the channel
        channel_data = data_descriptor

        self.channels = get_leaf_nodes(channel_data)

    def rowCount(self, QModelIndex_parent=None, *args, **kwargs):
        if QModelIndex_parent is None or not QModelIndex_parent.isValid():
            return len(self.channels)
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
            return QVariant(self.channels[index.row()][0])
        elif int_role == Qt.UserRole:
            return QVariant(self.channels[index.row()][1])


class PlotData(object):
    def __init__(self, xdata, ydata, plotitem, xlabel, ylabel, legend):
        self.xdata = xdata
        self.ydata = ydata
        # note that to 'update' this item a call to setData is all that is needed
        # there isn't something special - although memory management for the underlying
        # array/numpy array is recommended.
        self.plotitem = plotitem

        # we want the channel name to avoid repeating it
        chx = xlabel.split('->')[0]
        chy = ylabel.split('->')[0]
        self.channel = chx or chy  # this is a little cheeky
        self.xlabel = xlabel.split('->')[1]
        self.ylabel = ylabel.split('->')[1]

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

        self.dataChanged.emit(self.index(row, 1), self.index(row, 2))

    def rowCount(self, QModelIndex_parent=None, *args, **kwargs):
        return len(self.plots)

    def columnCount(self, QModelIndex_parent=None, *args, **kwargs):
        return 3

    def headerData(self, section, orientation, int_role=None):
        if int_role == Qt.DisplayRole:
            if section == 1:
                return QVariant("X")
            elif section == 2:
                return QVariant("Y")
            elif section == 0:
                return QVariant("Channel")

        return QVariant()

    def data(self, index, int_role=None):
        if not index.isValid():
            return QVariant()

        if int_role == Qt.DisplayRole:
            col = index.column()
            plot = self.plots[index.row()]
            if col == 1:
                return QVariant(plot.xlabel)
            elif col == 2:
                return QVariant(plot.ylabel)
            elif col == 0:
                return QVariant(plot.channel)
            else:
                return QVariant()

        if int_role == Qt.UserRole:
            plot = self.plots[index.row()]

            return QVariant(plot)


class MultiSourcePlotWindow(QMainWindow):
    def __init__(self, data_model):
        super(MultiSourcePlotWindow, self).__init__()
        self.data = data_model

        self.timer = QTimer()
        self.timer.setInterval(100)  # 10Hz
        self.timer.start(100)  # start in a second

        self.timer.timeout.connect(self.tick)

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

        self.ui.actionOpen.triggered.connect(self.fileopen)

        self.ui.actionSave_Current.triggered.connect(self.save_current)

        self.ui.actionLoad_Saved.triggered.connect(self.load_saved)

        self.ui.activeView.doubleClicked.connect(self.update_plot)

        n = 20
        self.next_pen_idx = 0
        self.pens = [(i, n) for i in xrange(n)]

    def tick(self):
        # for each plot, check if we need to update
        # or more accurately just update
        for plot in self.plot_model.plots:
            plot.plotitem.setData(plot.xdata, plot.ydata)

    def save_current(self):
        # open dialog box to save location
        config = ConfigParser.ConfigParser()

        path = os.path.join(os.path.expanduser('~'), 'git', 'acfr-lcm', 'src', 'python', 'config')

        file_name = QFileDialog.getSaveFileName(self, 'Save File', directory = path, filter = "Configs (*.cfg)")

        if file_name is None:
            print "No file selected."
        name = file_name[0]

        if name.endswith('.cfg'):
            name = name[:-4]

        config.add_section(name)
        rows = self.plot_model.rowCount()
        for row in range(rows):
            idx = self.plot_model.index(row, 0, QModelIndex())
            pd = self.plot_model.data(idx, Qt.UserRole).value()
            config.set(name, 'channel' + str(row), pd.channel)
            config.set(name, 'X label' + str(row), pd.xlabel)
            config.set(name, 'Y label' + str(row), pd.ylabel)

        with open(name+'.cfg', 'wb') as configfile:
            config.write(configfile)

    def load_saved(self):
        # display the dialog, to locate config file
        path = os.path.join(os.path.expanduser('~'), 'git', 'acfr-lcm', 'src', 'python', 'config')

        file_name = QFileDialog.getOpenFileName(self, "Open Config File", directory = path, filter = "Configs (*.cfg)")

        #check if file was selected
        if file_name[0] == "":
            print "No file selected."
            return
        name = file_name[0]
        print name
        #read file
        config = ConfigParser.ConfigParser()
        config.read(name)
        if name.endswith('.cfg'):
            name = name[:-4]
        #delete existing plots
        rows = self.plot_model.rowCount()
        for row in range(rows):
            idx = self.plot_model.index(0, 0, QModelIndex())
            pd = self.plot_model.data(idx, Qt.UserRole).value()
            self.ui.plotView.removeItem(pd.plotitem)
            self.ui.plotView.plotItem.legend.removeItem(pd.legend_name)
            self.plot_model.delete_plot(0)
        #Reset number of open plots
        self.next_pen_idx = 0
        #Read how many items are in the file then construct the message name and look for the corresponding node for plotting
        if not config.has_section(name):
            print "Section not found in configuration file. Does the section name match the path name?"
            return
        rows = len(config.items(name))
        for row in range(0, rows, 3):
            xname =  config.get(name, 'channel' + str(row/3)) + '->' + config.get(name, 'X label' + str(row/3))
            yname =  config.get(name, 'channel' + str(row/3)) + '->' + config.get(name, 'Y label' + str(row/3))
            model_element = self.ui.sourceView.model().sources[0]
            leaves = get_leaf_nodes(model_element)
            for i in range(len(leaves)):
                if leaves[i][0] == yname:
                    node = leaves[i][1]
                    y = node.data
                if leaves[i][0] == xname:
                    node = leaves[i][1]
                    x = node.data
            if ('y' in locals() and 'x' in locals()):
                self.plot_data(x, y, xname, yname)
            else:
                print "In config file " + xname + " or " + yname + "not found in LCM list."



    def fileopen(self):
        # display the dialog, pass the (valid) result to the data model
        file_name = QFileDialog.getOpenFileName(self, "Open LCM log", filter="LCM Logs (*.lcm)")

        if file_name is None:
            print "No file selected."
            return

        print file_name
        self.data.add_file(file_name[0])

    def update_source_selected(self, selected):
        for idx in selected:
            if idx.isValid():
                node = self.data.data(idx, Qt.UserRole).value()
                if isinstance(node, NumericElementData):
                    # we can plot this... so we can get a matching list
                    lm = ListModel(node)
                    self.ui.potentialView.setModel(lm)

                    # auto select the time index
                    utime_idx = None
                    for ii in xrange(lm.rowCount()):
                        if lm.data(lm.index(ii), Qt.DisplayRole).value().endswith('->utime'):
                            utime_idx = lm.index(ii)
                            break

                    if utime_idx is not None:
                        self.ui.potentialView.selectionModel().select(utime_idx, QItemSelectionModel.ClearAndSelect)

                    return

        self.ui.potentialView.setModel(None)

    def timeplot_by_index(self, idx):
        node = self.data.data(idx, Qt.UserRole).value()
        if isinstance(node, NumericElementData):
            # we need to find the matching utime (or other known reference)
            lm = self.ui.potentialView.model()
            utime_idx = None
            for ii in xrange(lm.rowCount()):
                if lm.data(lm.index(ii), Qt.DisplayRole).value().endswith('->utime'):
                    utime_idx = lm.index(ii)
                    break

            if utime_idx is not None:
                utime = lm.data(utime_idx, Qt.UserRole).value()
                self.plot_data(utime.data, node.data, '->utime', node.full_name())

    def xy_by_indices(self, idx):
        # the idx refers to our x
        # we still need to get our y (dependent) from sourceView

        x = idx.model().data(idx, Qt.UserRole).value().data
        xname = idx.model().data(idx, Qt.UserRole).value().full_name()

        for idx in self.ui.sourceView.selectedIndexes():
            y = self.data.data(idx, Qt.UserRole).value().data
            yname = self.data.data(idx, Qt.UserRole).value().full_name()

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

    def update_plot(self, idx):
        plot_data = self.plot_model.data(idx, Qt.UserRole)
        pdw = PlotDialogWindow(self, plot_data.value().plotitem)
        pdw.show()

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
