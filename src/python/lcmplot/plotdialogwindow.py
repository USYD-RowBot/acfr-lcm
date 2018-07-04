from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPen, QBrush
from PyQt5.QtWidgets import QDialog, QColorDialog

from .plotdialog import Ui_PlotDialog

from pyqtgraph import PlotDataItem, mkPen, mkBrush


class PlotDialogWindow(QDialog):
    def __init__(self, parent, plot_data):
        super(PlotDialogWindow, self).__init__(parent)

        self.ui = Ui_PlotDialog()
        self.ui.setupUi(self)

        # load in the valid line types
        self.ui.line_style.addItem("None", Qt.NoPen)
        self.ui.line_style.addItem("Solid", Qt.SolidLine)
        self.ui.line_style.addItem("Dash", Qt.DashLine)
        self.ui.line_style.addItem("Dot", Qt.DotLine)
        self.ui.line_style.addItem("Dash Dot", Qt.DashDotLine)
        self.ui.line_style.addItem("Dash Dot Dot", Qt.DashDotDotLine)

        # load in the valid symbol types
        self.ui.point_style.addItem("None", None)
        self.ui.point_style.addItem("Circle", "o")
        self.ui.point_style.addItem("Triangle", "t")
        self.ui.point_style.addItem("Square", "s")
        self.ui.point_style.addItem("Pentagon", "p")
        self.ui.point_style.addItem("Hexagon", "h")
        self.ui.point_style.addItem("Plus (+)", "+")
        self.ui.point_style.addItem("Cross (x)", "x")
        self.ui.point_style.addItem("Star", "star")

        # the PlotDataItem object we are editing the plot parameters for
        self.plot_data = plot_data

        # get the parameters of the existing plot
        # safest way is to make a pen using the parameters
        # the actual pen attribute may just be a colour tuple
        pen = mkPen(self.plot_data.opts['pen'])
        self.line_colour = pen.color()
        self.line_width = pen.widthF()
        self.line_style = pen.style()

        # same deal for outlines of the symbols
        outline = mkPen(self.plot_data.opts['symbolPen'])
        self.point_outline_colour = outline.color()
        self.point_outline_width = outline.widthF()

        # and now the symbol filling
        fill = mkBrush(self.plot_data.opts['symbolBrush'])
        self.point_fill_colour = fill.color()

        # finally the size and what symbol (if any!) to plot
        self.point_size = self.plot_data.opts['symbolSize']
        self.point_style = self.plot_data.opts['symbol']

        # now transfer details/parameters of the existing plot to the
        # elements in the dialog
        self.ui.point_fill_colour.setStyleSheet("background-color: " + self.point_fill_colour.name())
        self.ui.point_outline_colour.setStyleSheet("background-color: " + self.point_outline_colour.name())
        self.ui.line_colour.setStyleSheet("background-color: " + self.line_colour.name())

        idx = self.ui.line_style.findData(self.line_style)
        self.ui.line_style.setCurrentIndex(idx)

        idx = self.ui.point_style.findData(self.point_style)
        self.ui.point_style.setCurrentIndex(idx)

        self.ui.point_size.setValue(self.point_size)
        self.ui.line_width.setValue(self.line_width)
        self.ui.point_outline_width.setValue(self.point_outline_width)

        if self.line_style == Qt.NoPen:
            self.ui.line_width.setEnabled(False)
            self.ui.line_colour.setEnabled(False)
        if self.point_style is None:
            self.ui.point_outline_width.setEnabled(False)
            self.ui.point_outline_colour.setEnabled(False)
            self.ui.point_fill_colour.setEnabled(False)
            self.ui.point_size.setEnabled(False)

        # connect colour selection buttons to their actions
        self.ui.point_fill_colour.clicked.connect(self.update_point_fill_colour)
        self.ui.point_outline_colour.clicked.connect(self.update_point_outline_colour)
        self.ui.line_colour.clicked.connect(self.update_line_colour)

        # and when we update the other options
        self.ui.line_style.currentIndexChanged.connect(self.line_style_changed)
        self.ui.point_style.currentIndexChanged.connect(self.point_style_changed)

        self.ui.line_width.valueChanged.connect(self.line_width_changed)
        self.ui.point_size.valueChanged.connect(self.point_size_changed)
        self.ui.point_outline_width.valueChanged.connect(self.point_outline_width_changed)

    def update_point_fill_colour(self):
        colour = QColorDialog.getColor(self.point_fill_colour, self, "Symbol Fill Colour")
        if colour is not None and colour.isValid():
            self.point_fill_colour = colour
            self.ui.point_fill_colour.setStyleSheet("background-color: " + self.point_fill_colour.name())

            self.plot_data.setSymbolBrush(self.point_fill_colour)

    def update_point_outline_colour(self):
        colour = QColorDialog.getColor(self.point_outline_colour, self, "Symbol Outline Colour")
        if colour is not None and colour.isValid():
            self.point_outline_colour = colour
            self.ui.point_outline_colour.setStyleSheet("background-color: " + self.point_outline_colour.name())

            self.plot_data.setSymbolPen(self.point_outline_colour, width=self.point_outline_width)

    def update_line_colour(self):
        colour = QColorDialog.getColor(self.line_colour, self, "Line Colour")
        if colour is not None and colour.isValid():
            self.line_colour = colour
            self.ui.line_colour.setStyleSheet("background-color: " + self.line_colour.name())

            self.plot_data.setPen(self.line_colour, width=self.line_width, style=self.line_style)

    def line_style_changed(self, idx):
        self.line_style = self.ui.line_style.itemData(idx, Qt.UserRole)

        if self.line_style == Qt.NoPen:
            self.ui.line_width.setEnabled(False)
            self.ui.line_colour.setEnabled(False)
        else:
            self.ui.line_width.setEnabled(True)
            self.ui.line_colour.setEnabled(True)

        self.plot_data.setPen(self.line_colour, width=self.line_width, style=self.line_style)

    def point_style_changed(self, idx):
        self.point_style = self.ui.point_style.itemData(idx, Qt.UserRole)

        if self.point_style is None:
            self.ui.point_outline_width.setEnabled(False)
            self.ui.point_outline_colour.setEnabled(False)
            self.ui.point_fill_colour.setEnabled(False)
            self.ui.point_size.setEnabled(False)
        else:
            self.ui.point_outline_width.setEnabled(True)
            self.ui.point_outline_colour.setEnabled(True)
            self.ui.point_fill_colour.setEnabled(True)
            self.ui.point_size.setEnabled(True)

        self.plot_data.setSymbol(self.point_style)

    def line_width_changed(self, width):
        self.line_width = width

        self.plot_data.setPen(self.line_colour, width=self.line_width, style=self.line_style)

    def point_size_changed(self, size):
        self.point_size = size

        self.plot_data.setSymbolSize(self.point_size)

    def point_outline_width_changed(self, width):
        self.point_outline_width = width

        self.plot_data.setSymbolPen(self.point_outline_colour, width=self.point_outline_width)