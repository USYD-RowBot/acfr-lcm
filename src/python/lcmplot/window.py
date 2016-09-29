# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'plotwindow.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_DataPlotMain(object):
    def setupUi(self, DataPlotMain):
        DataPlotMain.setObjectName(_fromUtf8("DataPlotMain"))
        DataPlotMain.resize(905, 679)
        self.centralwidget = QtGui.QWidget(DataPlotMain)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.dataTree = QtGui.QTreeView(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dataTree.sizePolicy().hasHeightForWidth())
        self.dataTree.setSizePolicy(sizePolicy)
        self.dataTree.setObjectName(_fromUtf8("dataTree"))
        self.verticalLayout.addWidget(self.dataTree)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.pushRemove = QtGui.QPushButton(self.centralwidget)
        self.pushRemove.setObjectName(_fromUtf8("pushRemove"))
        self.horizontalLayout_2.addWidget(self.pushRemove)
        self.pushAdd = QtGui.QPushButton(self.centralwidget)
        self.pushAdd.setObjectName(_fromUtf8("pushAdd"))
        self.horizontalLayout_2.addWidget(self.pushAdd)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.graphicsView = PlotWidget(self.centralwidget)
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.horizontalLayout.addWidget(self.graphicsView)
        DataPlotMain.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(DataPlotMain)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 905, 28))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        DataPlotMain.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(DataPlotMain)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        DataPlotMain.setStatusBar(self.statusbar)

        self.retranslateUi(DataPlotMain)
        QtCore.QMetaObject.connectSlotsByName(DataPlotMain)

    def retranslateUi(self, DataPlotMain):
        DataPlotMain.setWindowTitle(_translate("DataPlotMain", "LCM Data Plot", None))
        self.pushRemove.setText(_translate("DataPlotMain", "Remove", None))
        self.pushAdd.setText(_translate("DataPlotMain", "Add", None))

from pyqtgraph import PlotWidget
