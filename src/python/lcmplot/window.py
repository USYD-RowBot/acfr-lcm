# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'plotwindow.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtWidgets

class Ui_DataPlotMain(object):
    def setupUi(self, DataPlotMain):
        DataPlotMain.setObjectName("DataPlotMain")
        DataPlotMain.resize(905, 679)
        self.centralwidget = QtWidgets.QWidget(DataPlotMain)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.dataTree = DragTreeView(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dataTree.sizePolicy().hasHeightForWidth())
        self.dataTree.setSizePolicy(sizePolicy)
        self.dataTree.setObjectName("dataTree")
        self.verticalLayout.addWidget(self.dataTree)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushRemove = QtWidgets.QPushButton(self.centralwidget)
        self.pushRemove.setObjectName("pushRemove")
        self.horizontalLayout_2.addWidget(self.pushRemove)
        self.pushAdd = QtWidgets.QPushButton(self.centralwidget)
        self.pushAdd.setObjectName("pushAdd")
        self.horizontalLayout_2.addWidget(self.pushAdd)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.graphicsView = DropPlotWidget(self.centralwidget)
        self.graphicsView.setObjectName("graphicsView")
        self.horizontalLayout.addWidget(self.graphicsView)
        DataPlotMain.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(DataPlotMain)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 905, 28))
        self.menubar.setObjectName("menubar")
        DataPlotMain.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(DataPlotMain)
        self.statusbar.setObjectName("statusbar")
        DataPlotMain.setStatusBar(self.statusbar)

        self.retranslateUi(DataPlotMain)
        QtCore.QMetaObject.connectSlotsByName(DataPlotMain)

    def retranslateUi(self, DataPlotMain):
        _translate = QtCore.QCoreApplication.translate
        DataPlotMain.setWindowTitle(_translate("DataPlotMain", "LCM Data Plot"))
        self.pushRemove.setText(_translate("DataPlotMain", "Remove"))
        self.pushAdd.setText(_translate("DataPlotMain", "Add"))


from .dropplot import DropPlotWidget
from .dragtree import DragTreeView
