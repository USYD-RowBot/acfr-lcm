# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'multiplotwindow.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.sourceView = DragTreeView(self.groupBox_2)
        self.sourceView.setObjectName("sourceView")
        self.verticalLayout_3.addWidget(self.sourceView)
        self.potentialView = QtWidgets.QListView(self.groupBox_2)
        self.potentialView.setObjectName("potentialView")
        self.verticalLayout_3.addWidget(self.potentialView)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.addplotbutton = QtWidgets.QPushButton(self.groupBox_2)
        self.addplotbutton.setObjectName("addplotbutton")
        self.horizontalLayout_3.addWidget(self.addplotbutton)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.verticalLayout.addWidget(self.groupBox_2)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.activeView = QtWidgets.QTreeView(self.groupBox)
        self.activeView.setObjectName("activeView")
        self.verticalLayout_2.addWidget(self.activeView)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.deleteplotbutton = QtWidgets.QPushButton(self.groupBox)
        self.deleteplotbutton.setObjectName("deleteplotbutton")
        self.horizontalLayout_2.addWidget(self.deleteplotbutton)
        self.swapaxesbutton = QtWidgets.QPushButton(self.groupBox)
        self.swapaxesbutton.setObjectName("swapaxesbutton")
        self.horizontalLayout_2.addWidget(self.swapaxesbutton)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.verticalLayout.addWidget(self.groupBox)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.plotView = DropPlotWidget(self.centralwidget)
        self.plotView.setObjectName("plotView")
        self.horizontalLayout.addWidget(self.plotView)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 19))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_2.setTitle(_translate("MainWindow", "New Plots"))
        self.addplotbutton.setText(_translate("MainWindow", "Add Plot"))
        self.groupBox.setTitle(_translate("MainWindow", "Active Plots"))
        self.deleteplotbutton.setText(_translate("MainWindow", "Delete Plot"))
        self.swapaxesbutton.setText(_translate("MainWindow", "Swap X and Y"))

from .dragtree import DragTreeView
from .dropplot import DropPlotWidget
