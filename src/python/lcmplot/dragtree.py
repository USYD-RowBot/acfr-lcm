from PyQt5.QtWidgets import QTreeView
from PyQt5.QtCore import Qt, QMimeData
from PyQt5.QtGui import QDrag


class DragTreeView(QTreeView):
    def __init__(self, *args, **kwargs):
        super(DragTreeView, self).__init__(*args, **kwargs)
        self.drag_start_position = None
        self.keys = None

    def mousePressEvent(self, ev):
        # call the parent class to let it select the objects
        # or do whatever it needs
        super(DragTreeView, self).mousePressEvent(ev)

        if ev.buttons() == Qt.LeftButton:
            # get the start position so we can detect dragging
            self.drag_start_position = ev.pos()


            # also record the SINGLE selected graph
            # TODO: handle multiple entries
            for idx in self.selectedIndexes():
                obj = self.model().data(idx, Qt.UserRole).value()

                if obj is not None:
                    self.keys = obj.name

    def mouseReleaseEvent(self, ev):
        if ev.buttons() == Qt.LeftButton:
            self.drag_start_position = None

    def mouseMoveEvent(self, ev):
        if not ev.buttons() == Qt.LeftButton:
            return

        if (ev.pos() - self.drag_start_position).manhattanLength() < 10:
            return

        drag = QDrag(self)

        data = QMimeData()

        print "plain/text:", self.keys
        data.setText(self.keys)
        drag.setMimeData(data)

        action = drag.exec_(Qt.CopyAction)
