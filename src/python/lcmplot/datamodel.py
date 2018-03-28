from PyQt4.QtCore import QAbstractListModel, QVariant, Qt


class DataModel(QAbstractListModel):
    def __init__(self, data, parent=None):
        super(DataModel, self).__init__(parent)
        self.provided_data = list(data.itervalues())
        self.provided_data.sort(cmp=lambda x, y: cmp(x.name, y.name))

        # question is do we filter by -> breaks ?? probably want to
        # although most of this is contained then in the data format here
        # doesn't interfere with later elements

        # we've sorted alphabetically, so just compare the splits on this with the next
        # and see what matches

    def breakup_by_message(self):
        # this is WIP for splitting the data from a list to a tree
        # we want to be able to selectively expand messages/platforms
        # indexing may be challenging
        last_split = list()

        for pd in self.provided_data:
            this_split = pd.name.split('->')

            for i, other in enumerate(last_split):
                if not other == this_split[i]:
                    break

            # i is the number of matching elements

    def rowCount(self, QModelIndex_parent=None, *args, **kwargs):
        return len(self.provided_data)

    def data(self, index, role=None):
        # need to be able to numerically index the data
        if not index.isValid():
            return QVariant()

        if index.row() >= len(self.provided_data):
            return QVariant()

        if role == Qt.DisplayRole:
            return QVariant(self.provided_data[index.row()].name)
        elif role == Qt.UserRole:
            return QVariant(self.provided_data[index.row()])

        return QVariant()

    def headerData(self, section, orientation, role=None):
        return QVariant()
