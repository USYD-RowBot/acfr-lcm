from PyQt5.QtCore import QAbstractItemModel, QVariant, Qt, QModelIndex

# some of the functions unfortunately shadow this
QMI = QModelIndex

import weakref


def custom_bisect_left(data_list, target_key, lo=0, hi=None, key=lambda x: x):
    """Return the index where to insert item x in list a, assuming a is sorted.
    The return value i is such that all e in a[:i] have e < x, and all e in
    a[i:] have e >= x.  So if x already appears in the list, a.insert(x) will
    insert just before the leftmost x already there.
    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """

    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = len(data_list)
    while lo < hi:
        mid = (lo+hi)//2
        if key(data_list[mid]) < target_key: lo = mid+1
        else: hi = mid
    return lo


class ElementData:
    def __init__(self, channel_data, element_name):
        self.name = element_name
        self.data = None
        self.time = None

        # not sure if we need this... but if we do it has to be a weakref
        self.parent = weakref.ref(channel_data)


class ChannelData:
    def __init__(self, channel_name, element_names):
        self.name = channel_name
        self.data_elements = []

        for en in element_names:
            self.data_elements.append(ElementData(self, en))


class DataModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super(DataModel, self).__init__(parent)
        self.provided_data = list()

    # works with just names - now need to copy data structure in properly
    # including time+linked data
    def insert_data(self, data):
        # group by prefix
        names = list(data.iterkeys())
        prefix_groups = dict()
        for name in names:
            prefix = name.split('->')[0]

            if prefix in prefix_groups:
                prefix_groups[prefix].append(name)
            else:
                prefix_groups[prefix] = [name]

        # insert each prefix set in one go
        for prefix, names in prefix_groups.iteritems():
            n = len(names)
            # find the location of the insert
            row = custom_bisect_left(self.provided_data, prefix, key=lambda x: x.name)

            if row == len(self.provided_data) or not self.provided_data[row].name == prefix:
                cdata = ChannelData(prefix, names)
                for edata in cdata.data_elements:
                    edata.data = data[edata.name].data
                    edata.time = data[edata.name].time

                self.beginInsertRows(QModelIndex(), row, row+1)
                self.provided_data.insert(row, cdata)
                self.endInsertRows()

        return True

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
        if QModelIndex_parent.column() > 0:
            return 0

        if QModelIndex_parent is None or not QModelIndex_parent.isValid():
            return len(self.provided_data)
        else:
            # we have either channel or element data
            parent = QModelIndex_parent.internalPointer()

            if hasattr(parent, 'parent'):
                return 0
            else:
                return len(parent.data_elements)

    def get_by_name(self, name):
        for entry in self.provided_data:
            if entry.name == name:
                return entry
        else:
            return None

    def data(self, index, role=None):
        # need to be able to numerically index the data
        if not index.isValid():
            return QVariant()

        if role == Qt.DisplayRole:
            return QVariant(index.internalPointer().name)
        elif role == Qt.UserRole:
            return QVariant(index.internalPointer())

        return QVariant()

    def headerData(self, section, orientation, role=None):
        return QVariant()

    def index(self, p_int, p_int_1, QModelIndex_parent=None, *args, **kwargs):
        if not self.hasIndex(p_int, p_int_1, QModelIndex_parent):
            return QModelIndex()

        if not QModelIndex_parent.isValid():
            return self.createIndex(p_int, p_int_1, self.provided_data[p_int])
        else:
            return self.createIndex(p_int, p_int_1, QModelIndex_parent.internalPointer().data_elements[p_int])

    def parent(self, QModelIndex=None):
        if QModelIndex is None or not QModelIndex.isValid():
            return QMI()

        child_item = QModelIndex.internalPointer()

        if hasattr(child_item, 'parent'):
            parent_item = child_item.parent()
            row = self.provided_data.index(parent_item)

            return self.createIndex(row, 0, parent_item)
        else:
            return QMI()

    def columnCount(self, QModelIndex_parent=None, *args, **kwargs):
        return 1
