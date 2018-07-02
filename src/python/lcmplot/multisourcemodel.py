from PyQt5.QtCore import QAbstractItemModel, QVariant, Qt, QModelIndex, QThread
import numpy as np
import numbers
import weakref
import lcm

try:
    from cStringIO.StringIO import BytesIO
except ImportError:
    from io import BytesIO

QMI = QModelIndex

def get_message_types(module_handle):
    """Get the fingerprints of LCM types in the module and the mapping between."""

    fingerprint_mapping = {}

    possible_types = dir(module_handle)

    for type_name in possible_types:
        ptype = getattr(module_handle, type_name)
        if hasattr(ptype, '_packed_fingerprint'):
            fingerprint_mapping[ptype._get_packed_fingerprint()] = ptype

    return fingerprint_mapping


def lambda_bisect_left(data_list, target_key, lo=0, hi=None, key=lambda x: x):
    """Modified version of bisect_left to use a key function."""

    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = len(data_list)
    while lo < hi:
        mid = (lo+hi)//2
        if key(data_list[mid]) < target_key:
            lo = mid+1
        else:
            hi = mid
    return lo


class NumericElementData:
    def __init__(self, parent, element_name):
        self.name = element_name
        self.data = list()
        self.parent = weakref.ref(parent)

    def handle_message(self, data):
        self.data.append(data)

    def full_name(self, for_child=False):
        return self.parent().full_name(for_child=True) + self.name

    def children(self):
        return 0

    def row(self):
        return self.parent().child_row(self)


class ListData(object):
    def __init__(self, parent, attribute_name):
        self.name = attribute_name
        self.parent = weakref.ref(parent)
        self.entries = list()

    def children(self):
        return len(self.entries)

    def full_name(self, for_child=False):
        return self.parent().full_name(for_child=True) + self.name

    def child_row(self, child):
        return self.entries.index(child)

    def row(self):
        return self.parent().child_row(self)

    def handle_message(self, data):
        # the biggest problem here is we can't tell if the array is fixed length
        # or dynamic from python - the information is encoded in the type but within the
        # code and not in parameters that are exposed unfortunately.

        # the result here is that we may trim children if the array length shortens
        # and ignore any results beyond that.
        # depending on the actual information there may be something useful to retrieve
        # from these later
        if len(self.entries) == 0:
            # we haven't initialised
            # so create the child entries
            for ii, d in enumerate(data):
                entry_name = '[{}]'.format(ii)
                if isinstance(d, numbers.Number):
                    entry = NumericElementData(self, entry_name)
                else:
                    entry = TypeData(self, entry_name)

                self.entries.append(entry)

        # add the data to the entries
        for ii, d in enumerate(data):
            if ii == len(self.entries):
                # we need to cull, or do NaN for the rest
                # TODO: we also need to call beginRemoveRows on the model...
                self.entries = self.entries[:len(data)]
                break

            self.entries[ii].handle_message(d)


class TypeData(object):
    def __init__(self, parent, attribute_name, data_type=None):
        self.name = attribute_name  # the name of the channel the data came in on
        self.parent = weakref.ref(parent)
        self.data_type = data_type  # if defined this denotes we are a channel and how to decode
        self.entries = []  # the child elements - both leaf and branch

    def children(self):
        return len(self.entries)

    def child_row(self, child):
        return self.entries.index(child)

    def row(self):
        return self.parent().child_row(self)

    def full_name(self, for_child=False):
        # we use data_type to denote if this is a channel
        # or child member of a channel data type
        suffix = ""
        parent = ""
        if for_child:
            if self.data_type is None:
                suffix = "."
            else:
                suffix = "->"

        if self.data_type is None:
            parent = self.parent().full_name(for_child=True)
        else:
            # we could optionally get the file name here
            pass

        return parent + self.name + suffix

    def handle_message(self, data):
        # if we are actually a channel handler decode the message
        if self.data_type is not None:
            data = self.data_type.decode(data)

        # we may not have created sub-elements as yet
        if len(self.entries) == 0:
            # each attribute/member of an lcm type is defined in slots
            # and the only valid member types are:
            # numbers (float, integer, bytes and boolean),
            # arrays (of numbers, strings or other lcm types),
            # strings or
            # other lcm types.
            try:
                for attr in data.__slots__:
                    value = getattr(data, attr)
                    if isinstance(value, numbers.Number):
                        entry = NumericElementData(self, attr)
                    elif isinstance(value, (tuple, list)):
                        entry = ListData(self, attr)
                    elif isinstance(value, basestring):
                        continue  # we don't plot/can't plot strings
                    else:
                        entry = TypeData(self, attr)

                    self.entries.append(entry)
            except AttributeError as e:
                print e
                print data

        for de in self.entries:
            de.handle_message(getattr(data, de.name))


class LCMThread(QThread):
    def __init__(self, lcm):
        QThread.__init__(self)
        self.lcm = lcm
        self.exitting = False

    def __del__(self):
        self.wait(2000)

    def do_exit(self):
        self.exitting = True

    def run(self):
        while not self.exitting:
            self.lcm.handle_timeout(1000)


class MessageSource(object):
    def __init__(self, model, filename=None):
        self.entries = list()  # the sorted list we use to track display position
        self.channel_lookup = dict()  # a dictionary that points to the same objects
        self.parent = weakref.ref(model)
        self.filename = filename
        self.name = filename

    def process(self):
        if self.filename is not None:
            # we have a file to plot!
            # process all the observations immediately...
            #self.lcm = lcm.LCM("file://{}?speed=0".format(self.filename))
            lcm_log = lcm.EventLog(self.filename)

            total_size = lcm_log.size()
            last = 0
            step = 5
            for event in lcm_log:
                self.handle_message(event.channel, event.data)
                # decode progress
                percent = 100 * lcm_log.tell() / total_size
                if percent >= last + step:
                    print "{:3}".format(percent)
                    last += step

            self.name = self.filename  # perhaps change this to basename of the file?
        else:
            # assume using live plots otherwise
            self.lcm = lcm.LCM()
            self.name = "Live"
            self.lcm.subscribe(".*", self.handle_message)
            # we still need to handle the LCM object to get the messages

            self.lcm_thread = LCMThread(self.lcm)
            self.lcm_thread.start()


    def handle(self, timeout=None):
        if timeout is not None:
            self.lcm.handle_timeout(timeout)
        else:
            self.lcm.handle()

    def children(self):
        return len(self.entries)

    def child_row(self, child):
        return self.entries.index(child)

    def row(self):
        return self.parent().child_row(self)

    def handle_message(self, channel_name, raw_data):
        initial = False
        if channel_name not in self.channel_lookup:
            # we have a new channel
            # and now must find the type
            buf = BytesIO(raw_data)
            fingerprint = buf.read(8)

            lcm_types = self.parent().lcm_types

            if fingerprint in lcm_types:
                data_type = lcm_types[fingerprint]
            else:
                # can't decode... so move on
                return

            ct = TypeData(self, channel_name, data_type)
            idx = lambda_bisect_left(self.entries, channel_name, key=lambda x: x.name)

            self.entries.insert(idx, ct)
            self.channel_lookup[channel_name] = ct

            initial = True

            # we need to indicate we are inserting rows
            model = self.parent()
            parent = model.index(model.child_row(self), 0, QMI())
            model.beginInsertRows(parent, idx, idx)

        # always handle the message
        self.channel_lookup[channel_name].handle_message(raw_data)

        # we only call this on the creation of new messages
        if initial:
            self.parent().endInsertRows()


class DataModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super(DataModel, self).__init__(parent)

        self.sources = list()
        self.lcm_types = dict()

    def child_row(self, child):
        return self.sources.index(child)

    def add_lcm_types(self, lcm_types):
        self.lcm_types = lcm_types

    def create_live(self):
        name = "Live"
        idx = lambda_bisect_left(self.sources, name, key=lambda x: x.name)
        ms = MessageSource(self)

        self.beginInsertRows(QModelIndex(), idx, idx)
        self.sources.insert(idx, ms)
        self.endInsertRows()

        ms.process()

    def add_file(self, filename):
        idx = lambda_bisect_left(self.sources, filename, key=lambda x: x.name)
        ms = MessageSource(self, filename)

        self.beginInsertRows(QModelIndex(), idx, idx)
        self.sources.insert(idx, ms)
        self.endInsertRows()

        ms.process()

    def rowCount(self, QModelIndex_parent=None, *args, **kwargs):
        if QModelIndex_parent.column() > 0:
            return 0

        if QModelIndex_parent is None or not QModelIndex_parent.isValid():
            return len(self.sources)
        else:
            # we have defined a children to list the number of rows for each
            # object...
            parent = QModelIndex_parent.internalPointer()

            return parent.children()

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

    def index(self, row, column, QModelIndex_parent=None, *args, **kwargs):
        if not self.hasIndex(row, column, QModelIndex_parent):
            return QModelIndex()

        if not QModelIndex_parent.isValid():
            return self.createIndex(row, column, self.sources[row])
        else:
            return self.createIndex(row, column, QModelIndex_parent.internalPointer().entries[row])

    def parent(self, QModelIndex=None):
        if QModelIndex is None or not QModelIndex.isValid():
            return QMI()

        child_item = QModelIndex.internalPointer()

        if hasattr(child_item, 'parent'):
            parent_item = child_item.parent()
            if parent_item is self:
                return QMI()
            else:
                row = parent_item.row()

            return self.createIndex(row, 0, parent_item)
        else:
            return QMI()

    def columnCount(self, QModelIndex_parent=None, *args, **kwargs):
        return 1
