import numpy as np

import numbers

try:
    from cStringIO.StringIO import BytesIO
except:
    from io import BytesIO


def get_message_types(module_handle):

    fingerprint_mapping = {}

    possible_types = dir(module_handle)

    for ptype_name in possible_types:
        ptype = getattr(module_handle, ptype_name)
        if hasattr(ptype, '_packed_fingerprint'):
            fingerprint_mapping[ptype._get_packed_fingerprint()] = ptype

    return fingerprint_mapping


class Channel(object):
    def __init__(self, channel_name, data_type):
        self.channel_name = channel_name
        self.data_type = data_type
        # this is used when we are decoding
        # subtypes that are used for values
        # we don't yet attempt to decode lists
        # of lcm_types - but can handle types in
        # types in types etc.
        self.subchannels = {}

        # we use the attributes as keys
        # and include sub-type attributes as well
        # with the key attr.attr in place
        self.data = {}

        #TODO: Create attributes in dict

    def handle_latest(self, lcm_type_set, raw_data):
        # we need to decode and stash the attributes away here
        # we want the raw types as well, no casting to strings etc.
        # this will help when creating numpy arrays later

        if self.data_type is None:
            # can't decode, so nothing to do
            return
        
        message = self.data_type.decode(raw_data)

        attributes = self.list_attributes(lcm_type_set, message)

        for key, value in attributes:
            if not key in self.data:
                self.data[key] = [value]
            else:
                self.data[key].append(value)

    def list_attributes(self, lcm_type_set, message):
        attributes = []
        # pylcm uses slots to declare the variable elements so we
        # iterate over them, no need to check anything else
        for attr in message.__slots__:
            value = getattr(message, attr)
            if type(value) in lcm_type_set:
                # we have a 'subchannel' with an lcm type we know
                # so split it out and get the attributes.

                # if this is the first time decoding create the subchannel
                if not attr in self.subchannels:
                    self.subchannels[attr] = Channel(attr, type(value))
                    
                # get all the subtypes attributes
                for sub_attr, value in self.subchannels[attr].list_attributes(lcm_type_set, value):
                    attributes.append((attr + '.'+ sub_attr, value))
                # don't want to append this
                continue
            else:
                attributes.append((attr, value))

        return attributes

class PlotData(object):
    def __init__(self, name, time, data):
        self.name = name
        self.time = time
        self.data = data

class MessageDecoder(object):
    def __init__(self, lcm_types):
        self.lcm_types = lcm_types
        self.lcm_type_set = set(lcm_types.values())

        self.channels = {}

    def handle_message(self, channel_name, raw_data):
        # if we haven't seen the channel
        # attempt detection
        if channel_name not in self.channels:
            self.detect_type(channel_name, raw_data)

        self.channels[channel_name].handle_latest(self.lcm_type_set, raw_data)

    def detect_type(self, channel_name, raw_data):
        # check the fingerprint against the known types
        buf = BytesIO(raw_data)

        fingerprint = buf.read(8)

        if fingerprint in self.lcm_types:
            data_type = self.lcm_types[fingerprint]
        else:
            data_type = None
        
        self.channels[channel_name] = Channel(channel_name, data_type)

    def export(self):
        # we have finished loading the data
        # now time to export
        # for each channel we need to extra info about the types contained
        # and if they are plottable. We can discard any non-numeric types
        # and also want to check for the presence of utime
        pass

        # each attribute gets an x and y
        # where x is a reference to the utime

        # if non numeric skip it
        data = {}

        for channel_name, channel_data in self.channels.iteritems():
            try:
                time = np.array(channel_data.data['utime'])
            except:
                if channel_data.data_type is not None:
                    print "{} has no 'utime' attribute".format(channel_name)
                continue
            for attribute_name, attribute_data in channel_data.data.iteritems():
                if isinstance(attribute_data[0], numbers.Number):
                    # numeric!
                    y = np.array(attribute_data)
                    name = channel_name + '->' + attribute_name
                    data[name] = PlotData(name, time, y)
        
        return data
