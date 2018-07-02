from pyqtgraph import AxisItem


class OffsetAxis(AxisItem):
    def __init__(self, orientation, offset, **kwargs):
        self.offset = offset
        self.offset_scale = 1e6
        super(OffsetAxis, self).__init__(orientation, **kwargs)

        self.setLabel('Time (s)')

    def setRange(self, mn, mx):
        super(OffsetAxis, self).setRange((mn - self.offset) / self.offset_scale, (mx - self.offset) / self.offset_scale)

    def update_offset(self, new_offset):
        mn = self.range[0] * self.offset_scale + self.offset
        mx = self.range[1] * self.offset_scale + self.offset

        self.offset = new_offset

        self.setRange(mn, mx)
