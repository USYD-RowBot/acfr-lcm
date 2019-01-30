import numpy as np


def get_filters():
    filters = {
            'None': None,  # quick test if none just use input, makes construction easier
            'Degrees to Radians': deg2rad,
            'Radians to Degrees': rad2deg,
            'Scale' : scale,
            }


    return filters

def get_filter_name(given_func):
    for key, func in get_filters().iteritems():
        if func is given_func:
            return key
    else:
        raise ValueError("Unknown function.")
        # return None ??

def deg2rad(input_data, option=None):
    return np.array(input_data) * np.pi / 180.0

def rad2deg(input_data, option=None):
    return np.array(input_data) * 180.0 / np.pi

def scale(input_data, option=None):
    # this catches all non-converts
    # such as empty strings or None
    # "" gives ValueError
    # None gives TypeError
    try:
        scale = float(option)
    except:
        scale = 1.0

    return np.array(input_data) * scale
