import numpy as np


def get_filters():
    filters = {
            'None': None,  # quick test if none just use input, makes construction easier
            'Degrees to Radians': deg2rad,
            'Radians to Degrees': rad2deg,
            }


    return filters

def deg2rad(input_data):
    return np.array(input_data) * np.pi / 180.0

def rad2deg(input_data):
    return np.array(input_data) * 180.0 / np.pi
