
import numpy as np


def normalize_angle(z):
    '''
    Normalize angles to -pi to pi
    # http://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap
    '''
    return (np.array(z) + np.pi) % (2*np.pi) - np.pi
