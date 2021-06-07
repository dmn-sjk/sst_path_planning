import numpy as np

def remove_array(L, arr):
    ind = 0
    size = len(L)
    while ind != size and not np.array_equal(L[ind], arr):
        ind += 1
    if ind != size:
        L.pop(ind)
        return L
    else:
        raise ValueError('array not found in list.')

def el_in_array(el, array):
    for node in array:
        if np.array_equal(el, node):
            return True
    return False