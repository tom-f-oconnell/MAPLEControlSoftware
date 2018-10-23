
"""
For making groups of modules relevant to experiments.
"""
# TODO might consider just using dicts like upstream, and deleting this file

from __future__ import print_function
from __future__ import division


class Workspace:
    modules = []
    # TODO store transform to each module (including rotations), rather than
    # just offsets
    # TODO or store as instance variables in each module?
    # tuples of (x_offset, y_offset)
    offsets = []


class Sequence:
    # TODO or is this supposed to be under init?
    """

    """
    def __init__(self):
        pass


def guess_sequence(workspace):
    # TODO check for flyplate / other sources / sinks
    #if 
    pass
