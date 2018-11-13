
"""
Elemental functional units MAPLE can interact with to run experiments.

Each module can source flies, sink flies, or do both.
"""

from __future__ import print_function
from __future__ import division

import abc
from abc import abstractmethod

import numpy as np

# All units in millimeters.
ABC = abc.ABCMeta('ABC', (object,), {})

class Module(ABC):
    def __init__(self, robot, offset, extent, flymanip_working_height):
        self.robot = robot

        # The top right (min(x), min(y)) of this module's bounding box, minus
        # the robot's origin, which should be (0, 0).
        if len(offset) != 2:
            raise ValueError('expecting length of offset to be 2 (x, y)')
        self.offset = np.array(offset)

        # (x, y, z) dimensions of bounding box
        if len(extent) != 3:
            raise ValueError('expecting length of extent to be 3 (x, y, z)')
        self.extent = np.array(extent)

        # Defined from 0 at the bottom of bounding box, in Z dimension.
        self.flymanip_working_height = flymanip_working_height

    def effectors_to_travel_height(self):
        # TODO config option to set buffer above module height?
        if self.robot is None:
            return

        # TODO add support for z1
        z0_t = self.robot.z0_to_worksurface - self.extent[2] - 2
        z2_t = self.robot.z2_to_worksurface - self.extent[2] - 2
        current_z = self.robot.currentPosition[2:]
        assert len(current_z) == 3

        # TODO implement s.t. multiple axes can move at once, while leaving
        # arbitrary combinations (according to some combination) where they are
        if current_z[0] <= z0_t:
            print('Z0 already above minimum travel height ({} <= {})'.format(
                current_z[0], z0_t))
        else:
            print('Moving part manipulator to travel height: {}'.format(z0_t))
            self.robot.moveZ0(z0_t)

        if current_z[2] <= z2_t:
            print('Z2 already above minimum travel height ({} <= {})'.format(
                current_z[2], z2_t))
        else:
            print('Moving fly manipulator to travel height: {}'.format(z2_t))
            self.robot.moveZ2(z2_t)

    # TODO so should this still be able to hold a point?
    # may depend on whether i want to do each transform in module, or all at
    # workspace level?

    # TODO check either source or sink is subclassed? (maybe not possible in a
    # clean way...)


# TODO ABC work after one level of inheritance? if not, can i use it as a mixin?
class Source(Module):
    def __init__(self, robot, offset, extent, flymanip_working_height):
        super(Source, self).__init__(robot, offset, extent,
                                     flymanip_working_height)

    # TODO return a sequence of steps needed to get a fly in (including opening
    # doors, etc?) just execute the gcode?
    # TODO typehint to indicat this returns None (?)
    @abstractmethod
    def get(self):
        """
        """
        pass

    # TODO typehint to indicate this returns boolean
    @abstractmethod
    def is_empty(self):
        """
        """
        pass

    def has_flies(self):
        """
        """
        return not self.is_empty()

    # TODO abstract method on some kind of generator of the actions?


class Sink(Module):
    def __init__(self, robot, offset, extent, flymanip_working_height):
        # this also work with python3?
        super(Sink, self).__init__(robot, offset, extent,
                                   flymanip_working_height)

    @abstractmethod
    def put(self):
        """
        """
        pass

    @abstractmethod
    def is_full(self):
        """
        """
        pass


# TODO need to do more than have some abstractmethods to prevent instantiation?
class Array(Source, Sink):
    def __init__(self, robot, offset, extent, flymanip_working_height,
            n_cols, n_rows, to_first_anchor, anchor_spacing, loaded=False):

        super(Array, self).__init__(robot, offset, extent,
            flymanip_working_height)

        self.n_cols = n_cols
        self.n_rows = n_rows
        self.to_first_anchor = to_first_anchor
        # TODO let this be an arbitrary xy vector
        self.anchor_spacing = anchor_spacing
        self.full = np.full((self.n_cols, self.n_rows), loaded)
        
    # TODO maybe use optional args?
    @abstractmethod
    def get(self, xy, ij):
        pass

    @abstractmethod
    def put(self, xy, ij):
        pass

    # TODO possible to make it so they can implement either *_xy methods, or
    # override indices methods?

    def put_indices(self, i, j):
        self.effectors_to_travel_height()
        xy = self.anchor_center(i, j)
        self.put(xy, (i,j))
        self.effectors_to_travel_height()
        self.full[i, j] = True

    def get_indices(self, i, j):
        self.effectors_to_travel_height()
        # TODO rename to _position / coords / xy?
        xy = self.anchor_center(i, j)
        self.get(xy, (i,j))
        self.effectors_to_travel_height()
        self.full[i, j] = False

    def anchor_center(self, i, j):
        """
        Override if necessary.
        """
        return (self.offset[:2] + self.to_first_anchor +
            [i*self.anchor_spacing, j*self.anchor_spacing])

    # TODO maybe put in a fn to check bounds of offsets + paths
    # TODO also let this be a function that generates positions in order?
    # (returns a generator?)
    # Should all be within extent, as should shifted GCode instructions.
    def next_anchor(self, full):
        # TODO reword
        """
        Helper method to get either the next anchor to put a fly in, if `full`
        is `False`, or to get a fly from, if `full` is `True`.
        """
        # TODO TODO in choice, may want to override which axis should be
        # iterated over first (put in constructor), to keep flies generally
        # close to behavior chambers (only really applies if not full...)
        mask = self.full if full else np.logical_not(self.full)
        coords = np.argwhere(mask)
        # TODO test sorting procedure is doing what i want
        # lexmax?
        # TODO flag to pick whether to start w/ columns or rows
        # (would need to thread up through calling methods?)
        return coords[np.lexsort((coords[:,1], coords[:,0]))][0]

    def random_anchor(self, full):
        # TODO reword
        """
        Helper method to randomly choose one of either the full or the empty
        anchors.
        """
        mask = self.full if full else np.logical_not(self.full)
        coords = np.argwhere(mask)
        return coords[np.random.choice(len(coords))]

    def get_next(self):
        # TODO time added order? (depends on how plate is being loaded...)
        i, j = self.next_anchor(True)
        self.get_indices(i, j)
        return i, j

    def get_random(self):
        i, j = self.random_anchor(True)
        self.get_indices(i, j)
        return i, j

    def put_next(self):
        i, j = self.next_anchor(False)
        self.put_indices(i, j)
        return i, j

    def put_random(self):
        i, j = self.random_anchor(False)
        self.put_indices(i, j)
        return i, j

    def is_full(self):
        return np.sum(self.full) == self.full.size

    def is_empty(self):
        return np.sum(np.logical_not(self.full)) == self.full.size


# TODO TODO it may be cleaner to construct without robot and pass robot down at
# some point, in which case a superclass "register_robot" method should work

class Morgue(Sink):
    """A dish with soapy water to discard used flies.
    """
    def __init__(self, robot, offset, diameter=54.0, height=14.75):
        """
        Default `diameter` and `height` are for the smaller part of a Petri
        dish.
        """
        extent = (diameter, diameter, height)
        flymanip_working_height = 18
        super(Morgue, self).__init__(robot, offset, extent,
              flymanip_working_height)
       
    # change name to match Array method names? optional args to unify?
    def put(self):
        """Ejects fly into the center of the morgue.

        Assumes fly is already in fly manipulator.
        """
        #if self.robot is None:
        print('Putting fly in morgue.')
        #return

        # TODO check we are already over morgue max z? when to pick max z?

        # could also pick a point, in the morgue, closer than the center
        center = self.offset + self.extent[:2] / 2.0

        if self.robot is not None:
            self.effectors_to_travel_height()

            self.robot.moveXY(center)

            # TODO update this to appropriate variables that config file is
            # supposed to set
            z0 = self.robot.z2_to_worksurface
            zw = z0 - self.flymanip_working_height
            print('Moving fly manipulator to working height: {}'.format(zw))
            self.robot.moveZ2(zw)

            self.robot.flyManipVac(False)
            self.robot.flyManipAir(True)
            self.robot.dwell_ms(2000)
            self.robot.flyManipAir(False)

            self.effectors_to_travel_height()


    def is_full(self):
        return False


class FlyPlate(Array):
    """Model of 96-well fly storage plate made by FlySorter.
    """
    def __init__(self, robot, offset, loaded=True):
        """
        loaded (bool): whether the plate starts as fully loaded. True if this is
            a plate that you are loading beforehand, False if you intend to load
            it during the experiment.
        """
        # TODO have full kwarg value influence whether this (can be) treated as
        # initial source
        # TODO handle more than just cases where plate is full or not full
        # detect! (or just get from manually entered database?)
        extent = (85.5, 128.0, 19.0)
        # Oriented long axis vertical, s.t. n_cols < n_rows.
        # Top right is A1, one to the left is B1.
        n_cols = 8
        n_rows = 12
        # TODO get from dimension drawing? this was just roughly measured
        to_first_well = (11.5, 13.0)
        well_spacing = 9.0
        # TODO test / play around with this
        flymanip_working_height = 15.0

        super(FlyPlate, self).__init__(robot, offset, extent,
              flymanip_working_height, n_cols, n_rows,
              to_first_well, well_spacing, loaded=loaded)


    def get(self, xy, ij):
        # TODO TODO just add a verbose flag, to print when using on a real robot
        #if self.robot is None:
        print('Getting fly from plate {} ({})'.format(ij, xy))
        #    return

        if self.robot is not None:
            # TODO any particular reason air is turned on in cft.homeWithdraw?
            # pinswap thing? mistake?
            self.robot.flyManipVac(True)
            self.robot.flyManipAir(False)
            self.robot.moveXY(xy)

            # TODO update this to appropriate variables that config file is
            # supposed to set
            z0 = self.robot.z2_to_worksurface
            zw = z0 - self.flymanip_working_height
            print('Moving fly manipulator to working height: {}'.format(zw))
            self.robot.moveZ2(zw)

            # TODO maybe copy vacuum burst thing in cft
            self.robot.dwell_ms(3000)


    def put(self, xy, ij):
        """
        Assuming fly manipulator vacuum is already on and air is off.
        """
        #if self.robot is None:
        print('Putting fly in plate {} ({})'.format(ij, xy))
        #    return

        if self.robot is not None:
            self.robot.moveXY(xy)

            # TODO update this to appropriate variables that config file is
            # supposed to set
            # TODO factor into a module fn?
            z0 = self.robot.z2_to_worksurface
            zw = z0 - self.flymanip_working_height
            print('Moving fly manipulator to working height: {}'.format(zw))
            self.robot.moveZ2(zw)

            # TODO TODO lower to working height
            # TODO could also experiment w/ just leaving vac on
            self.robot.flyManipVac(False)
            self.robot.flyManipAir(True)
            self.robot.dwell_ms(3000)

            # reason not to turn air off?
            self.robot.flyManipAir(False)

