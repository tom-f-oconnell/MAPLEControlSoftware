
"""
Elemental functional units MAPLE can interact with to run experiments.

Each module can source flies, sink flies, or do both.
"""

from __future__ import print_function
from __future__ import division

import abc
from abc import abstractmethod
import os
import pickle

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
            n_cols, n_rows, to_first_anchor, anchor_spacing, loaded=False,
            position_correction=True):

        super(Array, self).__init__(robot, offset, extent,
            flymanip_working_height)

        self.n_cols = n_cols
        self.n_rows = n_rows
        self.to_first_anchor = to_first_anchor
        # TODO let this be an arbitrary xy vector
        self.anchor_spacing = anchor_spacing
        self.full = np.full((self.n_cols, self.n_rows), loaded)

        self.correction = None

        if position_correction:
            self.fit_correction()
       

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
        # TODO TODO TODO definitely refactor (so that my choice script doesnt
        # try to get a fly from the flyplate before doing calibration on arena)
        # just a separate checking function that needs to be invoked? back to
        # constructor, and just don't construct objects before it's time to
        # calibrate them?
        if self.correction is None and self.position_correction:
            self._build_correction()

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


    def fit_correction(self):
        """
        """
        # TODO if there are different jigs, with different errors,
        # but one module of the same class in the same place in both
        # workspaces, that might be a strong case for more explicit handling
        # of workspaces, so the two could have their own corrections.

        # TODO test w/ "old" and "new-style" classes
        # might need to just use __name__ in latter case
        cls = self.__class__.__name__
        workspace_file = os.path.expanduser('~/.maple_workspace.p')

        if os.path.exists(workspace_file):
            print('Found saved module position transforms...')
            with open(workspace_file, 'rb') as f:
                corrections = pickle.load(f)

            if cls in corrections:
                for ox, oy, data in corrections[cls]:
                    if np.allclose([ox, oy], self.offset):
                        print('Found saved corrections for this module.')
                        self.correction = data['correction']
                        return

            print('No transform for current module')
                
        else:
            corrections = dict()

        corrections[cls] = []

        print('Measuring error to a few points...')
        xy_points, errors = self.measure_errors()
        print('Computing linear transform...')
        correction = self.compute_correction(xy_points, errors)

        data = dict()
        data['points'] = xy_points
        data['errors'] = errors
        data['correction'] = correction

        corrections[cls].append((self.offset[0], self.offset[1], data))

        with open(workspace_file, 'wb') as f:
            print('Saving transform to {}'.format(workspace_file))
            pickle.dump(corrections, f)


    def shape(self):
        # TODO convert to property?
        return (self.n_cols, self.n_rows)

    # TODO TODO provide general function like this (maybe even in Module)
    # where you can manually move robot to define the position of some key
    # points, and the best (x,y) offset (and maybe also some linear correction)
    # is saved to config file automatically
    def measure_errors(self, err_from_centering=True):
        """To manually record errors between Z2 effector and anchors.

        If `err_from_centering` is True, you control the robot in X and Y until
        Z2 appears centered over each anchor. Otherwise, you measure the
        distance between the anchor centers and where the Z2 end effector is,
        and enter those errors.
        """
        # TODO TODO make this fn (alternatively?) work by moving robot in X,Y
        # until centered, then measuring the distance moved as the error
        # (could be more precise than kinda eyeballing w/ calipers)
        if self.robot is None:
            raise IOError('can not measure_errors without a robot')

        self.effectors_to_travel_height()

        one_side = (0, 0)
        other_side = (self.n_cols - 1, self.n_rows - 1)
        index_coords = (one_side, other_side)

        # TODO allow motion in Z while correcting XY in err_from_centering case?
        # (to check effector can enter hole)

        points = []
        errors = []
        for i, j in index_coords:
            point = self.anchor_center(i, j)

            self.robot.moveXY(point)

            # TODO TODO factor this repeated parsing attempt for a float into a
            # function
            # TODO use __future__ for input and use that instead of raw_input
            cumulative_dz2 = 0.0
            while True:
                r = raw_input('Amount to move Z2 (+=down) ' +
                        '(empty to continue):\n')
                if r == '':
                    d_z2 = 0.0
                    break
                try:
                    d_z2 = float(r)
                    curr_z2 = self.robot.currentPosition[4]
                    self.robot.moveZ2(curr_z2 + d_z2)
                    cumulative_dz2 += d_z2
                except ValueError:
                    print('Could not parse input as a float')
            print('Moved Z2 a total of {}'.format(cumulative_dz2))

            if err_from_centering:
                initial_xy = self.robot.currentPosition[:2].copy()

                while True:
                    r = raw_input('Amount to move in X to center (+=left) ' +
                            '(empty to continue):\n')
                    if r == '':
                        dx = 0.0
                        break
                    try:
                        dx = float(r)
                        curr_x = self.robot.currentPosition[0]
                        self.robot.moveX(curr_x + dx)
                    except ValueError:
                        print('Could not parse input as a float')

                while True:
                    r = raw_input('Amount to move in Y to center ' +
                        '(+=towards you) (empty to continue):\n')
                    if r == '':
                        dy = 0.0
                        break
                    try:
                        dy = float(r)
                        curr_y = self.robot.currentPosition[1]
                        self.robot.moveY(curr_y + dy)
                    except ValueError:
                        print('Could not parse input as a float')

                # TODO just save final_xy and use that to compute transform?
                final_xy = self.robot.currentPosition[:2]
                x_err, y_err = final_xy - initial_xy

            else:
                while True:
                    r = raw_input('Enter X error in mm (+ = effector is too ' +
                        'far to the left):\n')
                    try:
                        x_err = float(r)
                        break
                    except ValueError:
                        print('Could not parse input as a float')

                while True:
                    r = raw_input('Enter Y error in mm (+ = effector is too ' + 
                        'far towards you):\n')
                    try:
                        y_err = float(r)
                        break
                    except ValueError:
                        print('Could not parse input as a float')

            points.append(point)
            errors.append((x_err, y_err))

            # TODO is this behaving as it should?
            # TODO allow option to increase height before moving to next
            # point? (w/ flag to suppress as kwarg i think)
            self.effectors_to_travel_height()

        # TODO TODO save correction under a hash of type + initial estimate
        # (then just keep initial estimate constant)
        # (actually don't use a hash, just truncate coords for ID, so that it
        # is straightforward to check approximate equality of coords)

        # TODO TODO also worth saving indices?
        return points, errors


    def compute_correction(self, xy_points, errors):
        """
        Computes a linear tranform that maps xy points computed from
        anchor_center to better estimates of the true anchor positions in
        Smoothie coordinate space.

        Stores the correction s.t. it is applied on subsequent calls to
        anchor_center.
        """
        self.correction, _, _, _ = np.linalg.lstsq(xy_points, errors)
        return self.correction


    # TODO share more of this function with measure_error?
    def measure_working_distance(self, z_axis=2):
        """To manually record a working distance for an effector in this array.
        """
        if z_axis < 0 or z_axis > 2:
            raise ValueError('0,1,2 are only valid values for z_axis')

        if z_axis != 2:
            raise NotImplementedError

        if self.robot is None:
            raise IOError('can not measure_working_distance without a robot')

        self.effectors_to_travel_height()

        one_side = (0, 0)
        other_side = (self.n_cols - 1, self.n_rows - 1)
        index_coords = (one_side, other_side)

        points = []
        z_positions = []
        for i, j in index_coords:
            point = self.anchor_center(i, j)

            # TODO TODO shift this as appropriate if z is 0 or 1
            # (assuming origin is defined wrt z2, as i have)
            # (just in X if modules are aligned along X and Y axes)
            self.robot.moveXY(point)

            curr_z = None
            while True:
                r = raw_input('Amount to move Z2 (+=down) ' +
                        '(empty to record current position):\n')
                if r == '':
                    if curr_z is None:
                        print('Please move axis away from travel distance at ' +
                            'least once. Type 0 and press enter to use this ' +
                            'position anyway.')
                    else:
                        break
                try:
                    dz = float(r)
                    curr_z = self.robot.currentPosition[2 + z_axis]

                    # TODO just make a move fn that takes an index?
                    if z_axis == 0:
                        move_z_fn = self.robot.moveZ0
                    elif z_axis == 1:
                        move_z_fn = self.robot.moveZ1
                    elif z_axis == 2:
                        move_z_fn = self.robot.moveZ2

                    move_z_fn(curr_z + dz)

                except ValueError:
                    print('Could not parse input as a float')

            points.append(point)
            z_positions.append(curr_z)

            self.effectors_to_travel_height()

        print('Mean recorded Z{} position (from zero defined '.format(z_axis) +
            'by homing) {}'.format(np.mean(z_positions)))
        print('Mean Z{} position (with zero as the '.format(z_axis) +
            'worksurface, positive up) {}'.format(
                self.flymanip_working_height - np.mean(z_positions)))

        return points, z_positions


    def anchor_center(self, i, j):
        """
        Override if necessary.

        Function from indices i,j to x,y in Smoothie space, assuming x and y
        axes of module as the same as those axes of Smoothie space.
        """
        # TODO maybe just allow saving true smoothie (x,y) of EACH center s.t.
        # coords can be looked up from a table, in case where linear
        # transformation fails?
        ideal_center = np.array((self.offset[:2] + self.to_first_anchor +
            [i*self.anchor_spacing, j*self.anchor_spacing]))

        if self.correction is not None:
            # Estimate errors, and add those to the original estimates.
            return ideal_center.dot(self.correction) + ideal_center
        else:
            return ideal_center


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

