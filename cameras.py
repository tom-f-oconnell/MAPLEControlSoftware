
"""
Abstraction layer for other camera interfaces (pyicic, opencv)
"""

import abc
from abc import abstractmethod
import warnings

import numpy as np
import cv2

# TODO module (file) level list of supported cam properties?
# then take dict of those @ constructors, via base class?

class CameraNotFoundError(IOError): pass

# Should work across Python 2 and 3.
# see: https://gist.github.com/alanjcastonguay/25e4db0edd3534ab732d6ff615ca9fc1
ABC = abc.ABCMeta('ABC', (object,), {})

class Camera(ABC):
    # TODO context manager fns for starting resources / freeing them? do any of
    # those functions actually need to be called for the kind of operation we
    # will use the cameras for?

    # TODO type hint on return type + comment on dims, colorspace, etc
    @abstractmethod
    def get_frame(self):
        pass

    # TODO provide instance variable for backing camera object

    def write_png(self, filename):
        """
        Args:
            filename (str): Path to save to.
        """
        # TODO make sure this uses the subclass get_frame
        frame = self.get_frame()
        cv2.imwrite(filename, frame)


    def write_jpg(self, filename, quality=50):
        """
        Args:
            filename (str): Path to save to.
            quality (int): (optional) 0-100 jpg quality (check bounds. 99?)
        """
        frame = self.get_frame()
        raise NotImplementedError
        # TODO add jpeg quality option
        cv2.imwrite(filename, frame)


class PyICIC_Camera(Camera):
    def __init__(self):
        import pyicic.IC_ImagingControl

        ic_ic = pyicic.IC_ImagingControl.IC_ImagingControl()
        ic_ic.init_library()
        cam_names = ic_ic.get_unique_device_names()

        if len(cam_names) == 0:
            raise CameraNotFoundError

        cam = ic_ic.get_device(cam_names[0])
        cam.open()

        cam.gain.value = 10
        cam.exposure.auto = False
        # TODO does a negative exposure make sense?
        cam.exposure.value = -10
        cam.set_video_format('BY8 (2592x1944)')
        cam.set_frame_rate(4.00)

        cam.prepare_live()

        self.cam = cam


    def get_frame(self):
        """Returns a current frame from the camera.
        """
        # TODO TODO normalized format. numpy array? colorspace? how to check?
        self.cam.start_live()
        self.cam.snap_image()
        imgdata, w, h, d = self.cam.get_image_data()
        self.cam.stop_live()
        return np.ndarray(buffer=imgdata, dtype=np.uint8, shape=(h, w, d))


    def write_jpg(self, filename, quality=50):
        """Writes a current image to filename in jpg format.
        """
        robot.cam.start_live()
        robot.cam.snap_image()
        # the 1 is for JPG, 0 is for BMP
        robot.cam.save_image(filename, 1, jpeq_quality=qualPic)
        robot.cam.stop_live()


class OpenCVCamera(Camera):
    def __init__(self):
        if hasattr(cv2, 'cv'):
            cv2_v3 = True
        else:
            cv2_v3 = False

        def cap_prop_id(name):
            """Returns cv2 integer code for property with name."""
            # TODO also handle case where property doesn't exist
            return getattr(cv2 if cv2_v3 else cv2.cv,
                ('' if cv2_v3 else 'CV_') + 'CAP_PROP_' + name)

        def set_property(vc, name, value):
            """Sets cv2.VideoCapture property. Warns if can not."""
            property_code = cap_prop_id(name)
            v = vc.get(property_code)
            if v == -1:
                warnings.warn('Could not set property {}'.format(name))
            else:
                vc.set(property_code, value)

        cam = VideoCapture(0)
        if not cam.isOpened():
            return None

        # these don't need to happen before resource is open, do they?
        set_property(cam, 'GAIN', 10)
        # TODO false? 0?
        # seems 0.25 might actually be value to turn off auto exposure?
        # https://github.com/opencv/opencv/issues/9738
        set_property(cam, 'AUTO_EXPOSURE', 0)
        set_property(cam, 'EXPOSURE', -10)
        # TODO this closest to by8 above? necessary?
        #set_property(cam, 'FORMAT', )
        # ?
        #set_property(cam, 'FRAME_WIDTH', )
        #set_property(cam, 'FRAME_HEIGHT', )
        set_property(cam, 'FPS', 4.00)

        # TODO provide unified camera cleanup call
        # and put cam.release() there in this case

        self.cam = cam


    def get_frame():
        """Returns a current frame from the camera.
        """
        # TODO TODO normalized format. numpy array? colorspace? how to check?
        return self.cam.read()
        #return np.ndarray(buffer=imgdata, dtype=np.uint8, shape=(h, w, d))

