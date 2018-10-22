##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

"""
Abstraction layer for other camera interfaces (pyicic, opencv, etc)

To implement a custom camera, subclass cameras.Camera, and implement the
methods with the @abstractmethod decorator.
"""

import abc
from abc import abstractmethod
import warnings

import numpy as np
import cv2


class CameraNotFoundError(IOError): pass
class NoFrameError(IOError): pass

# Should work across Python 2 and 3.
# see: https://gist.github.com/alanjcastonguay/25e4db0edd3534ab732d6ff615ca9fc1
ABC = abc.ABCMeta('ABC', (object,), {})

class Camera(ABC):
    @abstractmethod
    def __init__(self):
        """
        Camera needs an __init__ with no arguments, so it can be instantiated
        in a uniform way inside of the MAPLE __init__.

        Any dependencies specific to your camera should be imported as the first
        lines inside this function.

        Your __init__ implementation will likely create some other Python object
        to manage interactions with the camera through whatever other library.
        If this object is called `backing_camera_object`, the last line of your
        __init__ function should be:
        ```
        self.cam = backing_camera_object
        ```

        Then, to change other properties of the camera after __init__, you can
        access the camera of the MAPLE robot like so:
        ```
        robot = robotutil.MAPLE('MAPLE.cfg')
        camera_wrapper = robot.cam
        backing_camera_object = camera_wrapper.cam
        ```
        """
        pass

    @abstractmethod
    def get_frame(self):
        """Returns a current frame from the camera.
        
        Returned frame must be a numpy.ndarray of dimensions
        (height, width, color), where the colorspace is BGR.

        Raises NoFrameError if could not get a frame from the camera.
        """
        pass

    @abstractmethod
    def close(self):
        """Releases resources associated with connection to this camera.
        """
        pass

    def write_png(self, filename):
        """Write current frame to filename in PNG format.

        Args:
            filename (str): Path to save to.

        Raises NoFrameError if could not get a frame from the camera.
        """
        frame = self.get_frame()
        cv2.imwrite(filename, frame)

    def write_jpg(self, filename, quality=95):
        """Write current frame to filename in JPG format, with optional quality.

        Args:
            filename (str): Path to save to.
            quality (int): (optional) 0-100 jpg quality (100=highest quality)

        Raises NoFrameError if could not get a frame from the camera.
        """
        frame = self.get_frame()
        IMWRITE_JPEG_QUALITY = 1
        cv2.imwrite(filename, frame, [IMWRITE_JPEG_QUALITY, quality])


class PyICIC_Camera(Camera):
    def __init__(self):
        import pyicic.IC_ImagingControl

        ic_ic = pyicic.IC_ImagingControl.IC_ImagingControl()
        ic_ic.init_library()
        cam_names = ic_ic.get_unique_device_names()

        if len(cam_names) == 0:
            raise CameraNotFoundError('pyicic camera not found.')

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
        # TODO is there much overhead w/ start_live / stop_live calls?
        # mechanism to keep live?
        self.cam.start_live()
        self.cam.snap_image()
        imgdata, w, h, d = self.cam.get_image_data()
        self.cam.stop_live()
        return np.ndarray(buffer=imgdata, dtype=np.uint8, shape=(h, w, d))

    def write_jpg(self, filename, quality=50):
        """Writes a current image to filename in jpg format.
        """
        self.cam.start_live()
        self.cam.snap_image()
        # the 1 is for JPG, 0 is for BMP
        self.cam.save_image(filename, 1, jpeq_quality=qualPic)
        self.cam.stop_live()

    def close(self):
        """Releases resources associated with connection to this camera.
        """
        self.cam.close()


class OpenCVCamera(Camera):
    def __init__(self):
        if hasattr(cv2, 'cv'):
            self.cv2_v3 = False
        else:
            self.cv2_v3 = True

        cam = cv2.VideoCapture(0)
        if not cam.isOpened():
            raise CameraNotFoundError('OpenCV compatible camera not found.')

        self.cam = cam

    def get_frame(self):
        """Returns a current frame from the camera.

        Raises NoFrameError if could not get a frame from the camera.
        """
        # TODO detect and give meaningful error message for that select timeout
        # err? (having trouble reproducing now... camera wasn't closed
        # properly?)
        success, data = self.cam.read()
        if not success:
            raise NoFrameError('OpenCV VideoCapture.read() failed.')
        else:
            return data

    def close(self):
        """Releases resources associated with connection to this camera.
        """
        self.cam.release()

    def cap_prop_id(self, name):
        """Returns cv2 integer code for property with name."""
        # TODO also handle case where property doesn't exist
        return getattr(cv2 if self.cv2_v3 else cv2.cv,
                ('' if self.cv2_v3 else 'CV_') + 'CAP_PROP_' + name)

    def set_property(self, vc, name, value):
        """Sets cv2.VideoCapture property. Warns if can not."""
        print 'trying to set {} to {}'.format(name, value)
        property_code = self.cap_prop_id(name)
        v = vc.get(property_code)
        if v == -1:
            warnings.warn('Could not set property {}'.format(name))
        else:
            vc.set(property_code, value)

    # TODO provide method to list properties camera *does* seem to support

