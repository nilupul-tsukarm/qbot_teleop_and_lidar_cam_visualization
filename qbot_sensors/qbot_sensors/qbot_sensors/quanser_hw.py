"""Quanser hardware classes for the QBot Platform (copied from the Quanser
quick-start code) used by the ROS 2 sensor nodes."""
import platform
import numpy as np
from quanser.devices import (RangingMeasurements, RangingMeasurementMode,
                             DeviceError, RangingDistance)
from quanser.multimedia import (Video3D, Video3DStreamType, MediaError,
                                ImageFormat, ImageDataType)

# NOTE: Quanser's original check uses os.getlogin(), which crashes when a
# node is started by "ros2 launch" (no controlling terminal). The Jetson
# Orin Nano on the physical robot is aarch64, so that check alone is used.
IS_PHYSICAL_QBOTPLATFORM = (platform.machine() == 'aarch64')


class Camera3D():
    def __init__(
            self,
            mode='RGB, Depth',
            frameWidthRGB=1920,
            frameHeightRGB=1080,
            frameRateRGB=30.0,
            frameWidthDepth=1280,
            frameHeightDepth=720,
            frameRateDepth=15.0,
            frameWidthIR=1280,
            frameHeightIR=720,
            frameRateIR=15.0,
            deviceId='0',
            readMode=1,
            focalLengthRGB=np.array([[None], [None]], dtype=np.float64),
            principlePointRGB=np.array([[None], [None]], dtype=np.float64),
            skewRGB=None,
            positionRGB=np.array([[None], [None], [None]], dtype=np.float64),
            orientationRGB=np.array(
                [[None, None, None], [None, None, None], [None, None, None]],
                dtype=np.float64),
            focalLengthDepth=np.array([[None], [None]], dtype=np.float64),
            principlePointDepth=np.array([[None], [None]], dtype=np.float64),
            skewDepth=None,
            positionDepth=np.array([[None], [None], [None]], dtype=np.float64),
            orientationDepth=np.array(
                [[None, None, None], [None, None, None], [None, None, None]],
                dtype=np.float64)
        ):
        """This class configures RGB-D cameras (eg. Intel Realsense) for use.

        By default, mode is set to RGB&DEPTH, which reads both streams.
        Set it to RGB or DEPTH to get exclusive RGB or DEPTH streaming.
        If you specify focal lengths, principle points, skew as well as
        camera position & orientation in the world/inertial frame,
        camera instrinsics/extrinsic matrices can also be extracted
        using corresponding methods in this class.
        """

        self.mode = mode
        self.readMode = readMode
        self.streamIndex = 0

        self.imageBufferRGB = np.zeros(
            (frameHeightRGB, frameWidthRGB, 3),
            dtype=np.uint8
        )
        self.imageBufferDepthPX = np.zeros(
            (frameHeightDepth, frameWidthDepth, 1),
            dtype=np.uint16
        )
        self.imageBufferDepthM = np.zeros(
            (frameHeightDepth, frameWidthDepth, 1),
            dtype=np.float32
        )
        self.imageBufferIRLeft = np.zeros(
            (frameHeightIR, frameWidthIR, 1),
            dtype=np.uint8
        )
        self.imageBufferIRRight = np.zeros(
            (frameHeightIR, frameWidthIR, 1),
            dtype=np.uint8
        )

        self.frameWidthRGB = frameWidthRGB
        self.frameHeightRGB = frameHeightRGB
        self.frameWidthDepth = frameWidthDepth
        self.frameHeightDepth = frameHeightDepth
        self.frameWidthIR = frameWidthIR
        self.frameHeightIR = frameHeightIR

        self.focalLengthRGB = 2*focalLengthRGB
        self.focalLengthRGB[0, 0] = -self.focalLengthRGB[0, 0]
        self.principlePointRGB = principlePointRGB
        self.skewRGB = skewRGB
        self.positionRGB = positionRGB
        self.orientationRGB = orientationRGB

        self.focalLengthDepth = 2*focalLengthDepth
        self.focalLengthDepth[0, 0] = -self.focalLengthDepth[0, 0]
        self.principlePointDepth = principlePointDepth
        self.skewDepth = skewDepth
        self.positionDepth = positionDepth
        self.orientationDepth = orientationDepth

        try:
            self.video3d = Video3D(deviceId)
            self.streamOpened = False
            if 'rgb' in self.mode.lower():
                self.streamRGB = self.video3d.stream_open(
                    Video3DStreamType.COLOR,
                    self.streamIndex,
                    frameRateRGB,
                    frameWidthRGB,
                    frameHeightRGB,
                    ImageFormat.ROW_MAJOR_INTERLEAVED_BGR,
                    ImageDataType.UINT8
                )
                self.streamOpened = True
            if 'depth' in self.mode.lower():
                self.streamDepth = self.video3d.stream_open(
                    Video3DStreamType.DEPTH,
                    self.streamIndex,
                    frameRateDepth,
                    frameWidthDepth,
                    frameHeightDepth,
                    ImageFormat.ROW_MAJOR_GREYSCALE,
                    ImageDataType.UINT16
                )
                self.streamOpened = True
            if 'ir' in self.mode.lower():
                self.streamIRLeft = self.video3d.stream_open(
                    Video3DStreamType.INFRARED,
                    1,
                    frameRateIR,
                    frameWidthIR,
                    frameHeightIR,
                    ImageFormat.ROW_MAJOR_GREYSCALE,
                    ImageDataType.UINT8
                )
                self.streamIRRight = self.video3d.stream_open(
                    Video3DStreamType.INFRARED,
                    2,
                    frameRateIR,
                    frameWidthIR,
                    frameHeightIR,
                    ImageFormat.ROW_MAJOR_GREYSCALE,
                    ImageDataType.UINT8
                )
                self.streamOpened = True
            # else:
            #     self.streamRGB = self.video3d.stream_open(
            #         Video3DStreamType.COLOR,
            #         self.streamIndex,
            #         frameRateRGB,
            #         frameWidthRGB,
            #         frameHeightRGB,
            #         ImageFormat.ROW_MAJOR_INTERLEAVED_BGR,
            #         ImageDataType.UINT8
            #     )
            #     self.streamDepth = self.video3d.stream_open(
            #         Video3DStreamType.DEPTH,
            #         self.streamIndex,
            #         frameRateDepth,
            #         frameWidthDepth,
            #         frameHeightDepth,
            #         ImageFormat.ROW_MAJOR_GREYSCALE,
            #         ImageDataType.UINT8
            #     )
            #     self.streamOpened = True
            self.video3d.start_streaming()
        except MediaError as me:
            print(me.get_error_message())

    def terminate(self):
        """Terminates all started streams correctly."""

        try:
            self.video3d.stop_streaming()
            if self.streamOpened:
                if 'rgb' in self.mode.lower():
                    self.streamRGB.close()
                if 'depth' in self.mode.lower():
                    self.streamDepth.close()
                if 'ir' in self.mode.lower():
                    self.streamIRLeft.close()
                    self.streamIRRight.close()

            self.video3d.close()

        except MediaError as me:
            print(me.get_error_message())

    def read_RGB(self):
        """Reads an image from the RGB stream. It returns a timestamp
            for the frame just read. If no frame was available, it returns -1.
        """

        timestamp = -1
        try:
            frame = self.streamRGB.get_frame()
            while not frame:
                if not self.readMode:
                    break
                frame = self.streamRGB.get_frame()
            if not frame:
                pass
            else:
                frame.get_data(self.imageBufferRGB)
                timestamp = frame.get_timestamp()
                frame.release()
        except KeyboardInterrupt:
            pass
        except MediaError as me:
            print(me.get_error_message())
        finally:
            return timestamp

    def read_depth(self, dataMode='PX'):
        """Reads an image from the depth stream. Set dataMode to
            'PX' for pixels or 'M' for meters. Use the corresponding image
            buffer to get image data. If no frame was available, it returns -1.
        """
        timestamp = -1
        try:
            frame = self.streamDepth.get_frame()
            while not frame:
                if not self.readMode:
                    break
                frame = self.streamDepth.get_frame()
            if not frame:
                pass
            else:
                if dataMode == 'PX':
                    frame.get_data(self.imageBufferDepthPX)
                elif dataMode == 'M':
                    frame.get_meters(self.imageBufferDepthM)
                timestamp = frame.get_timestamp()
                frame.release()
        except KeyboardInterrupt:
            pass
        except MediaError as me:
            print(me.get_error_message())
        finally:
            return timestamp

    def read_IR(self, lens='LR'):
        """Reads an image from the left and right IR streams
            based on the lens parameter (L or R). Use the corresponding
            image buffer to get image data. If no frame was available,
            it returns -1.
        """
        timestamp = -1
        try:
            if 'l' in lens.lower():
                frame = self.streamIRLeft.get_frame()
                while not frame:
                    if not self.readMode:
                        break
                    frame = self.streamIRLeft.get_frame()
                if not frame:
                    pass
                else:
                    frame.get_data(self.imageBufferIRLeft)
                    timestamp = frame.get_timestamp()
                    frame.release()
            if 'r' in lens.lower():
                frame = self.streamIRRight.get_frame()
                while not frame:
                    if not self.readMode:
                        break
                    frame = self.streamIRRight.get_frame()
                if not frame:
                    pass
                else:
                    frame.get_data(self.imageBufferIRRight)
                    timestamp = frame.get_timestamp()
                    frame.release()
        except KeyboardInterrupt:
            pass
        except MediaError as me:
            print(me.get_error_message())
        finally:
            return timestamp

    def extrinsics_rgb(self):
        """Provides the Extrinsic Matrix for the RGB Camera"""
        # define rotation matrix from camera frame into the body frame
        transformFromCameraToBody = np.concatenate(
            (np.concatenate((self.orientationRGB, self.positionRGB), axis=1),
                [[0, 0, 0, 1]]),
            axis=0
        )

        return np.linalg.inv(transformFromCameraToBody)



    def intrinsics_rgb(self):
        """Provides the Intrinsic Matrix for the RGB Camera"""
        # construct the intrinsic matrix
        return np.array(
            [[self.focalLengthRGB[0,0], self.skewRGB,
                    self.principlePointRGB[0,0]],
             [0, self.focalLengthRGB[1,0], self.principlePointRGB[1,0]],
             [0, 0, 1]],
            dtype = np.float64
        )

    def extrinsics_depth(self):
        """Provides the Extrinsic Matrix for the Depth Camera"""
        # define rotation matrix from camera frame into the body frame
        transformFromCameraToBody = np.concatenate(
            (np.concatenate((self.orientationDepth, self.positionDepth),
                axis=1), [[0, 0, 0, 1]]),
            axis=0
        )

        return np.linalg.inv(transformFromCameraToBody)


    def intrinsics_depth(self):
        """Provides the Intrinsic Matrix for the Depth Camera"""
        # construct the intrinsic matrix
        return np.array(
            [[self.focalLengthDepth[0,0], self.skewDepth,
                self.principlePointDepth[0,0]],
             [0, self.focalLengthDepth[1,0], self.principlePointDepth[1,0]],
             [0, 0, 1]],
            dtype = np.float64
        )


    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.terminate()



class Lidar():
    """A class for interacting with common LiDAR devices.

    This class provides an interface for working with LiDAR devices, such as
    RPLidar and Leishen MS10 or M10P. It simplifies the process of reading measurements
    and managing connections with these devices.

    Attributes:
        numMeasurements (int): The number of measurements per scan.
        distances (numpy.ndarray): An array containing distance measurements.
        angles (numpy.ndarray): An array containing the angle measurements.

    Example usage:

    .. code-block:: python

        from lidar import Lidar

        # Initialize a Lidar device (e.g. RPLidar)
        lidar_device = Lidar(type='RPLidar')

        # Read LiDAR measurements
        lidar_device.read()

        # Access measurement data
        print((lidar_device.distances, lidar_device.angles))

        # Terminate the LiDAR device connection
        lidar_device.terminate()

    """

    def __init__(
            self,
            type='RPLidar',
            numMeasurements=384,
            rangingDistanceMode=2,
            interpolationMode=0,
            interpolationMaxDistance=0,
            interpolationMaxAngle=0
        ):
        """Initialize a Lidar device with the specified configuration.

        Args:
            type (str, optional): The type of LiDAR device
                ('RPLidar' or 'LeishenMS10' or 'LeishenM10P'). Defaults to 'RPLidar'.
            numMeasurements (int, optional): The number of measurements
                per scan. Defaults to 384.
            rangingDistanceMode (int, optional): Ranging distance mode
                (0: Short, 1: Medium, 2: Long). Defaults to 2.
            interpolationMode (int, optional): Interpolation mode
                (0: Normal, 1: Interpolated). Defaults to 0.
            interpolationMaxDistance (float, optional): Maximum distance
                for interpolation. Defaults to 0.
            interpolationMaxAngle (float, optional): Maximum angle for
                interpolation. Defaults to 0.
        """

        self.numMeasurements = numMeasurements
        self.distances = np.zeros((numMeasurements,1), dtype=np.float32)
        self.angles = np.zeros((numMeasurements,1), dtype=np.float32)
        self._measurements = RangingMeasurements(numMeasurements)
        self._rangingDistanceMode = rangingDistanceMode
        self._interpolationMode = interpolationMode
        self._interpolationMaxDistance = interpolationMaxDistance
        self._interpolationMaxAngle = interpolationMaxAngle

        if type.lower() == 'rplidar':
            self.type = 'RPLidar'
            from quanser.devices import RPLIDAR as RPL
            self._lidar = RPL()
            if not hasattr(self, "url"):
                self.url = ("serial-cpu://localhost:2?baud='115200',"
                        "word='8',parity='none',stop='1',flow='none',dsr='on'")
            # Open the lidar device with ranging mode settings.
            self._lidar.open(self.url, self._rangingDistanceMode)

        elif type.lower() == 'leishenms10':
            self.type = 'LeishenMS10'
            from quanser.devices import LeishenMS10
            self._lidar = LeishenMS10()
            if not hasattr(self, "url"):
                self.url = ("serial-cpu://localhost:2?baud='460800',"
                        "word='8',parity='none',stop='1',flow='none'")
            self._lidar.open(self.url, samples_per_scan = self.numMeasurements)

        elif type.lower() == 'leishenm10p':
            self.type = 'LeishenM10P'
            from quanser.devices import LeishenM10P
            self._lidar = LeishenM10P()
            if not hasattr(self, "url"):
                self.url = ("serial://localhost:0?baud='512000',"
                        "word='8',parity='none',stop='1',flow='none',device='/dev/lidar'") #serial://localhost:0?device='/dev/lidar',baud='512000',word='8',parity='none',stop='1',flow='none'
            self._lidar.open(self.url, samples_per_scan = self.numMeasurements)

        else:
            # TODO: Assert error
            return

        try:
            # Ranging distance mode check
            if rangingDistanceMode == 2:
                self._rangingDistanceMode = RangingDistance.LONG
            elif rangingDistanceMode == 1:
                self._rangingDistanceMode = RangingDistance.MEDIUM
            elif rangingDistanceMode == 0:
                self._rangingDistanceMode = RangingDistance.SHORT
            else:
                print('Unsupported Ranging Distance Mode provided.'
                        'Configuring LiDAR in Long Range mode.')
                self._rangingDistanceMode = RangingDistance.LONG

            # Interpolation check (will be used in the read method)
            if interpolationMode == 0:
                self._interpolationMode = RangingMeasurementMode.NORMAL
            elif interpolationMode == 1:
                self._interpolationMode = RangingMeasurementMode.INTERPOLATED
                self._interpolationMaxAngle = interpolationMaxAngle
                self._interpolationMaxDistance = interpolationMaxDistance
            else:
                print('Unsupported Interpolation Mode provided.'
                        'Configuring LiDAR without interpolation.')
                self._interpolationMode = RangingMeasurementMode.NORMAL

        except DeviceError as de:
            if de.error_code == -34:
                pass
            else:
                print(de.get_error_message())

    def read(self):
        """Read a scan and store the measurements

        Read a scan from the LiDAR device and store the measurements in the
        'distances' and 'angles' attributes.
        """
        flag = False
        try:
            numValues = self._lidar.read(
                self._interpolationMode,
                self._interpolationMaxDistance,
                self._interpolationMaxAngle,
                self._measurements
            )
            if numValues > 0:
                self.distances = np.array(self._measurements.distance)
                self.angles = np.array(self._measurements.heading)
                flag = True
        except DeviceError as de:
            if de.error_code == -34:
                pass
            else:
                print(de.get_error_message())
        finally:
            return flag

    def terminate(self):
        """Terminate the LiDAR device connection correctly."""
        try:
            self._lidar.close()
            # print("lidar closed")
        except DeviceError as de:
            if de.error_code == -34:
                pass
            else:
                print(de.get_error_message())

    def __enter__(self):
        """Return self for use in a 'with' statement."""
        return self

    def __exit__(self, type, value, traceback):
        """
        Terminate the LiDAR device connection when exiting a 'with' statement.
        """
        self.terminate()


class QBotPlatformLidar(Lidar):
    """Leishen M10P lidar on the QBot Platform."""
    def __init__(self, numMeasurements=1680, interpolationMode=0,
                 interpolationMaxDistance=0, interpolationMaxAngle=0):
        if IS_PHYSICAL_QBOTPLATFORM:
            self.url = ("serial://localhost:0?baud='512000',"
                        "word='8',parity='none',stop='1',flow='none',device='/dev/lidar'")
        else:
            self.url = "tcpip://localhost:18918"
        super().__init__(type='leishenm10p', numMeasurements=numMeasurements,
                         interpolationMode=interpolationMode,
                         interpolationMaxDistance=interpolationMaxDistance,
                         interpolationMaxAngle=interpolationMaxAngle)


class QBotPlatformRealSense(Camera3D):
    """Intel RealSense D435 on the QBot Platform."""
    def __init__(self, mode='RGB, Depth', width=640, height=480, fps=30.0):
        deviceId = '0' if IS_PHYSICAL_QBOTPLATFORM else "0@tcpip://localhost:18917"
        super().__init__(mode=mode,
                         frameWidthRGB=width, frameHeightRGB=height, frameRateRGB=fps,
                         frameWidthDepth=width, frameHeightDepth=height, frameRateDepth=fps,
                         frameWidthIR=width, frameHeightIR=height, frameRateIR=fps,
                         deviceId=deviceId, readMode=0)
