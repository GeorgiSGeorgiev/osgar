"""
    Osgar driver for Luxonis OAK cameras. (depthai v3)
    https://www.luxonis.com/
"""
from threading import Thread
from pathlib import Path
import logging

import depthai as dai
import numpy as np
import cv2


g_logger = logging.getLogger(__name__)

# ----------- oak_camera_v2.py copy & paste ----------------
g_resolution_dic = {
    "THE_400_P": (640, 400),
    "THE_480_P": (640, 480),
    "THE_720_P": (1280, 720),
    "THE_800_P": (1280, 800),
    "THE_1080_P": (1920, 1080),
    "THE_4_K": (3840, 2160),
    "THE_12_MP": (4000, 3000),
    "THE_13_MP": (4160, 3120)
}

def get_video_encoder(name):
    # https://docs.luxonis.com/projects/api/en/latest/components/nodes/video_encoder/
    if name == 'h264':
        return dai.VideoEncoderProperties.Profile.H264_MAIN
    elif name == 'h265':
        return dai.VideoEncoderProperties.Profile.H265_MAIN
    elif name == 'mjpeg':
        return dai.VideoEncoderProperties.Profile.MJPEG
    else:
        assert 0, f'"{name}" is not supported'
# ----------- END OF oak_camera_v2.py copy & paste ----------------


def cam_is_available(cam_id):
    devices = dai.Device.getAllAvailableDevices()
    available_devices_names = []
    available_devices_MxId = []
    for dev in devices:
        if cam_id == dev.name or cam_id == dev.getDeviceId():
            return True
        available_devices_names.append(dev.name)
        available_devices_MxId.append(dev.getDeviceId())

    g_logger.error(f"{cam_id} was not found!")
    g_logger.info(f'Found device names: {", ".join(available_devices_names)}')
    g_logger.info(f'Found device MxIds: {", ".join(available_devices_MxId)}')

    return False


class OakCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        compress_depth_stream = config.get('compress_depth_stream', True)
        self.bus.register('depth:gz' if compress_depth_stream else 'depth',
                          'color', 'orientation_list', 'detections', 'left_im', 'right_im',
                          # *_seq streams are needed for output sync amd they are published BEFORE payload data
                          'depth_seq', 'color_seq', 'detections_seq', 'left_im_seq', 'right_im_seq',
                          'nn_mask:gz',
                          'pose3d', 'gridmap',
                          'redroad:gz', 'robotourist:gz',
                          'qr_code')

        self.is_color = config.get('is_color', False)
        self.is_depth = config.get('is_depth', False)
        self.is_stereo_images = config.get('is_stereo_images', False)
        self.fps = config.get('fps', 10)
        self.subsample = config.get('subsample')
        self.cam_id = config.get('cam_id')

        color_resolution_value = config.get("color_resolution", "THE_1080_P")
        self.color_resolution = g_resolution_dic[color_resolution_value]
        self.mono_resolution = config.get("mono_resolution", (640, 400))
        if isinstance(self.mono_resolution, str):
            assert self.mono_resolution in g_resolution_dic, self.mono_resolution
            self.mono_resolution = g_resolution_dic[self.mono_resolution]

        # NOTE (v3 fix): device.setIrLaserDotProjectorIntensity()/setIrFloodLightIntensity()
        # take a 0..1 intensity fraction in depthai v3, NOT mA (that was the v2
        # setIrLaserDotProjectorBrightness(mA) API). Config keeps mA units for
        # continuity with existing configs/intuition; we convert to a fraction
        # of the hardware's rated max (1200mA laser / 1500mA flood) below.
        laser_current_mA = config.get("laser_projector_current")
        if laser_current_mA is not None:
            assert 0 <= laser_current_mA <= 1200, laser_current_mA  # The limit is 1200 mA.
        self.laser_projector_current = (
            None if laser_current_mA is None else min(1.0, laser_current_mA / 1200))
        flood_current_mA = config.get("flood_light_current")
        if flood_current_mA is not None:
            assert 0 <= flood_current_mA <= 1500, flood_current_mA  # The limit is 1500 mA.
        self.flood_light_current = (
            None if flood_current_mA is None else min(1.0, flood_current_mA / 1500))

        self.video_encoder = get_video_encoder(config.get('video_encoder', 'mjpeg'))
        self.video_encoder_h264_bitrate = config.get('h264_bitrate', 0)  # 0 = automatic

        color_orientation = config.get("color_orientation", "AUTO")
        assert color_orientation in ["AUTO", "HORIZONTAL_MIRROR", "NORMAL", "ROTATE_180_DEG",
                                     "VERTICAL_FLIP"], color_orientation
        self.color_orientation = getattr(dai.CameraImageOrientation, color_orientation)

        self.color_manual_focus = config.get("color_manual_focus")  # 0..255 [far..near]
        self.color_manual_exposure = config.get("color_manual_exposure")  # [exposure, iso] 1..33000 [us] and 100..1600
        self.color_exposure_compensation = config.get("color_exposure_compensation")  # auto exposure compensation (-9..9)
        self.color_manual_wb = config.get("color_manual_wb")  # 1000..12000 K
        self.stereo_manual_exposure = config.get(
            "stereo_manual_exposure")  # [exposure, iso] 1..33000 [us] and 100..1600
        self.stereo_manual_wb = config.get("stereo_manual_wb")  # 1000..12000 K

        depth_profile_value = config.get("depth_profile")  # PR-#1049 had "ROBOTICS"
        # Available profiles: FAST_ACCURACY, FAST_DENSITY, DEFAULT, FACE, HIGH_DETAIL, ROBOTICS, DENSITY, ACCURACY.
        # https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/#Configuring%20Stereo%20Depth
        if depth_profile_value is not None:
            self.depth_profile = getattr(dai.node.StereoDepth.PresetMode, depth_profile_value)
        else:
            self.depth_profile = None
        self.is_extended_disparity = config.get("stereo_extended_disparity")
        self.is_subpixel = config.get("stereo_subpixel")
        self.is_left_right_check = config.get("stereo_left_right_check")
        median_filter_value = config.get("stereo_median_filter")
        if median_filter_value is not None:
            assert median_filter_value in ["KERNEL_7x7", "KERNEL_5x5", "KERNEL_3x3", "MEDIAN_OFF"], median_filter_value
            self.median_filter = getattr(dai.MedianFilter, median_filter_value)
        else:
            self.median_filter = None

        # explicit confidence threshold (0..255, lower = more accurate/less fill);
        # overrides whatever the depth_profile preset set, if both are given
        self.confidence_threshold = config.get("stereo_confidence_threshold")
        if self.confidence_threshold is not None:
            assert 0 <= self.confidence_threshold <= 255, self.confidence_threshold

        # speckle filter - removes small blobs of noisy disparity
        self.speckle_filter = config.get("stereo_speckle_filter")  # True/False
        self.speckle_range = config.get("stereo_speckle_range", 50)

        # spatial filter - edge-preserving smoothing + hole fill
        self.spatial_filter = config.get("stereo_spatial_filter")  # True/False
        self.spatial_filter_alpha = config.get("stereo_spatial_filter_alpha", 0.5)
        self.spatial_filter_delta = config.get("stereo_spatial_filter_delta", 8)
        self.spatial_filter_hole_filling_radius = config.get("stereo_spatial_filter_hole_filling_radius", 2)
        self.spatial_filter_num_iterations = config.get("stereo_spatial_filter_num_iterations", 1)

        # temporal filter - persistency across frames; CAUTION: intended for
        # static scenes, can smear/blur depth for a moving robot - off by default
        self.temporal_filter = config.get("stereo_temporal_filter")  # True/False

        # brightness filter - invalidate over/under-exposed pixels (helps with
        # rectification-edge artifacts); defaults of 0/255 filter nothing, tune
        # to your actual mono exposure histogram
        self.brightness_filter = config.get("stereo_brightness_filter")  # True/False
        self.brightness_filter_min = config.get("stereo_brightness_filter_min", 0)
        self.brightness_filter_max = config.get("stereo_brightness_filter_max", 255)

        # threshold filter - clip to the robot's actually-relevant depth range, in mm
        self.threshold_filter_min_mm = config.get("stereo_threshold_filter_min_mm")
        self.threshold_filter_max_mm = config.get("stereo_threshold_filter_max_mm")

        # SIPP (Signal Image Processing Pipeline) memory pool - shared on-chip
        # buffer used by ISP, mono-camera Warp/rectification, AND the stereo
        # median filter. Bump this if you hit "'Median' out of system
        # resources" / "Invalid StereoDepth config, error 525" - that error is
        # this pool running out, not something wrong with the filter itself.
        # Per Luxonis docs: 0 moves the pool from the (tiny) on-chip CMX
        # memory into DDR instead, at a fixed 256KB - usually the easiest fix
        # since CMX is small and shared with everything else on the chip.
        self.sipp_buffer_size = config.get("sipp_buffer_size")
        self.sipp_dma_buffer_size = config.get("sipp_dma_buffer_size")

        # Shaves + CMX memory slices reserved specifically for the StereoDepth
        # postprocessing chain (median/speckle/spatial/etc). This is a SEPARATE
        # resource pool from sipp_buffer_size above (that one is shared with
        # ISP/Warp too). Per Luxonis docs, THIS is the documented fix for
        # "'Median' out of system resources" / shave-allocation OOM errors when
        # enabling more than one postprocessing filter at once:
        # https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth
        #   "If the pipeline complains about shave/memory allocation, try
        #    increasing the HW resources used in postprocessing with
        #    setPostProcessingHardwareResources(n_shaves, n_cmx)."
        # Start with (3, 3) and increase if the error persists; note this
        # competes for the same limited on-chip shaves/CMX as the NN model
        # running on the color camera, so raising it may require lowering
        # numShaves on the NN side too.
        self.postprocessing_shaves = config.get("stereo_postprocessing_shaves")
        self.postprocessing_cmx = config.get("stereo_postprocessing_cmx")

        self.set_rectify_edge_fill_color = config.get("set_rectify_edge_fill_color")  # 0 for black color
        self.enable_distortion_correction = config.get("enable_distortion_correction")  # True or False
        self.set_left_right_check_threshold = config.get("set_left_right_check_threshold")
        self.color_depth_alignment = config.get("color_depth_alignment", False)

        self.is_imu_enabled = config.get('is_imu_enabled', False)
        # Preferred number of IMU records in one packet
        self.number_imu_records = config.get('number_imu_records', 20)
        self.disable_magnetometer_fusion = config.get('disable_magnetometer_fusion', False)
        self.is_visual_odom = config.get('is_visual_odom', False)
        self.is_slam = config.get('is_slam', False)
        if self.is_slam:
            assert self.is_visual_odom, "Visual odometry is not allowed."

        if self.is_slam or self.is_visual_odom:
            assert self.is_imu_enabled, "IMU is required for SLAM and visual odometry."
            assert self.is_depth, "Depth is required for SLAM and visual odometry."

        # QR code reading - decoded on host via OpenCV (offline, no NN needed),
        # from a dedicated raw (uncompressed) output off the color camera.
        self.is_qr_detection = config.get('is_qr_detection', False)
        qr_resolution_value = config.get('qr_resolution', 'THE_800_P')
        self.qr_resolution = (g_resolution_dic[qr_resolution_value] if isinstance(qr_resolution_value, str)
                              else tuple(qr_resolution_value))
        assert not self.is_qr_detection or self.is_color, 'is_qr_detection requires is_color'

        self.sleep_on_start_sec = config.get('sleep_on_start_sec')
        self.verbose_detections = config.get('verbose_detections', True)

        # NN setup variables - support for multiple models
        self.models = config.get("models", [])

        # Backwards compatibility for single-model configs
        if "model" in config and len(self.models) == 0:
            self.models.append({
                "model": config.get("model"),
                "nn_config": config.get("nn_config", {}),
                "mappings": config.get("mappings", {})
            })

    def start(self):
        if self.sleep_on_start_sec is not None:
            print(f'sleeping for {self.sleep_on_start_sec}s')
            self.bus.sleep(self.sleep_on_start_sec)
            print(f'END of sleep, starting ...')
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):

        class VideoPublisher(dai.node.HostNode):
            def __init__(self, *args, **kwargs):
                dai.node.HostNode.__init__(self, *args, **kwargs)
                self.bus = None
                self.stream_type = None
                self.subsample = None

            def build(self, *args):
                self.link_args(*args)
                return self

            def process(self, frame):
                seq_num = frame.getSequenceNum()  # for sync of various outputs
                if self.subsample and seq_num % self.subsample != 0:
                    return
                dt = frame.getTimestamp()  # datetime.timedelta
                timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                self.bus.publish(f"{self.stream_type}_seq", [seq_num, timestamp_us])
                self.bus.publish(self.stream_type, frame.getData().tobytes())

        if self.cam_id:
            if cam_is_available(self.cam_id):
                device_info = dai.DeviceInfo(self.cam_id)
                target_device = dai.Device(device_info)
                pipeline_ctx = dai.Pipeline(target_device)
            else:
                self.request_stop()
                return
        else:
            pipeline_ctx = dai.Pipeline()

        with pipeline_ctx as pipeline:
            if self.sipp_buffer_size is not None:
                pipeline.setSippBufferSize(self.sipp_buffer_size)
            if self.sipp_dma_buffer_size is not None:
                pipeline.setSippDmaBufferSize(self.sipp_dma_buffer_size)

            # Define source and output
            if self.is_color:
                cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                cam_rgb.setImageOrientation(self.color_orientation)
                if self.color_manual_focus is not None:
                    cam_rgb.initialControl.setManualFocus(self.color_manual_focus)
                if self.color_manual_exposure is not None:
                    exposure, iso = self.color_manual_exposure
                    cam_rgb.initialControl.setManualExposure(exposure, iso)
                if self.color_exposure_compensation is not None:
                    cam_rgb.initialControl.setAutoExposureCompensation(self.color_exposure_compensation)
                if self.color_manual_wb is not None:
                    cam_rgb.initialControl.setManualWhiteBalance(self.color_manual_wb)

                # should frameRate = fps?? should this be limited on input?
                output = cam_rgb.requestOutput(self.color_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
                encoded = pipeline.create(dai.node.VideoEncoder).build(output,
                                                                       frameRate=self.fps,
                                                                       profile=self.video_encoder)
                saver = pipeline.create(VideoPublisher).build(encoded.out)
                saver.bus = self.bus
                saver.stream_type = "color"
                saver.subsample = self.subsample

                if self.is_qr_detection:
                    # separate raw (uncompressed) output - decoding a QR code from
                    # h265-compressed frames is unreliable, compression artifacts
                    # blur the fine modules
                    qr_output = cam_rgb.requestOutput(self.qr_resolution, type=dai.ImgFrame.Type.BGR888p, fps=self.fps)
                    qr_queue = qr_output.createOutputQueue(blocking=False)

            if self.is_depth or self.is_stereo_images:
                mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

                if self.stereo_manual_exposure is not None:
                    exposure, iso = self.stereo_manual_exposure
                    mono_left.initialControl.setManualExposure(exposure, iso)  # exposure time and ISO
                    mono_right.initialControl.setManualExposure(exposure, iso)  # exposure time and ISO

                if self.stereo_manual_wb is not None:
                    mono_left.initialControl.setManualWhiteBalance(self.stereo_manual_wb)
                    mono_right.initialControl.setManualWhiteBalance(self.stereo_manual_wb)

                # type=dai.ImgFrame.Type.NV12 ??
                mono_left_out = mono_left.requestOutput(self.mono_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
                mono_right_out = mono_right.requestOutput(self.mono_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
                if self.is_depth:
                    stereo = pipeline.create(dai.node.StereoDepth)
                    mono_left_out.link(stereo.left)
                    mono_right_out.link(stereo.right)
                    depth_queue = stereo.depth.createOutputQueue(blocking=False)

                if self.is_stereo_images:
                    for mono_out, stream_type in zip([mono_left_out, mono_right_out], ['left_im', 'right_im']):
                        mono_encoded = pipeline.create(dai.node.VideoEncoder).build(mono_out,
                                                                                    frameRate=self.fps,
                                                                                    profile=self.video_encoder)
                        mono_saver = pipeline.create(VideoPublisher).build(mono_encoded.out)
                        mono_saver.bus = self.bus
                        mono_saver.stream_type = stream_type
                        mono_saver.subsample = self.subsample

            if self.is_imu_enabled:
                # copy from basalt_vio.py
                imu = pipeline.create(dai.node.IMU)

                if self.is_visual_odom:
                    odom = pipeline.create(dai.node.BasaltVIO)
                    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
                    imu.setBatchReportThreshold(1)
                    imu.setMaxBatchReports(10)
                else:
                    if self.disable_magnetometer_fusion:
                        imu.enableIMUSensor(dai.IMUSensor.GAME_ROTATION_VECTOR, 100)  # without magnetometer
                    else:
                        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
                    imu.setBatchReportThreshold(self.number_imu_records)
                    imu.setMaxBatchReports(20)

            if self.is_visual_odom:
                imu.out.link(odom.imu)
                odom_queue = odom.transform.createOutputQueue(blocking=False)
            elif self.is_imu_enabled:
                imu_queue = imu.out.createOutputQueue(blocking=False)

            if self.is_slam:
                slam = pipeline.create(dai.node.RTABMapSLAM)
                params = {"RGBD/CreateOccupancyGrid": "true",
                          "Grid/3D": "true",
                          "Rtabmap/SaveWMState": "true"}
                slam.setParams(params)

            if self.is_depth:
                if self.depth_profile is not None:
                    stereo.setDefaultProfilePreset(self.depth_profile)
                if self.is_extended_disparity is not None:
                    stereo.setExtendedDisparity(self.is_extended_disparity)
                if self.is_left_right_check is not None:
                    stereo.setLeftRightCheck(self.is_left_right_check)
                if self.is_subpixel is not None:
                    stereo.setSubpixel(self.is_subpixel)
                if self.median_filter is not None:
                    stereo.initialConfig.setMedianFilter(self.median_filter)
                if self.confidence_threshold is not None:
                    stereo.initialConfig.setConfidenceThreshold(self.confidence_threshold)
                if self.speckle_filter is not None:
                    stereo.initialConfig.postProcessing.speckleFilter.enable = self.speckle_filter
                    stereo.initialConfig.postProcessing.speckleFilter.speckleRange = self.speckle_range
                if self.spatial_filter is not None:
                    stereo.initialConfig.postProcessing.spatialFilter.enable = self.spatial_filter
                    stereo.initialConfig.postProcessing.spatialFilter.alpha = self.spatial_filter_alpha
                    stereo.initialConfig.postProcessing.spatialFilter.delta = self.spatial_filter_delta
                    stereo.initialConfig.postProcessing.spatialFilter.holeFillingRadius = self.spatial_filter_hole_filling_radius
                    stereo.initialConfig.postProcessing.spatialFilter.numIterations = self.spatial_filter_num_iterations
                if self.temporal_filter is not None:
                    stereo.initialConfig.postProcessing.temporalFilter.enable = self.temporal_filter
                if self.threshold_filter_min_mm is not None:
                    stereo.initialConfig.postProcessing.thresholdFilter.minRange = self.threshold_filter_min_mm
                if self.threshold_filter_max_mm is not None:
                    stereo.initialConfig.postProcessing.thresholdFilter.maxRange = self.threshold_filter_max_mm
                if self.brightness_filter:
                    # Note: BrightnessFilter has no separate enable flag - it's
                    # always "active", so leaving min/max at their 0/255
                    # defaults is what makes it a no-op. Setting real values
                    # here is what "enables" it in practice.
                    stereo.initialConfig.postProcessing.brightnessFilter.minBrightness = self.brightness_filter_min
                    stereo.initialConfig.postProcessing.brightnessFilter.maxBrightness = self.brightness_filter_max
                if self.postprocessing_shaves is not None and self.postprocessing_cmx is not None:
                    stereo.setPostProcessingHardwareResources(self.postprocessing_shaves, self.postprocessing_cmx)
                if self.set_rectify_edge_fill_color is not None:
                    stereo.setRectifyEdgeFillColor(self.set_rectify_edge_fill_color)  # was 0
                if self.enable_distortion_correction is not None:
                    stereo.enableDistortionCorrection(self.enable_distortion_correction)  # was True
                if self.set_left_right_check_threshold is not None:
                    stereo.initialConfig.setLeftRightCheckThreshold(self.set_left_right_check_threshold)  # was 10
                if self.color_depth_alignment:
                    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # RGB camera
                else:
                    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)  # Left camera, default v3.x

                if self.is_visual_odom:
                    stereo.syncedLeft.link(odom.left)
                    stereo.syncedRight.link(odom.right)

            if self.is_slam:
                stereo.depth.link(slam.depth)
                stereo.rectifiedLeft.link(slam.rect)
                odom.transform.link(slam.odom)
                gridmap_queue = slam.occupancyGridMap.createOutputQueue(blocking=False)

            # Configure Neural Network processing blocks dynamically
            nn_queues = []
            for model_cfg in self.models:
                nn_path = model_cfg.get("model", {}).get("blob")
                if not nn_path:
                    continue
                assert Path(nn_path).exists(), "No blob found at '{}'!".format(nn_path)
                # optionally limit number of shaves for "superblob NN archives"
                num_shaves = model_cfg.get("model", {}).get("num_shaves")

                nn_config = model_cfg.get("nn_config", {})
                nn_family = nn_config.get("NN_family", "YOLO")

                input_size_str = nn_config.get("input_size")
                if input_size_str:
                    W, H = tuple(map(int, input_size_str.split('x')))
                else:
                    W, H = 416, 416  # default fallback

                is_nn_archive = nn_path.endswith(".tar.xz")
                if is_nn_archive:
                    # superblob with config
                    # detectionNetwork - NN_ARCHIVE_PATH
                    nn = pipeline.create(dai.node.DetectionNetwork)
                    if num_shaves is None:
                        # unfortunately the DepthAI API does not support numShaves=None
                        nn.setNNArchive(dai.NNArchive(nn_path))
                    else:
                        nn.setNNArchive(dai.NNArchive(nn_path), numShaves=num_shaves)
                else:
                    nn = pipeline.create(dai.node.NeuralNetwork)
                    nn.setBlobPath(nn_path)
                    nn.setNumInferenceThreads(2)
                nn.input.setBlocking(False)

                if nn_family == 'YOLO' and not nn_path.endswith(".tar.xz"):  # no overload for NN archives (yet)
                    metadata = nn_config.get("NN_specific_metadata", {})
                    parser = pipeline.create(dai.node.DetectionParser)
                    if "confidence_threshold" in metadata:
                        parser.setConfidenceThreshold(metadata["confidence_threshold"])
                    if "classes" in metadata:
                        parser.setNumClasses(metadata["classes"])
                    if "coordinates" in metadata:
                        parser.setCoordinateSize(metadata["coordinates"])
                    if "anchors" in metadata:
                        parser.setAnchors(metadata["anchors"])
                    if "anchor_masks" in metadata:
                        parser.setAnchorMasks(metadata["anchor_masks"])
                    if "iou_threshold" in metadata:
                        parser.setIouThreshold(metadata["iou_threshold"])

                    nn.out.link(parser.input)
                    queue = parser.out.createOutputQueue(blocking=False)
                else:
                    queue = nn.out.createOutputQueue(blocking=False)

                # Feed from color camera via specific requested output resolution per model
                if self.is_color:
                    nn_cam_out = cam_rgb.requestOutput((W, H), type=dai.ImgFrame.Type.BGR888p)
                else:
                    assert self.is_stereo_images
                    nn_cam_out = mono_left.requestOutput((W, H), type=dai.ImgFrame.Type.BGR888p)
                # make sure that there is no delay and only the latest image is processed
                nn.input.setMaxSize(1)
                nn.input.setBlocking(False)
                nn_cam_out.link(nn.input)

                nn_queues.append({
                    "queue": queue,
                    "family": nn_family,
                    "image_size": (W, H),
                    "labels": model_cfg.get("mappings", {}).get("labels", [])
                })

            # Connect to device and start pipeline
            pipeline.start()
            if self.laser_projector_current is not None:
                pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(self.laser_projector_current)
            if self.flood_light_current is not None:
                pipeline.getDefaultDevice().setIrFloodLightIntensity(self.flood_light_current)

            qr_detector = cv2.QRCodeDetector() if self.is_qr_detection else None
            last_qr_text = None  # dedup - a code sitting in view decodes on every frame

            while pipeline.isRunning() and self.bus.is_alive():
                processed_any = False

                # 1. Check Neural Networks
                for nn_setup in nn_queues:
                    queue = nn_setup["queue"]
                    nn_family = nn_setup["family"]
                    W, H = nn_setup["image_size"]
                    labels = nn_setup["labels"]

                    nn_packets = queue.tryGetAll()
                    if nn_packets and len(nn_packets) > 0:
                        processed_any = True
                        packet = nn_packets[-1]  # use latest packet
                        seq_num = packet.getSequenceNum()
                        dt = packet.getTimestamp()
                        timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)

                        if nn_family == 'YOLO':
                            detections = packet.detections
                            bbox_list = []
                            for detection in detections:
                                bbox = (detection.xmin, detection.ymin, detection.xmax, detection.ymax)
                                if self.verbose_detections:
                                    print(labels[detection.label], detection.confidence, bbox)
                                bbox_list.append([labels[detection.label], detection.confidence, list(bbox)])
                            self.bus.publish("detections_seq", [seq_num, timestamp_us])
                            self.bus.publish('detections', bbox_list)

                        elif nn_family == 'resnet':
                            nn_output = packet.getLayerFp16('output')
                            mask = np.array(nn_output).reshape((2, H, W))
                            mask = mask.argmax(0).astype(np.uint8)
                            self.bus.publish('nn_mask', mask)

                        elif nn_family == 'robotourist' or nn_family == 'redroad':
                            # v3: getTensor automatically returns a NumPy array
                            nn_output = packet.getTensor('redroad_output')
                            nn_robotourist_output = packet.getTensor('robotourist_output')

                            redroad = np.array(nn_output).reshape((H//2, W//2))
                            self.bus.publish('redroad', redroad)
                            mask = (redroad > 0).astype(np.uint8)
                            mask = np.array(mask).reshape((H//2, W//2))
                            self.bus.publish('nn_mask', mask)

                            # ver0 (1280, 7, 7), ver1 (160, 7, 7)
                            robotourist = np.array(nn_robotourist_output).reshape((len(nn_robotourist_output), 7, 7))
                            self.bus.publish('robotourist', robotourist)


                # 2. Check Depth
                if self.is_depth:
                    depth_frames = depth_queue.tryGetAll()
                    if depth_frames and len(depth_frames) > 0:
                        processed_any = True
                        depth_frame = depth_frames[-1]
                        seq_num = depth_frame.getSequenceNum()
                        if self.subsample and seq_num % self.subsample != 0:
                            continue
                        dt = depth_frame.getTimestamp()
                        timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                        self.bus.publish("depth_seq", [seq_num, timestamp_us])
                        frame = depth_frame.getCvFrame()
                        frame_cp = frame.copy()
                        upper = frame_cp[:280]
                        upper[upper == 0] = 15000
                        self.bus.publish("depth", frame_cp)

                # 3. Check Visual Odom
                if self.is_visual_odom:
                    odom_frames = odom_queue.tryGetAll()
                    if odom_frames and len(odom_frames) > 0:
                        processed_any = True
                        odom_frame = odom_frames[-1]
                        t = odom_frame.getTranslation()
                        q = odom_frame.getQuaternion()
                        self.bus.publish("pose3d", [[t.x, t.y, t.z], [q.qx, q.qy, q.qz, q.qw]])

                # 4. Check SLAM
                if self.is_slam:
                    gridmaps = gridmap_queue.tryGetAll()
                    if gridmaps and len(gridmaps) > 0:
                        processed_any = True
                        gridmap = gridmaps[-1]
                        self.bus.publish('gridmap', gridmap.getFrame())

                # 5. Check IMU
                if self.is_imu_enabled and not self.is_visual_odom:
                    imu_packets = imu_queue.tryGetAll()
                    if imu_packets and len(imu_packets) > 0:
                        processed_any = True
                        for packet in imu_packets:
                            quaternions = [[data.rotationVector.getTimestampDevice().total_seconds(),
                                            data.rotationVector.rotationVectorAccuracy,
                                            data.rotationVector.i, data.rotationVector.j,
                                            data.rotationVector.k, data.rotationVector.real]
                                           for data in packet.packets]
                            self.bus.publish("orientation_list", quaternions)

                # 6. Check QR code
                if self.is_qr_detection:
                    qr_frames = qr_queue.tryGetAll()
                    if qr_frames and len(qr_frames) > 0:
                        processed_any = True
                        frame = qr_frames[-1].getCvFrame()
                        text, points, _ = qr_detector.detectAndDecode(frame)
                        if text:
                            if text != last_qr_text:
                                # a code sitting in view decodes fresh every
                                # frame (fps times/sec) - only publish on
                                # actual change, not every re-decode of the
                                # same still-visible code
                                print('QR code decoded:', text)
                                self.bus.publish('qr_code', text)
                                last_qr_text = text
                        else:
                            last_qr_text = None  # code left view - a later re-appearance re-announces

                # Only rest the CPU if no frames were pulled in this tick loop
                if not processed_any:
                    self.bus.sleep(0.01)

    def request_stop(self):
        self.bus.shutdown()
