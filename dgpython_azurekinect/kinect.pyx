# distutils: language = c++

import sys
import numbers
import collections
import time
import traceback
import ctypes
import atexit
from threading import Thread

import spatialnde2 as snde

from libcpp cimport bool as bool_t
from libcpp.string cimport string
from cython.operator cimport dereference as deref
from cpython cimport Py_DECREF,PyObject

from dataguzzler_python.dgpy import Module as dgpy_Module
from dataguzzler_python.dgpy import CurContext
from dataguzzler_python.dgpy import RunInContext
from dataguzzler_python.dgpy import InitCompatibleThread
from dataguzzler_python.dynamic_metadata import DynamicMetadata

## OBSOLETE: Very important to use dataguzzler_python condition variables
## NOT native python condition variables as the dataguzzler_python
## version avoids cross-module deadlocks. 
##from dataguzzler_python.lock import Condition
from threading import Condition,Lock

#import pint # units library

from libc.stdio cimport fprintf,stderr

from libc.stdint cimport uint64_t
from libc.stdint cimport int64_t
from libc.stdint cimport int32_t
from libc.stdint cimport uint32_t
from libc.stdint cimport int16_t
from libc.stdint cimport uint16_t
from libc.stdint cimport int8_t
from libc.stdint cimport uint8_t
from libc.stdint cimport uintptr_t
from libc.errno cimport errno,EAGAIN,EINTR
from libc.stdlib cimport malloc,calloc,free
from libc.string cimport memcpy

import numpy as np
cimport numpy as np
from numpy cimport NPY_SHORT,PyArray_New,import_array,npy_intp


orient_dtype = [('offset', '<f4', (4,)), ('quat', '<f4', (4,))]

import_array()

cdef extern from "k4a/k4a.h" nogil:
    ctypedef void *k4a_device_t
    ctypedef void *k4a_capture_t
    ctypedef void *k4a_image_t
    ctypedef void *k4a_transformation_t
    
    ctypedef enum k4a_result_t:
        K4A_RESULT_SUCCEEDED, # =0
        K4A_RESULT_FAILED
        pass

    ctypedef enum k4a_buffer_result_t:
        K4A_BUFFER_RESULT_SUCCEEDED, # = 0, /**< The result was successful */
        K4A_BUFFER_RESULT_FAILED,    #    /**< The result was a failure */
        K4A_BUFFER_RESULT_TOO_SMALL #    /**< The input buffer was too small */
        pass

    ctypedef enum k4a_wait_result_t:
        K4A_WAIT_RESULT_SUCCEEDED, # = 0, /**< The result was successful */
        K4A_WAIT_RESULT_FAILED,    #    /**< The result was a failure */
        K4A_WAIT_RESULT_TIMEOUT,   #    /**< The operation timed out */
        pass
    
    ctypedef enum k4a_log_level_t:
        K4A_LOG_LEVEL_CRITICAL
        K4A_LOG_LEVEL_ERROR,
        K4A_LOG_LEVEL_WARNING,
        K4A_LOG_LEVEL_INFO,
        K4A_LOG_LEVEL_TRACE,
        K4A_LOG_LEVEL_OFF

    ctypedef enum k4a_depth_mode_t:
        K4A_DEPTH_MODE_OFF,
        K4A_DEPTH_MODE_NFOV_2X2BINNED, 
        K4A_DEPTH_MODE_NFOV_UNBINNED,  
        K4A_DEPTH_MODE_WFOV_2X2BINNED, 
        K4A_DEPTH_MODE_WFOV_UNBINNED, 
        K4A_DEPTH_MODE_PASSIVE_IR

    ctypedef enum k4a_color_resolution_t:
        K4A_COLOR_RESOLUTION_OFF,
        K4A_COLOR_RESOLUTION_720P,
        K4A_COLOR_RESOLUTION_1080P, #   /**< 1920 * 1080 16:9 */
        K4A_COLOR_RESOLUTION_1440P, #   /**< 2560 * 1440 16:9 */
        K4A_COLOR_RESOLUTION_1536P, #  /**< 2048 * 1536 4:3  */
        K4A_COLOR_RESOLUTION_2160P, #  /**< 3840 * 2160 16:9 */
        K4A_COLOR_RESOLUTION_3072P #  /**< 4096 * 3072 4:3  */
        

    ctypedef enum k4a_image_format_t:
        K4A_IMAGE_FORMAT_COLOR_MJPG,
        K4A_IMAGE_FORMAT_COLOR_NV12,
        K4A_IMAGE_FORMAT_COLOR_YUY2,
        K4A_IMAGE_FORMAT_COLOR_BGRA32,
        K4A_IMAGE_FORMAT_DEPTH16,
        K4A_IMAGE_FORMAT_IR16,
        K4A_IMAGE_FORMAT_CUSTOM8,
        K4A_IMAGE_FORMAT_CUSTOM16,
        K4A_IMAGE_FORMAT_CUSTOM
    ctypedef enum k4a_transformation_interpolation_type_t:
        K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
        K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR

    ctypedef enum k4a_fps_t:
        K4A_FRAMES_PER_SECOND_5, # = 0, /**< 5 FPS */
        K4A_FRAMES_PER_SECOND_15,#    /**< 15 FPS */
        K4A_FRAMES_PER_SECOND_30 #    /**< 30 FPS */

    ctypedef enum k4a_color_control_command_t:
        K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, # = 0,
        K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY,
        K4A_COLOR_CONTROL_BRIGHTNESS,
        K4A_COLOR_CONTROL_CONTRAST,
        K4A_COLOR_CONTROL_SATURATION,
        K4A_COLOR_CONTROL_SHARPNESS,
        K4A_COLOR_CONTROL_WHITEBALANCE,
        K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION,
        K4A_COLOR_CONTROL_GAIN,
        K4A_COLOR_CONTROL_POWERLINE_FREQUENCY
        

    ctypedef enum k4a_color_control_mode_t:
        K4A_COLOR_CONTROL_MODE_AUTO, # = 0, /**< set the associated k4a_color_control_command_t to auto*/
        K4A_COLOR_CONTROL_MODE_MANUAL #   /**< set the associated k4a_color_control_command_t to manual*/

    ctypedef enum k4a_wired_sync_mode_t:
        K4A_WIRED_SYNC_MODE_STANDALONE, 
        K4A_WIRED_SYNC_MODE_MASTER,  
        K4A_WIRED_SYNC_MODE_SUBORDINATE
        
    ctypedef enum k4a_calibration_type_t:
        K4A_CALIBRATION_TYPE_UNKNOWN, # = -1, /**< Calibration type is unknown */
        K4A_CALIBRATION_TYPE_DEPTH,   #     /**< Depth sensor */
        K4A_CALIBRATION_TYPE_COLOR,   #     /**< Color sensor */
        K4A_CALIBRATION_TYPE_GYRO,    #     /**< Gyroscope sensor */
        K4A_CALIBRATION_TYPE_ACCEL,   #     /**< Accelerometer sensor */
        K4A_CALIBRATION_TYPE_NUM     #     /**< Number of types excluding unknown type*/

    ctypedef enum k4a_calibration_model_type_t:
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN, # = 0, /**< Calibration model is unknown */
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_THETA, # /**< Deprecated (not supported). Calibration model is Theta (arctan).
        
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_POLYNOMIAL_3K, #/**< Deprecated (not supported). Calibration model is   Polynomial 3K. */
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT, # /**< Deprecated (only supported early internal devices) Calibration model is Rational 6KT. */
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY #/**< Calibration model is Brown Conrady (compatible with OpenCV )

    ctypedef enum k4a_firmware_build_t:
        K4A_FIRMWARE_BUILD_RELEASE,
        K4A_FIRMWARE_BUILD_DEBUG
        pass

    ctypedef enum k4a_firmware_signature_t:
        K4A_FIRMWARE_SIGNATURE_MSFT,
        K4A_FIRMWARE_SIGNATURE_TEST,
        K4A_FIRMWARE_SIGNATURE_UNSIGNED
        pass
    
    ctypedef void(*k4a_logging_message_cb_t)(void *context, k4a_log_level_t level, const char *file, const int line, const char *message)
    ctypedef void(*k4a_memory_destroy_cb_t)(void *buffer, void *context);
    
    ctypedef uint8_t *(*k4a_memory_allocate_cb_t)(int size, void **context)


    ctypedef struct k4a_device_configuration_t:
        k4a_image_format_t color_format
        k4a_color_resolution_t color_resolution
        k4a_depth_mode_t depth_mode
        k4a_fps_t camera_fps
        bool_t synchronized_images_only
        int32_t depth_delay_off_color_usec
        k4a_wired_sync_mode_t wired_sync_mode
        uint32_t subordinate_delay_off_master_usec
        bool_t disable_streaming_indicator
        pass
    
    extern int K4A_DEVICE_DEFAULT # 0
    extern int K4A_WAIT_INFINITE # -1
        
    extern k4a_device_configuration_t K4A_DEVICE_CONFIG_INIT_DISABLE_ALL

    ctypedef struct k4a_calibration_extrinsics_t:
        float rotation[9]
        float translation[3]
        pass

    cdef struct _param:
        float cx
        float cy
        float fx
        float fy
        float k1
        float k2
        float k3
        float k4
        float k5
        float k6
        float codx
        float cody
        float p2
        float p1
        float metric_radius
        pass
    
    ctypedef union k4a_calibration_intrinsic_parameters_t:
        _param param
        float v[15]
        pass

    ctypedef struct k4a_calibration_intrinsics_t:
        k4a_calibration_model_type_t type #                 /**< Type of calibration model used*/
        unsigned int parameter_count #                      /**< Number of valid entries in parameters*/
        k4a_calibration_intrinsic_parameters_t parameters # /**< Calibration parameters*/
        pass

    ctypedef struct k4a_calibration_camera_t:
        k4a_calibration_extrinsics_t extrinsics # /**< Extrinsic calibration data. */
        k4a_calibration_intrinsics_t intrinsics # /**< Intrinsic calibration data. */
        int resolution_width #                    /**< Resolution width of the calibration sensor. */
        int resolution_height #                   /**< Resolution height of the calibration sensor. */
        float metric_radius #                     /**< Max FOV of the camera. */
        pass

    ctypedef struct k4a_calibration_t:
        k4a_calibration_camera_t depth_camera_calibration # /**< Depth camera calibration. */

        k4a_calibration_camera_t color_camera_calibration # /**< Color camera calibration. */

        k4a_calibration_extrinsics_t extrinsics[<unsigned>K4A_CALIBRATION_TYPE_NUM][<unsigned>K4A_CALIBRATION_TYPE_NUM]

        k4a_depth_mode_t depth_mode #             /**< Depth camera mode for which calibration was obtained. */
        k4a_color_resolution_t color_resolution # /**< Color camera resolution for which calibration was obtained. */
        pass

    ctypedef struct k4a_version_t:
        uint32_t major #     /**< Major version; represents a breaking change. */
        uint32_t minor #     /**< Minor version; represents additional features, no regression from lower versions with same major version. */
        uint32_t iteration #  /**< Reserved. */
        pass

    ctypedef struct k4a_hardware_version_t:
        k4a_version_t rgb #          /**< Color camera firmware version. */
        k4a_version_t depth #        /**< Depth camera firmware version. */
        k4a_version_t audio #        /**< Audio device firmware version. */
        k4a_version_t depth_sensor  # /**< Depth sensor firmware version. */
        
        k4a_firmware_build_t firmware_build #         /**< Build type reported by the firmware. */
        k4a_firmware_signature_t firmware_signature # /**< Signature type of the firmware. */
        pass

    cdef struct _xy:
        float x
        float y
        pass
    
    ctypedef union k4a_float2_t:
        _xy xy
        float v[2]
        pass
    
    cdef struct _xyz:
        float x
        float y
        float z
        pass
    
    ctypedef union k4a_float3_t:
        _xyz xyz
        float v[4]
        pass
    
    ctypedef struct k4a_imu_sample_t:
        # don't expect to need this
        pass
    
    uint32_t k4a_device_get_installed_count()
    k4a_result_t k4a_set_debug_message_handler(k4a_logging_message_cb_t message_cb,void *message_cb_context,k4a_log_level_t min_level)
    k4a_result_t k4a_set_allocator(k4a_memory_allocate_cb_t allocate, k4a_memory_destroy_cb_t free)

    k4a_result_t k4a_device_open(uint32_t index, k4a_device_t *device_handle)
    void k4a_device_close(k4a_device_t device_handle)
    k4a_wait_result_t k4a_device_get_capture(k4a_device_t device_handle,k4a_capture_t *capture_handle, int32_t timeout_in_ms)

    k4a_wait_result_t k4a_device_get_imu_sample(k4a_device_t device_handle,k4a_imu_sample_t *imu_sample,int32_t timeout_in_ms)

    k4a_result_t k4a_capture_create(k4a_capture_t *capture_handle)
    void k4a_capture_release(k4a_capture_t capture_handle)

    void k4a_capture_reference(k4a_capture_t capture_handle)
    k4a_image_t k4a_capture_get_color_image(k4a_capture_t capture_handle)
    k4a_image_t k4a_capture_get_depth_image(k4a_capture_t capture_handle)
    k4a_image_t k4a_capture_get_ir_image(k4a_capture_t capture_handle)
    void k4a_capture_set_color_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
    void k4a_capture_set_depth_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
    void k4a_capture_set_ir_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
    void k4a_capture_set_temperature_c(k4a_capture_t capture_handle, float temperature_c)
    float k4a_capture_get_temperature_c(k4a_capture_t capture_handle)
    k4a_result_t k4a_image_create(k4a_image_format_t format,int width_pixels,int height_pixels,int stride_bytes, k4a_image_t *image_handle)

    k4a_result_t k4a_image_create(k4a_image_format_t format,int width_pixels, int height_pixels, int stride_bytes, k4a_image_t *image_handle);

    k4a_result_t k4a_image_create_from_buffer(k4a_image_format_t format,
                                              int width_pixels,
                                              int height_pixels,
                                              int stride_bytes,
                                              uint8_t *buffer,
                                              size_t buffer_size,
                                              k4a_memory_destroy_cb_t *buffer_release_cb,
                                              void *buffer_release_cb_context,
                                              k4a_image_t *image_handle)
    
    uint8_t *k4a_image_get_buffer(k4a_image_t image_handle)

    size_t k4a_image_get_size(k4a_image_t image_handle)    
    
    k4a_image_format_t k4a_image_get_format(k4a_image_t image_handle)

    int k4a_image_get_width_pixels(k4a_image_t image_handle)
    
    int k4a_image_get_height_pixels(k4a_image_t image_handle)

    int k4a_image_get_stride_bytes(k4a_image_t image_handle)

    uint64_t k4a_image_get_device_timestamp_usec(k4a_image_t image_handle)
    uint64_t k4a_image_get_system_timestamp_nsec(k4a_image_t image_handle)
    uint64_t k4a_image_get_exposure_usec(k4a_image_t image_handle)
    uint32_t k4a_image_get_white_balance(k4a_image_t image_handle)
    uint32_t k4a_image_get_iso_speed(k4a_image_t image_handle)
    void k4a_image_set_device_timestamp_usec(k4a_image_t image_handle, uint64_t timestamp_usec)
    void k4a_image_set_system_timestamp_nsec(k4a_image_t image_handle, uint64_t timestamp_nsec)
    void k4a_image_set_exposure_usec(k4a_image_t image_handle, uint64_t exposure_usec)
    void k4a_image_set_exposure_time_usec(k4a_image_t image_handle, uint64_t exposure_usec)
    void k4a_image_set_white_balance(k4a_image_t image_handle, uint32_t white_balance)
    void k4a_image_set_iso_speed(k4a_image_t image_handle, uint32_t iso_speed)
    void k4a_image_reference(k4a_image_t image_handle)
    void k4a_image_release(k4a_image_t image_handle)
    k4a_result_t k4a_device_start_cameras(k4a_device_t device_handle, const k4a_device_configuration_t *config)

    void k4a_device_stop_cameras(k4a_device_t device_handle)
    k4a_result_t k4a_device_start_imu(k4a_device_t device_handle)
    void k4a_device_stop_imu(k4a_device_t device_handle)

    k4a_buffer_result_t k4a_device_get_serialnum(k4a_device_t device_handle,
                                                 char *serial_number,
                                                 size_t *serial_number_size)
    k4a_result_t k4a_device_get_version(k4a_device_t device_handle, k4a_hardware_version_t *version)

    k4a_result_t k4a_device_get_color_control_capabilities(k4a_device_t device_handle,
                                                           k4a_color_control_command_t command,
                                                           bool_t *supports_auto,
                                                           int32_t *min_value,
                                                           int32_t *max_value,
                                                           int32_t *step_value,
                                                           int32_t *default_value,
                                                           k4a_color_control_mode_t *default_mode)

    k4a_result_t k4a_device_get_color_control(k4a_device_t device_handle,
                                              k4a_color_control_command_t command,
                                              k4a_color_control_mode_t *mode,
                                              int32_t *value)
    
    k4a_result_t k4a_device_set_color_control(k4a_device_t device_handle,
                                              k4a_color_control_command_t command,
                                              k4a_color_control_mode_t mode,
                                              int32_t value)
    
    k4a_buffer_result_t k4a_device_get_raw_calibration(k4a_device_t device_handle,
                                                       uint8_t *data,
                                                       size_t *data_size)

    k4a_result_t k4a_device_get_calibration(k4a_device_t device_handle,
                                            const k4a_depth_mode_t depth_mode,
                                            const k4a_color_resolution_t color_resolution,
                                            k4a_calibration_t *calibration)


    k4a_result_t k4a_device_get_sync_jack(k4a_device_t device_handle,
                                          bool_t *sync_in_jack_connected,
                                          bool_t *sync_out_jack_connected)


    k4a_result_t k4a_calibration_get_from_raw(char *raw_calibration,
                                              size_t raw_calibration_size,
                                              const k4a_depth_mode_t depth_mode,
                                              const k4a_color_resolution_t color_resolution,
                                              k4a_calibration_t *calibration)
    

    k4a_result_t k4a_calibration_3d_to_3d(const k4a_calibration_t *calibration,
                                          const k4a_float3_t *source_point3d_mm,
                                          const k4a_calibration_type_t source_camera,
                                          const k4a_calibration_type_t target_camera,
                                          k4a_float3_t *target_point3d_mm)
    
    k4a_result_t k4a_calibration_2d_to_3d(const k4a_calibration_t *calibration,
                                          const k4a_float2_t *source_point2d,
                                          const float source_depth_mm,
                                          const k4a_calibration_type_t source_camera,
                                          const k4a_calibration_type_t target_camera,
                                          k4a_float3_t *target_point3d_mm,
                                          int *valid)

    k4a_result_t k4a_calibration_3d_to_2d(const k4a_calibration_t *calibration,
                                          const k4a_float3_t *source_point3d_mm,
                                          const k4a_calibration_type_t source_camera,
                                          const k4a_calibration_type_t target_camera,
                                          k4a_float2_t *target_point2d,
                                        int *valid);

    k4a_result_t k4a_calibration_2d_to_2d(const k4a_calibration_t *calibration,
                                          const k4a_float2_t *source_point2d,
                                          const float source_depth_mm,
                                          const k4a_calibration_type_t source_camera,
                                          const k4a_calibration_type_t target_camera,
                                          k4a_float2_t *target_point2d,
                                          int *valid)

    k4a_result_t k4a_calibration_color_2d_to_depth_2d(const k4a_calibration_t *calibration,
                                                      const k4a_float2_t *source_point2d,
                                                      const k4a_image_t depth_image,
                                                      k4a_float2_t *target_point2d,
                                                      int *valid)
    
    k4a_transformation_t k4a_transformation_create(const k4a_calibration_t *calibration)

    void k4a_transformation_destroy(k4a_transformation_t transformation_handle)

    k4a_result_t k4a_transformation_depth_image_to_color_camera(k4a_transformation_t transformation_handle,
                                                                const k4a_image_t depth_image,
                                                                k4a_image_t transformed_depth_image)


    k4a_result_t k4a_transformation_depth_image_to_color_camera_custom(k4a_transformation_t transformation_handle,
                                                                       const k4a_image_t depth_image,
                                                                       const k4a_image_t custom_image,
                                                                       k4a_image_t transformed_depth_image,
                                                                       k4a_image_t transformed_custom_image,
                                                                       k4a_transformation_interpolation_type_t interpolation_type,
                                                                       uint32_t invalid_custom_value)

    k4a_result_t k4a_transformation_color_image_to_depth_camera(k4a_transformation_t transformation_handle,
                                                                const k4a_image_t depth_image,
                                                                const k4a_image_t color_image,
                                                                k4a_image_t transformed_color_image)

    
    k4a_result_t k4a_transformation_depth_image_to_point_cloud(k4a_transformation_t transformation_handle,
                                                               const k4a_image_t depth_image,
                                                               const k4a_calibration_type_t camera,
                                                               k4a_image_t xyz_image)


    pass



cdef extern from "k4arecord/types.h" nogil:
    ctypedef void *k4a_playback_t
    ctypedef void *k4a_playback_data_block_t
    char *K4A_TRACK_NAME_COLOR
    char *K4A_TRACK_NAME_DEPTH
    char *K4A_TRACK_NAME_IR
    char *K4A_TRACK_NAME_IMU
    ctypedef enum k4a_stream_result_t:
        K4A_STREAM_RESULT_SUCCEEDED # = 0  The result was successful. 
        K4A_STREAM_RESULT_FAILED    #      The result was a failure. 
        K4A_STREAM_RESULT_EOF       #      The end of the data stream was reached. 
        pass
    ctypedef enum k4a_playback_seek_origin_t:
        K4A_PLAYBACK_SEEK_BEGIN       # Seek relative to the beginning of a recording.
        K4A_PLAYBACK_SEEK_END         # Seek relative to the end of a recording. 
        K4A_PLAYBACK_SEEK_DEVICE_TIME # Seek to an absolute device timestamp. 
        pass

    ctypedef struct k4a_record_configuration_t:
        #  Image format used to record the color camera.
        k4a_image_format_t color_format
        #  Image resolution used to record the color camera. 
        k4a_color_resolution_t color_resolution
        #  Mode used to record the depth camera. 
        k4a_depth_mode_t depth_mode
        # Frame rate used to record the color and depth camera. 
        k4a_fps_t camera_fps
        # True if the recording contains Color camera frames.
        bool_t color_track_enabled
        # True if the recording contains Depth camera frames.
        bool_t depth_track_enabled
        # True if the recording contains IR camera frames. 
        bool_t ir_track_enabled
        # True if the recording contains IMU sample data.
        bool_t imu_track_enabled
        # The delay between color and depth images in the recording.
        # A negative delay means depth images are first, and a positive delay means color images are first.
        int32_t depth_delay_off_color_usec
        # External synchronization mode
        k4a_wired_sync_mode_t wired_sync_mode
        # The delay between this recording and the externally synced master camera.
        # This value is 0 unless \p wired_sync_mode is set to ::K4A_WIRED_SYNC_MODE_SUBORDINATE
        uint32_t subordinate_delay_off_master_usec
        # The timestamp offset of the start of the recording. All recorded timestamps are offset by this value such that
        # the recording starts at timestamp 0. This value can be used to synchronize timestamps between 2 recording files.
        uint32_t start_timestamp_offset_usec
        pass

    ctypedef struct k4a_record_video_settings_t:
        uint64_t width   # Frame width of the video 
        uint64_t height  # Frame height of the video  
        uint64_t frame_rate # Frame rate (frames-per-second) of the video 
        pass

    ctypedef struct k4a_record_subtitle_settings_t:
        bool_t high_freq_data
        pass

    pass


cdef extern from "k4arecord/playback.h" nogil:
    k4a_result_t k4a_playback_open(const char *path, k4a_playback_t *playback_handle)
    k4a_buffer_result_t k4a_playback_get_raw_calibration(k4a_playback_t playback_handle,uint8_t *data,size_t *data_size)
    k4a_result_t k4a_playback_get_calibration(k4a_playback_t playback_handle,k4a_calibration_t *calibration)
    k4a_result_t k4a_playback_get_record_configuration(k4a_playback_t playback_handle,k4a_record_configuration_t *config)
    bool_t k4a_playback_check_track_exists(k4a_playback_t playback_handle, const char *track_name)
    size_t k4a_playback_get_track_count(k4a_playback_t playback_handle)
    k4a_buffer_result_t k4a_playback_get_track_name(k4a_playback_t playback_handle,size_t track_index,char *track_name,size_t *track_name_size)
    bool_t k4a_playback_track_is_builtin(k4a_playback_t playback_handle, const char *track_name)
    k4a_result_t k4a_playback_track_get_video_settings(k4a_playback_t playback_handle,const char *track_name,k4a_record_video_settings_t *video_settings)
    k4a_buffer_result_t k4a_playback_track_get_codec_id(k4a_playback_t playback_handle,const char *track_name,char *codec_id,size_t *codec_id_size)
    k4a_buffer_result_t k4a_playback_track_get_codec_context(k4a_playback_t playback_handle,const char *track_name,uint8_t *codec_context,size_t *codec_context_size)
    k4a_buffer_result_t k4a_playback_get_tag(k4a_playback_t playback_handle,const char *name,char *value,size_t *value_size)
    k4a_result_t k4a_playback_set_color_conversion(k4a_playback_t playback_handle,k4a_image_format_t target_format)
    k4a_buffer_result_t k4a_playback_get_attachment(k4a_playback_t playback_handle,const char *file_name,uint8_t *data,size_t *data_size)
    k4a_stream_result_t k4a_playback_get_next_capture(k4a_playback_t playback_handle, k4a_capture_t *capture_handle)
    k4a_stream_result_t k4a_playback_get_previous_capture(k4a_playback_t playback_handle,k4a_capture_t *capture_handle)
    k4a_stream_result_t k4a_playback_get_next_imu_sample(k4a_playback_t playback_handle,k4a_imu_sample_t *imu_sample)
    k4a_stream_result_t k4a_playback_get_previous_imu_sample(k4a_playback_t playback_handle,k4a_imu_sample_t *imu_sample)
    k4a_stream_result_t k4a_playback_get_next_data_block(k4a_playback_t playback_handle,const char *track_name,k4a_playback_data_block_t *data_block_handle)
    k4a_stream_result_t k4a_playback_get_previous_data_block(k4a_playback_t playback_handle,const char *track_name,k4a_playback_data_block_t *data_block_handle)
    uint64_t k4a_playback_data_block_get_device_timestamp_usec(k4a_playback_data_block_t data_block_handle)
    size_t k4a_playback_data_block_get_buffer_size(k4a_playback_data_block_t data_block_handle)
    uint8_t *k4a_playback_data_block_get_buffer(k4a_playback_data_block_t data_block_handle)
    void k4a_playback_data_block_release(k4a_playback_data_block_t data_block_handle)
    k4a_result_t k4a_playback_seek_timestamp(k4a_playback_t playback_handle,int64_t offset_usec,k4a_playback_seek_origin_t origin)
    uint64_t k4a_playback_get_recording_length_usec(k4a_playback_t playback_handle)
    void k4a_playback_close(k4a_playback_t playback_handle)
















    



k4a_meters_per_lsb=1.0e-3 # distance per least signficant bit of these various numbers recorded (1mm)

# Note: Once Cython 3.x is widely available it should be possible
# to replace this mapping via scoped enumerations and cpdef enum
# https://cython.readthedocs.io/en/latest/src/userguide/wrapping_CPlusPlus.html#scoped-enumerations
syms = {
    "K4A_RESULT_SUCCEEDED": K4A_RESULT_SUCCEEDED,
    "K4A_RESULT_FAILED": K4A_RESULT_FAILED,
    "K4A_BUFFER_RESULT_SUCCEEDED": K4A_BUFFER_RESULT_SUCCEEDED,
    "K4A_BUFFER_RESULT_FAILED": K4A_BUFFER_RESULT_FAILED,
    "K4A_BUFFER_RESULT_TOO_SMALL": K4A_BUFFER_RESULT_TOO_SMALL,
    "K4A_WAIT_RESULT_SUCCEEDED": K4A_WAIT_RESULT_SUCCEEDED,
    "K4A_WAIT_RESULT_FAILED": K4A_WAIT_RESULT_FAILED,
    "K4A_WAIT_RESULT_TIMEOUT": K4A_WAIT_RESULT_TIMEOUT,
    "K4A_LOG_LEVEL_CRITICAL": K4A_LOG_LEVEL_CRITICAL,
    "K4A_LOG_LEVEL_ERROR": K4A_LOG_LEVEL_ERROR,
    "K4A_LOG_LEVEL_WARNING": K4A_LOG_LEVEL_WARNING,
    "K4A_LOG_LEVEL_INFO": K4A_LOG_LEVEL_INFO,
    "K4A_LOG_LEVEL_TRACE": K4A_LOG_LEVEL_TRACE,
    "K4A_LOG_LEVEL_OFF": K4A_LOG_LEVEL_OFF,
    "K4A_DEPTH_MODE_OFF": K4A_DEPTH_MODE_OFF,
    "K4A_DEPTH_MODE_NFOV_2X2BINNED": K4A_DEPTH_MODE_NFOV_2X2BINNED,
    "K4A_DEPTH_MODE_NFOV_UNBINNED": K4A_DEPTH_MODE_NFOV_UNBINNED,
    "K4A_DEPTH_MODE_WFOV_2X2BINNED": K4A_DEPTH_MODE_WFOV_2X2BINNED,
    "K4A_DEPTH_MODE_WFOV_UNBINNED": K4A_DEPTH_MODE_WFOV_UNBINNED,
    "K4A_DEPTH_MODE_PASSIVE_IR": K4A_DEPTH_MODE_PASSIVE_IR,
    "K4A_COLOR_RESOLUTION_OFF": K4A_COLOR_RESOLUTION_OFF,
    "K4A_COLOR_RESOLUTION_720P": K4A_COLOR_RESOLUTION_720P,
    "K4A_COLOR_RESOLUTION_1080P": K4A_COLOR_RESOLUTION_1080P,
    "K4A_COLOR_RESOLUTION_1440P": K4A_COLOR_RESOLUTION_1440P,
    "K4A_COLOR_RESOLUTION_1536P": K4A_COLOR_RESOLUTION_1536P,
    "K4A_COLOR_RESOLUTION_2160P": K4A_COLOR_RESOLUTION_2160P,
    "K4A_COLOR_RESOLUTION_3072P": K4A_COLOR_RESOLUTION_3072P,
    "K4A_IMAGE_FORMAT_COLOR_MJPG": K4A_IMAGE_FORMAT_COLOR_MJPG,
    "K4A_IMAGE_FORMAT_COLOR_NV12": K4A_IMAGE_FORMAT_COLOR_NV12,
    "K4A_IMAGE_FORMAT_COLOR_YUY2": K4A_IMAGE_FORMAT_COLOR_YUY2,
    "K4A_IMAGE_FORMAT_COLOR_BGRA32": K4A_IMAGE_FORMAT_COLOR_BGRA32,
    "K4A_IMAGE_FORMAT_DEPTH16": K4A_IMAGE_FORMAT_DEPTH16,
    "K4A_IMAGE_FORMAT_IR16": K4A_IMAGE_FORMAT_IR16,
    "K4A_IMAGE_FORMAT_CUSTOM8": K4A_IMAGE_FORMAT_CUSTOM8,
    "K4A_IMAGE_FORMAT_CUSTOM16": K4A_IMAGE_FORMAT_CUSTOM16,
    "K4A_IMAGE_FORMAT_CUSTOM": K4A_IMAGE_FORMAT_CUSTOM,
    "K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST": K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
    "K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR": K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR,
    "K4A_FRAMES_PER_SECOND_5": K4A_FRAMES_PER_SECOND_5,
    "K4A_FRAMES_PER_SECOND_15": K4A_FRAMES_PER_SECOND_15,
    "K4A_FRAMES_PER_SECOND_30": K4A_FRAMES_PER_SECOND_30,
    "K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE": K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
    "K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY": K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY,
    "K4A_COLOR_CONTROL_BRIGHTNESS": K4A_COLOR_CONTROL_BRIGHTNESS,
    "K4A_COLOR_CONTROL_CONTRAST": K4A_COLOR_CONTROL_CONTRAST,
    "K4A_COLOR_CONTROL_SATURATION": K4A_COLOR_CONTROL_SATURATION,
    "K4A_COLOR_CONTROL_SHARPNESS": K4A_COLOR_CONTROL_SHARPNESS,
    "K4A_COLOR_CONTROL_WHITEBALANCE": K4A_COLOR_CONTROL_WHITEBALANCE,
    "K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION": K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION,
    "K4A_COLOR_CONTROL_GAIN": K4A_COLOR_CONTROL_GAIN,
    "K4A_COLOR_CONTROL_POWERLINE_FREQUENCY": K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
    "K4A_COLOR_CONTROL_MODE_AUTO": K4A_COLOR_CONTROL_MODE_AUTO,
    "K4A_COLOR_CONTROL_MODE_MANUAL": K4A_COLOR_CONTROL_MODE_MANUAL,
    "K4A_WIRED_SYNC_MODE_STANDALONE": K4A_WIRED_SYNC_MODE_STANDALONE,
    "K4A_WIRED_SYNC_MODE_MASTER": K4A_WIRED_SYNC_MODE_MASTER,
    "K4A_WIRED_SYNC_MODE_SUBORDINATE": K4A_WIRED_SYNC_MODE_SUBORDINATE,
    "K4A_CALIBRATION_TYPE_UNKNOWN": K4A_CALIBRATION_TYPE_UNKNOWN,
    "K4A_CALIBRATION_TYPE_DEPTH": K4A_CALIBRATION_TYPE_DEPTH,
    "K4A_CALIBRATION_TYPE_COLOR": K4A_CALIBRATION_TYPE_COLOR,
    "K4A_CALIBRATION_TYPE_GYRO": K4A_CALIBRATION_TYPE_GYRO,
    "K4A_CALIBRATION_TYPE_ACCEL": K4A_CALIBRATION_TYPE_ACCEL,
    "K4A_CALIBRATION_TYPE_NUM": K4A_CALIBRATION_TYPE_NUM,
    "K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN": K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN,
    "K4A_CALIBRATION_LENS_DISTORTION_MODEL_THETA": K4A_CALIBRATION_LENS_DISTORTION_MODEL_THETA,
    "K4A_CALIBRATION_LENS_DISTORTION_MODEL_POLYNOMIAL_3K": K4A_CALIBRATION_LENS_DISTORTION_MODEL_POLYNOMIAL_3K,
    "K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT": K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT,
    "K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY": K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY,
    "K4A_FIRMWARE_BUILD_RELEASE": K4A_FIRMWARE_BUILD_RELEASE,
    "K4A_FIRMWARE_BUILD_DEBUG": K4A_FIRMWARE_BUILD_DEBUG,
    "K4A_FIRMWARE_SIGNATURE_MSFT": K4A_FIRMWARE_SIGNATURE_MSFT,
    "K4A_FIRMWARE_SIGNATURE_TEST": K4A_FIRMWARE_SIGNATURE_TEST,
    "K4A_FIRMWARE_SIGNATURE_UNSIGNED": K4A_FIRMWARE_SIGNATURE_UNSIGNED,
    "K4A_DEVICE_DEFAULT": K4A_DEVICE_DEFAULT, # 0
    "K4A_WAIT_INFINITE": K4A_WAIT_INFINITE, # -1        
}

    

def get_device_serial_numbers():
    cdef k4a_device_t dev
    cdef size_t serial_size
    cdef char *serial
    cdef uint32_t num_devices
    cdef uint32_t devicenum

    dev=NULL
    
    num_devices = k4a_device_get_installed_count()
    
    if num_devices < 1:
        raise IOError("No Azure Kinect devices found")

    serial_strs = []
    for devicenum in range(num_devices):
        serial_size = 0
        
        dev=NULL
        k4a_device_open(devicenum,&dev)
        k4a_device_get_serialnum(dev,NULL,&serial_size)
        serial = <char *>calloc(serial_size,1)
        k4a_device_get_serialnum(dev,serial,&serial_size)
        serial_bytes =  <bytes>serial
        serial_str = serial_bytes.decode('utf-8')
        
        serial_strs.append(serial_str)
        
        free(serial)
        k4a_device_close(dev)
        dev=NULL
        pass

    return serial_strs

cdef class K4AAcquisition:
    cdef object serial_number_or_filename
    cdef k4a_capture_t capt
    cdef object monotonic_timestamp
    cdef object os_timestamp

    def __cinit__(self):
        
        self.serial_number_or_filename = None
        self.capt = NULL
        self.monotonic_timestamp = None
        self.os_timestamp = None
        pass

    @staticmethod
    cdef K4AAcquisition create(object serial_number_or_filename,k4a_capture_t capt, object monotonic_timestamp,object os_timestamp):
        cdef K4AAcquisition o = K4AAcquisition()
        o.serial_number_or_filename = serial_number_or_filename
        o.capt = capt
        o.monotonic_timestamp = monotonic_timestamp
        o.os_timestamp = os_timestamp
        #print("type(o)=%s o=%s" % (str(type(o)),str(o)))
        return o
    


    cdef get_depth_data(self,k4a_depth_mode_t depth_mode,k4a_transformation_t transformation,object buffer, int width, int height,int point_cloud_flag,int float_flag):
        """buffer should be of the image width and image
           height, with space for 1 (int16 or float depending on float_flag) per pixel or 3 (int16s or floats) depending on point_cloud_flag"""
        #cdef k4a_depth_mode_t depth_mode = lowlevel.config.depth_mode
        cdef k4a_image_t depth_image=NULL
        cdef k4a_image_t point_cloud_image=NULL
        cdef k4a_result_t errcode
        cdef void *imgbuf;
        cdef uint64_t *longbuf;
        cdef npy_intp strides[3]
        cdef npy_intp dims[3]
        cdef object k4a_array  # numpy array
        cdef size_t nbytes
        
        assert(depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED or 
               depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED or 
               depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED or 
               depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED)
        
        with nogil:
            
            
            depth_image = k4a_capture_get_depth_image(self.capt);
            if depth_image is NULL:
                with gil:
                    raise IOError("k4a_device_get_depth_image failed on device serial number %s" % (self.serial_number))
                pass

            if not(k4a_image_get_width_pixels(depth_image)==width and k4a_image_get_height_pixels(depth_image)==height):
                with gil:
                    raise ValueError("Image dimensions (%d,%d) do not match expected value (%d,%d)" % (k4a_image_get_width_pixels(depth_image),k4a_image_get_height_pixels(depth_image),width,height))
                pass

            #imgbuf = <void*>k4a_image_get_buffer(depth_image);
            #longbuf=<uint64_t *>imgbuf;

            #ctypes.cast(<uintptr_t>imgbuf,ctypes.POINTER(ctypes.c_char))
            dims[0]=width
            dims[1]=height
            strides[0]=2
            strides[1]=k4a_image_get_stride_bytes(depth_image)
            
            if not point_cloud_flag:
                # Image based
                with gil:
                    # Wrap k4a data with a numpy array, then transfer into our buffer
                    k4a_array = PyArray_New(np.ndarray,2,dims,NPY_SHORT,strides,<void *>k4a_image_get_buffer(depth_image),2,0,None)
                    if float_flag:
                        # When we generate output in floating point we generate it in spatialnde2 (OpenGL)
                        # coordinates in meters vs. the native integer output from the Azure Kinect is in
                        # OpenCV coordinates in mm.
                        
                        # Negate z coordinates in the depth image; we will also invert the y
                        # axis interpretation via metadata. 
                        buffer[:,:] = (-k4a_meters_per_lsb)*k4a_array
                        pass
                    else:
                        buffer[:,:] = k4a_array
                        pass
                    #sys.stderr.write("refcount=%d\n" % ((<PyObject *>k4a_array).ob_refcnt))
                    ##Py_DECREF(k4a_array) # prevent memory leak (not necessary as it turns out)
                    del k4a_array
                    pass
                pass
            else:
                # point_cloud

                if float_flag:
                    # Create a temporary integer buffer
                    #strides[0]=3*sizeof(int16)
                    strides[1] = width*3*sizeof(int16_t)
                    nbytes = strides[1]*height
                    imgbuf = malloc(nbytes)
                    errcode = k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_CUSTOM,
                                                           width,
                                                           height,
                                                           strides[1],
                                                           <uint8_t *>imgbuf,
                                                           nbytes,
                                                           NULL,
                                                           NULL,
                                                           &point_cloud_image)
                    if errcode != K4A_RESULT_SUCCEEDED:
                        with gil:
                            raise RuntimeError("Error creating temporary point cloud image buffer")
                        pass
                    
                    
                    errcode = k4a_transformation_depth_image_to_point_cloud(transformation,depth_image,K4A_CALIBRATION_TYPE_DEPTH,point_cloud_image)
                    if errcode != K4A_RESULT_SUCCEEDED:
                        with gil:
                            raise RuntimeError("Error transforming depth image into point cloud")
                        pass

                    
                    # Wrap k4a data with a numpy array, then transfer into our buffer
                    dims[0]=3
                    dims[1]=width
                    dims[2]=height
                    strides[0]=2
                    strides[1]=3*sizeof(int16_t)
                    strides[2]=k4a_image_get_stride_bytes(point_cloud_image)
                    with gil:
                        k4a_array = PyArray_New(np.ndarray,3,dims,NPY_SHORT,strides,<void *>k4a_image_get_buffer(point_cloud_image),2,0,None)
                        #if strides[2] != width*3*sizeof(int16_t):
                        #    raise ValueError("A")
                        #if k4a_array.nbytes != nbytes:
                        #    raise ValueError("B")
                        


                        # When we generate output in floating point we generate it in spatialnde2 (OpenGL)
                        # coordinates vs. the native integer output from the Azure Kinect is in
                        # OpenCV coordinates.

                        # Transfer x coordinates unchanged into the point cloud.
                        buffer[0,:,:] = k4a_meters_per_lsb*k4a_array[0,:,:]
                        # Negate y and z coordinates in the point cloud.                        
                        buffer[1:3,:,:] = (-k4a_meters_per_lsb)*k4a_array[1:3,:,:]
                        
                        #Py_DECREF(k4a_array) # prevent memory leak (not necessary as it turns out)
                        del k4a_array
                        pass

                    k4a_image_release(point_cloud_image)
                    free(imgbuf)
                    
                    pass
                else:
                    # Integer mode: Convert directly into given buffer
                    with gil:
                        strides[1]=buffer.strides[1]
                        imgbuf = <void *>buffer.data
                        nbytes = buffer.nbytes
                        pass
                    
                    # Integer mode: Convert directly into the numpy array
                    errcode = k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_CUSTOM,
                                                           width,
                                                           height,
                                                           strides[1],
                                                           <uint8_t *>imgbuf,
                                                           nbytes,
                                                           NULL,
                                                           NULL,
                                                           &point_cloud_image)
                    if errcode != K4A_RESULT_SUCCEEDED:
                        with gil:
                            raise RuntimeError("Error creating point cloud image from buffer")
                        pass
                    
                    errcode = k4a_transformation_depth_image_to_point_cloud(transformation,depth_image,K4A_CALIBRATION_TYPE_DEPTH,point_cloud_image)
                    if errcode != K4A_RESULT_SUCCEEDED:
                        with gil:
                            raise RuntimeError("Error transforming depth image into point cloud")
                        pass
                    
                    k4a_image_release(point_cloud_image)
                    pass
                
                pass
            k4a_image_release(depth_image)
            pass

        pass


    def release_buffers(self):
        if self.capt:
            k4a_capture_release(self.capt)
            self.capt=NULL
            pass
        pass
    
    def __del__(self):
        if self.capt:
            k4a_capture_release(self.capt)
            pass
        self.capt=NULL
        pass

    
cdef class K4ALowLevel:
    cdef k4a_device_t dev
    #cdef object result_chan_ptr # swig-wrapped shared_ptr[channel] 
    cdef object serial_number # Python string
    cdef k4a_calibration_t calibration  # Only valid when capture_running
    cdef k4a_transformation_t transformation
    cdef bool_t capture_running

    cdef k4a_calibration_type_t point_cloud_frame # K4A_CALIBRATION_TYPE_DEPTH or K4A_CALIBRATION_TYPE_COLOR
    
    cdef k4a_device_configuration_t config
    cdef object config_lock

    def __cinit__(self,serial_number_str_or_none):
        #""" channel_ptr should be a swig-rwapped shared_ptr to snde::channel"""
        cdef size_t serial_size
        cdef char *serial
        cdef uint32_t num_devices
        cdef uint32_t devicenum
        cdef k4a_result_t errcode
        cdef k4a_buffer_result_t buferrcode

        #self.result_chan_ptr = channel_ptr 
        
        self.dev=NULL
        self.config_lock=Lock()
        self.serial_number = None
        #self.calibration=NULL
        self.transformation=NULL
        self.capture_running=False
        
        num_devices = k4a_device_get_installed_count()
        
        if num_devices < 1:
            raise IOError("No Azure Kinect devices found")
        
        if serial_number_str_or_none is None:
            errcode = k4a_device_open(K4A_DEVICE_DEFAULT,&self.dev)
            if errcode != K4A_RESULT_SUCCEEDED:
                raise IOError("Error contacting default Azure Kinect device")
            self.serial_number = "None"
            pass
        else:
            serial_strs = []
            for devicenum in range(num_devices):
                serial_size = 0

                self.dev=NULL
                errcode = k4a_device_open(devicenum,&self.dev)
                if errcode != K4A_RESULT_SUCCEEDED:
                    raise IOError("Error contacting Azure Kinect device index %d" % (devicenum))
                
                buferrcode = k4a_device_get_serialnum(self.dev,NULL,&serial_size)
                if buferrcode != K4A_BUFFER_RESULT_SUCCEEDED:
                    raise IOError("Error obtaining serial number of Azure Kinect device index %d" % (devicenum))

                serial = <char *>calloc(serial_size,1)
                k4a_device_get_serialnum(self.dev,serial,&serial_size)
                serial_bytes =  <bytes>serial
                serial_str = serial_bytes.decode('utf-8')
                if serial_str == serial_number_str_or_none:
                    # Got match!
                    self.serial_number = serial
                    free(serial)
                    break

                serial_strs.append(serial_str)
                
                free(serial)
                k4a_device_close(self.dev)
                self.dev=NULL
                pass

            if self.dev is NULL:
                raise IOError("No Azure Kinect devices found matching serial number %s; Found serial numbers: %s" % (serial_number_str_or_none,serial_strs))
            pass

        pass
    
    
    def __del__(self):
        if self.transformation is not NULL:
            k4a_transformation_destroy(self.transformation)
            self.transformation=NULL
            pass

        if self.dev is not NULL:
            k4a_device_close(self.dev)
            self.dev=NULL
            pass
        pass
    
    def get_running_depth_pixel_shape(self):
        cdef k4a_depth_mode_t depth_mode

        with self.config_lock:
            depth_mode = self.config.depth_mode
            pass
        
        assert(self.capture_running)
        
        if depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED :
            return (320,288)
        elif depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED:
            return (640,576)
        elif depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED:
            return (512,512)
        elif depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED:
            return (1024,1024)
        elif depth_mode == K4A_DEPTH_MODE_PASSIVE_IR:
            return (1024,1024)
        pass
    

    def get_calibration_extrinsics(self,unsigned source,unsigned target):
        cdef k4a_color_resolution_t color_res
        cdef k4a_result_t errcode
        cdef k4a_calibration_t calibration
        cdef float rotation[9];
        cdef float translation[3];
        
        if source >= <unsigned>K4A_CALIBRATION_TYPE_NUM:
            raise ValueError("Invalid calibration source %u (see K4A_CALIBRATION_TYPE_xxxxx)" % (source))

        if target >= <unsigned>K4A_CALIBRATION_TYPE_NUM:
            raise ValueError("Invalid calibration target %u (see K4A_CALIBRATION_TYPE_xxxxx)" % (target))

        with self.config_lock:
            color_res = self.config.color_resolution
            # If the color resolution is not set, the k4a_device_get_calibration() will fail to get the calibration extrinsics for the color camera. Since calibration extrinsics do not depend on color resolution, we arbitrarily substitute 720p.
            if color_res == K4A_COLOR_RESOLUTION_OFF:
                color_res = K4A_COLOR_RESOLUTION_720P
                pass
            errcode = k4a_device_get_calibration(self.dev,self.config.depth_mode,color_res,&calibration)
            pass

        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error obtaining Azure Kinect device calibration (device serial %s)" % (self.serial_number))

        rotation = calibration.extrinsics[source][target].rotation
        translation = calibration.extrinsics[source][target].translation
        
        rotmtx = np.zeros((4,4),dtype='f',order="F")
        rotmtx[0,0] = rotation[0]
        rotmtx[0,1] = rotation[1]
        rotmtx[0,2] = rotation[2]
        rotmtx[1,0] = rotation[3]
        rotmtx[1,1] = rotation[4]
        rotmtx[1,2] = rotation[5]
        rotmtx[2,0] = rotation[6]
        rotmtx[2,1] = rotation[7]
        rotmtx[2,2] = rotation[8]

        rotmtx[0,3] = translation[0]
        rotmtx[1,3] = translation[1]
        rotmtx[2,3] = translation[2]
        rotmtx[3,3] = 1.0

        k4a_targetak_over_sourceak = snde.rotmtx_build_orientation(rotmtx.ravel(order="F"))
        # This transformation is in the Azure Kinect coordinate convention. But we want it
        # in terms of spatialnde2 coordinates which are (a) rotated 180 deg around x, and
        # (b) offsets in meters instead of mm.

        gl_over_ak = np.array(((0.0,0.0,0.0,0.0),(1.0,0.0,0.0,0.0)),dtype=orient_dtype) # pi rotation about x axis
        ak_over_gl = snde.orientation_inverse(gl_over_ak) # not actually different

        k4a_targetgl_over_sourcegl_mm = snde.orientation_orientation_multiply(gl_over_ak,snde.orientation_orientation_multiply(k4a_targetak_over_sourceak,ak_over_gl))
        
        k4a_targetgl_over_sourcegl = k4a_targetgl_over_sourcegl_mm.copy()
        k4a_targetgl_over_sourcegl["offset"] = k4a_targetgl_over_sourcegl["offset"]/1000.0

        return k4a_targetgl_over_sourcegl
    
    def start_capture(self):
        cdef k4a_result_t errcode

        assert(not self.capture_running)

        with self.config_lock:
            errcode = k4a_device_get_calibration(self.dev,self.config.depth_mode,self.config.color_resolution,&self.calibration)
            pass
        
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error obtaining Azure Kinect device calibration (device serial %s)" % (self.serial_number))

        if self.transformation is not NULL:
            k4a_transformation_destroy(self.transformation)
            self.transformation=NULL
            pass
        
        
        self.transformation = k4a_transformation_create(&self.calibration)

        with self.config_lock:
            errcode = k4a_device_start_cameras(self.dev,&self.config)
            pass
        
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error starting Azure Kinect device capture (device serial %s): errcode = %d. Check if you have selected a supported configuration or try enabling debug messages with the K4A_ENABLE_LOG_TO_STDOUT=1 environment variable." % (self.serial_number,errcode))

        self.capture_running = True
        pass

    cpdef K4AAcquisition wait_frame(self,int32_t timeout_ms):
        """timeout_ms may be K4A_WAIT_INFINITE to wait forever"""
        cdef k4a_wait_result_t waitresult
        cdef k4a_capture_t capt=NULL;
        
        assert(self.capture_running)
        #sys.stderr.write("kinect.pyx k4a_device_get_capture start\n")
        with nogil:
            waitresult = k4a_device_get_capture(self.dev,&capt,timeout_ms)
            pass
        #sys.stderr.write("kinect.pyx k4a_device_get_capture end : waitresult = %d\n"%waitresult)

        monotonic_timestamp = time.monotonic()
        os_timestamp = time.time()
        
        if waitresult == K4A_WAIT_RESULT_TIMEOUT:
            #sys.stderr.write("kinect.pyx : got K4A_WAIT_RESULT_TIMEOUT\n")
            return None
        elif waitresult == K4A_WAIT_RESULT_FAILED:
            k4a_device_stop_cameras(self.dev)  # per SDK instructions, issue stop instruction after error return from k4a_device_get_capture
            self.capture_running=False
            #sys.stderr.write("kinect.pyx : got K4A_WAIT_RESULT_FAILED LowLevel.capture_running=False\n")
            raise IOError("k4a_device_get_capture failed on serial number %s" % (self.serial_number))
        
        assert(waitresult==K4A_WAIT_RESULT_SUCCEEDED)

        return K4AAcquisition.create(self.serial_number,capt,monotonic_timestamp,os_timestamp)
    
    def halt_from_other_thread(self):
        # Per the documentation, this will make k4a_device_get_capture exit with an error message
        #sys.stderr.write("kinect.pyx : halt_from_other_thread was called : self.capture_running = %s\n"%(str(self.capture_running)))
        k4a_device_stop_cameras(self.dev)

        pass
    def abort(self):
        self.capture_running = False
        pass
    pass



cdef class K4AFileLowLevel:
    cdef k4a_playback_t playback
    #cdef object result_chan_ptr # swig-wrapped shared_ptr[channel] 
    cdef object filename # Python string
    cdef k4a_calibration_t calibration  # Only valid when capture_running
    cdef k4a_transformation_t transformation
    cdef bool_t capture_running
    cdef k4a_record_configuration_t config

    cdef k4a_calibration_type_t point_cloud_frame # K4A_CALIBRATION_TYPE_DEPTH or K4A_CALIBRATION_TYPE_COLOR
    

    def __cinit__(self,filename):
        #""" channel_ptr should be a swig-rwapped shared_ptr to snde::channel"""
        cdef k4a_result_t errcode
        cdef k4a_buffer_result_t buferrcode
        cdef k4a_depth_mode_t depth_mode
    
        #self.result_chan_ptr = channel_ptr 
        
        self.playback=NULL
        #self.calibration=NULL
        self.transformation=NULL
        self.capture_running=False
        
        
        errcode = k4a_playback_open(filename.encode('utf-8'),&self.playback)
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error contacting Azure Kinect recording file %s" % (filename))
        
        errcode = k4a_playback_get_calibration(self.playback,&self.calibration)
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error obtaining calibration from recording file")


        self.transformation = k4a_transformation_create(&self.calibration)

        errcode = k4a_playback_get_record_configuration(self.playback,&self.config)
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error obtaining configuration from recording file")
        
        pass
    
    
    def __del__(self):
        if self.transformation is not NULL:
            k4a_transformation_destroy(self.transformation)
            self.transformation=NULL
            pass

        if self.playback is not NULL:
            k4a_playback_close(self.playback)
            self.playback=NULL
            pass
        pass


    def get_calibration_extrinsics(self,unsigned source,unsigned target):
        cdef float rotation[9];
        cdef float translation[3];
        
        if source >= <unsigned>K4A_CALIBRATION_TYPE_NUM:
            raise ValueError("Invalid calibration source %u (see K4A_CALIBRATION_TYPE_xxxxx)" % (source))

        if target >= <unsigned>K4A_CALIBRATION_TYPE_NUM:
            raise ValueError("Invalid calibration target %u (see K4A_CALIBRATION_TYPE_xxxxx)" % (target))



        rotation = self.calibration.extrinsics[source][target].rotation
        translation = self.calibration.extrinsics[source][target].translation
        
        rotmtx = np.zeros((4,4),dtype='f',order="F")
        rotmtx[0,0] = rotation[0]
        rotmtx[0,1] = rotation[1]
        rotmtx[0,2] = rotation[2]
        rotmtx[1,0] = rotation[3]
        rotmtx[1,1] = rotation[4]
        rotmtx[1,2] = rotation[5]
        rotmtx[2,0] = rotation[6]
        rotmtx[2,1] = rotation[7]
        rotmtx[2,2] = rotation[8]

        rotmtx[0,3] = translation[0]
        rotmtx[1,3] = translation[1]
        rotmtx[2,3] = translation[2]
        rotmtx[3,3] = 1.0

        k4a_targetak_over_sourceak = snde.rotmtx_build_orientation(rotmtx.ravel(order="F"))
        # This transformation is in the Azure Kinect coordinate convention. But we want it
        # in terms of spatialnde2 coordinates which are (a) rotated 180 deg around x, and
        # (b) offsets in meters instead of mm.

        gl_over_ak = np.array(((0.0,0.0,0.0,0.0),(1.0,0.0,0.0,0.0)),dtype=orient_dtype) # pi rotation about x axis
        ak_over_gl = snde.orientation_inverse(gl_over_ak) # not actually different

        k4a_targetgl_over_sourcegl_mm = snde.orientation_orientation_multiply(gl_over_ak,snde.orientation_orientation_multiply(k4a_targetak_over_sourceak,ak_over_gl))
        
        k4a_targetgl_over_sourcegl = k4a_targetgl_over_sourcegl_mm.copy()
        k4a_targetgl_over_sourcegl["offset"] = k4a_targetgl_over_sourcegl["offset"]/1000.0

        return k4a_targetgl_over_sourcegl

    
    def get_running_depth_pixel_shape(self):
        assert(self.capture_running)

        
        if self.config.depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED :
            return (320,288)
        elif self.config.depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED:
            return (640,576)
        elif self.config.depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED:
            return (512,512)
        elif self.config.depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED:
            return (1024,1024)
        elif self.config.depth_mode == K4A_DEPTH_MODE_PASSIVE_IR:
            return (1024,1024)
        pass
    
        

    cpdef K4AAcquisition next_frame(self):
        cdef k4a_capture_t capt=NULL
        cdef k4a_stream_result_t streamerr
        
        assert(self.capture_running)

        with nogil:
            streamerr = k4a_playback_get_next_capture(self.playback, &capt)
            pass

        if streamerr == K4A_STREAM_RESULT_EOF:
            self.capture_running=False
            return None

        if streamerr != K4A_STREAM_RESULT_SUCCEEDED:
            self.capture_running=False
            raise IOError("Error obtaining capture from recording file")
            
        
        monotonic_timestamp = time.monotonic()
        os_timestamp = time.time()
        
        return K4AAcquisition.create(self.filename,capt,monotonic_timestamp,os_timestamp)
    
    def halt_from_other_thread(self):
        # no-op
        pass


    def rewind(self):
        cdef k4a_result_t errcode
        cdef k4a_record_configuration_t config
        
        errcode = k4a_playback_get_record_configuration(self.playback,&config)
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error obtaining configuration from recording file")

        errcode = k4a_playback_seek_timestamp(self.playback,config.start_timestamp_offset_usec,K4A_PLAYBACK_SEEK_BEGIN)
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error rewinding playback file")
        pass
    
    pass




class K4A(object,metaclass=dgpy_Module):
    # dgpy_Module ensures that all calls to this are within the same thread
    module_name=None # our module name
    recdb = None # Swig-wrapped recording database
    LowLevel=None # kinect_lowlevel.K4ALowLevel object ... After initialization all access are from the capture thread. (at least so far)
    result_depth_channel_name = None 
    result_color_channel_name = None 
    result_depth_channel_ptr = None # swig-wrapped shared pointer to snde::channel
    result_color_channel_ptr = None # swig-wrapped shared pointer to snde::channel
    _capture_thread = None

    _capture_running_cond = None # dataguzzler-python Condition variable for waiting on capture_running. This is after the module context locks in the locking order, after transaction locks, but before other spatialnde2 locks
    _capture_running = None  # Boolean, written only by sub-thread with capture_running_cond locked
    _capture_start = None  # Boolean, set only by main thread and cleared only by sub-thread with capture_running_cond locked
    _capture_stop = None  # Boolean, set only by main thread and cleared only by sub-thread with capture_running_cond locked
    _capture_failed = None # Boolean, set only by sub thread and cleared only by main thread to indicate a failure condition
    _capture_exit = None # Boolean, set only by main thread; triggers sub thread to exit. 
    _previous_globalrev_complete_waiter = None # Used for _calcsync mode; set only by sub thread with capture_running_cond locked but used by main thread

    _desired_run_state = None

    _depth_data_mode = None
    _depth_data_type = None
    _calcsync = None

    dynamic_metadata = None

    
    @property
    def running(self):
        with self._capture_running_cond:
            return self._capture_running
        pass

    @running.setter
    def running(self,value):
        self._desired_run_state = value
        with self._capture_running_cond:
            if self._capture_running and not self._desired_run_state:
                self._stop_capture_cond_locked()
                pass
            elif not self._capture_running and self._desired_run_state:
                self._start_capture_cond_locked()
                pass
            pass
        pass


    @property
    def color_format(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.color_format
        pass
    
    @color_format.setter
    def color_format(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.color_format = value
                pass
            self._restart_if_appropriate()
            pass
        
        pass


    @property
    def color_resolution(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.color_resolution
        pass
    @color_resolution.setter
    def color_resolution(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.color_resolution = value
                pass
            self._restart_if_appropriate()
            pass        
        pass


    @property
    def depth_mode(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.depth_mode
        pass
    
    @depth_mode.setter
    def depth_mode(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.depth_mode = value
                pass
            self._restart_if_appropriate()
            pass        
        pass



    @property
    def camera_fps(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.camera_fps
        pass
    
    @camera_fps.setter
    def camera_fps(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.camera_fps = value
                pass
            self._restart_if_appropriate()
            pass        
        pass

    @property
    def synchronized_images_only(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.synchronized_images_only
        pass
    
    @synchronized_images_only.setter
    def synchronized_images_only(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value = bool(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.synchronized_images_only = value
                pass
            self._restart_if_appropriate()
            pass        
        pass
    
    @property
    def depth_delay_off_color_usec(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.depth_delay_off_color_usec
        pass
    
    @depth_delay_off_color_usec.setter
    def depth_delay_off_color_usec(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.depth_delay_off_color_usec = value
                pass
            self._restart_if_appropriate()
            pass        
        pass

    
    @property
    def wired_sync_mode(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.wired_sync_mode
        pass
    @wired_sync_mode.setter
    def wired_sync_mode(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.wired_sync_mode = value
                pass
            self._restart_if_appropriate()
            pass        
        pass
    
    @property
    def subordinate_delay_off_master_usec(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.subordinate_delay_off_master_usec
        pass
    @subordinate_delay_off_master_usec.setter
    def subordinate_delay_off_master_usec(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.subordinate_delay_off_master_usec = value
                pass
            self._restart_if_appropriate()
            pass        
        pass

    @property
    def disable_streaming_indicator(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        with LowLevel.config_lock:
            return LowLevel.config.disable_streaming_indicator
        pass
    @disable_streaming_indicator.setter
    def disable_streaming_indicator(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value = bool(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            with LowLevel.config_lock:
                LowLevel.config.disable_streaming_indicator = value
                pass
            self._restart_if_appropriate()
            pass        
        pass

    @property
    def point_cloud_frame(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        return LowLevel.point_cloud_frame
    @point_cloud_frame.setter
    def point_cloud_frame(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        if value != K4A_CALIBRATION_TYPE_DEPTH and value != K4A_CALIBRATION_TYPE_COLOR:
            raise ValueError("Invalid point_cloud_frame: Must be K4A_CALIBRATION_TYPE_DEPTH or K4A_CALIBRATION_TYPE_COLOR")        
        with self._capture_running_cond:
            self._stop_temporarily()
            LowLevel.point_cloud_frame = value
            self._restart_if_appropriate()
            pass        
        pass


    
    @property
    def depth_data_mode(self):
        return self._depth_data_mode
    @depth_data_mode.setter
    def depth_data_mode(self,value):
        if value == "IMAGE" or value == "POINTCLOUD":
            with self._capture_running_cond:
                self._stop_temporarily()
                self._depth_data_mode = value;
                self._restart_if_appropriate()
                pass
            pass
        else:
            raise ValueError("Valid depth data modes are \"IMAGE\" and \"POINTCLOUD\".")
        pass
    
    @property
    def depth_data_type(self):
        return self._depth_data_type
    @depth_data_type.setter
    def depth_data_type(self,value):
        if value == "INT" or value == "FLOAT":
            with self._capture_running_cond:
                self._stop_temporarily()
                self._depth_data_type = value;
                self._restart_if_appropriate()
                pass
            pass
        else:
            raise ValueError("Valid depth data types are \"INT\" and \"FLOAT\".")
        pass
    


    @property
    def calcsync(self):
        return self._calcsync
    @calcsync.setter
    def calcsync(self,value):
        value = bool(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            self._calcsync = value;
            self._restart_if_appropriate()
            pass
        pass
    


    def _stop_capture_cond_locked(self):
        # call with self._capture_running_cond locked
        self._capture_stop=True
        self.LowLevel.halt_from_other_thread()
        prevcomplete_waiter = self._previous_globalrev_complete_waiter
        if prevcomplete_waiter is not None:
            prevcomplete_waiter.interrupt()
            pass
        
        self._capture_running_cond.wait_for(lambda: not self._capture_running)
        pass

    def _start_capture_cond_locked(self):
        # call with self._capture_running_cond locked
        self._capture_start=True
        self._capture_running_cond.notify_all()
        self._capture_running_cond.wait_for(lambda: (self._capture_running or self._capture_failed))
        self._capture_failed = False # Accept message 
        pass

    
    def _stop_temporarily(self):
        # call with self._capture_running_cond locked
        if self._capture_running:
            self._stop_capture_cond_locked()
            pass
        pass

    def _restart_if_appropriate(self):
        # call with self._capture_running_cond locked
        if not self._capture_running and self._desired_run_state:
            self._start_capture_cond_locked()
            pass
        pass
    
    
    def __init__(self,module_name,recdb,device_serialnumber,result_depth_channel_name,result_color_channel_name=None):
        """
        device_serialnumber can be a string or None if only one camera is attached. """

        cdef K4ALowLevel LowLevel
        
        self.module_name = module_name
        self.recdb = recdb
        self.LowLevel = K4ALowLevel(device_serialnumber)
        self.result_depth_channel_name=result_depth_channel_name
        self.result_color_channel_name=result_color_channel_name

        self._capture_running_cond = Condition()
        #self.capture_running = None
        self._capture_running = False
        self._capture_start = False
        self._capture_stop = False
        self._capture_failed = False
        self._desired_run_state = False
        self._capture_exit = False
        
        # Default configuration
        LowLevel = self.LowLevel
        with LowLevel.config_lock:
            LowLevel.config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
            if result_color_channel_name is not None:
                LowLevel.config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32
                LowLevel.config.color_resolution = K4A_COLOR_RESOLUTION_720P
                pass
            
            if result_depth_channel_name is not None:
                LowLevel.config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED         # will give 640x576 array
                pass
        

            LowLevel.config.camera_fps = K4A_FRAMES_PER_SECOND_30

            if result_color_channel_name is not None and result_depth_channel_name is not None:
                LowLevel.config.synchronized_images_only = True
                pass
            
            LowLevel.config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE
            LowLevel.config.subordinate_delay_off_master_usec = 0  # use 160 for a secondary camera
            LowLevel.config.disable_streaming_indicator = False
            LowLevel.point_cloud_frame = K4A_CALIBRATION_TYPE_DEPTH
            pass
        
        self._depth_data_mode = "POINTCLOUD"
        self._depth_data_type = "FLOAT"
        self._calcsync = True

        self.dynamic_metadata = DynamicMetadata(module_name)
        
        # Transaction required to add a channel
        transact = recdb.start_transaction()

        if result_depth_channel_name is not None:
            self.result_depth_channel_ptr = recdb.reserve_channel(snde.channelconfig(result_depth_channel_name,module_name,self,False))
            pass

        if result_color_channel_name is not None: 
            self.result_color_channel_ptr = recdb.reserve_channel(snde.channelconfig(result_color_channel_name,module_name,self,False))
            pass
        
        transact.end_transaction()
        
        self.capture_thread = Thread(target=self.capture_thread_code)
        self.capture_thread.start() # Won't actually be able to record a transaction until this one ends.
        
        ## Wait for thread to initialize (probably not necessary)
        #with self._capture_running_cond:
        #    self._capture_running_cond.wait_for(lambda: self._capture_running is not None)
        #    pass

        atexit.register(self.atexit) # Register an atexit function so that we can cleanly trigger our subthread to end. Otherwise we might well crash on exit.
        pass

    def atexit(self):
        #print("kinect: Performing atexit()")
        with self._capture_running_cond:
            self._capture_exit = True;
            self._stop_temporarily()
            pass

        self.capture_thread.join()
        
        pass
    

    def capture_thread_code(self):
        cdef k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
        cdef K4AAcquisition cur_acq=K4AAcquisition()
        cdef size_t depth_width
        cdef size_t depth_height
        cdef k4a_depth_mode_t depth_mode
        cdef k4a_calibration_t calibration
        cdef K4ALowLevel LowLevel

        InitCompatibleThread(self,"_k4a_capture_thread")
        LowLevel = self.LowLevel
        
        

        ## Notify parent by flipping _capture_running to False from None
        #with self._capture_running_cond:
        #    self._capture_running = False
        #    self._capture_running_cond.notify_all()
        #    pass
        
        while True:

            with self._capture_running_cond:
                #sys.stderr.write("kinect.pyx : About to call capture_running_cond.wait_for, LowLevel.capture_running = %s\n"%str(LowLevel.capture_running))
                self._capture_running_cond.wait_for(lambda: self._capture_start or self._capture_exit)
                #sys.stderr.write("kinect.pyx : capture_running_cond.wait_for has returned LowLevel.capture_running = %s self._capture_start = %s\n"%(str(LowLevel.capture_running),str(self._capture_start)))

                if self._capture_start:
                    try:
                        LowLevel.start_capture()
                        self._capture_running = True
                        self._capture_start = False
                        self._capture_running_cond.notify_all()
                        pass
                    except IOError as e:
                        snde.snde_warning(str(e))
                        self._capture_running=False
                        self._capture_start = False
                        self._capture_failed = True
                        self._capture_running_cond.notify_all()
                        pass
                    pass
                
                pass

            if self._capture_exit:
                return

            while self._capture_running:  # Other threads not allowed to change this variable so we are safe to read it
                aborted = False
                with self._capture_running_cond:
                    previous_globalrev_complete_waiter = self._previous_globalrev_complete_waiter
                    # Check for stop request
                    if self._capture_stop:
                        self._capture_running=False
                        self._capture_stop=False
                        self._capture_running_cond.notify_all()
                        aborted = True
                        pass
                    pass
                    
                if aborted:
                    LowLevel.abort()
                    break
                
                if previous_globalrev_complete_waiter is not None:

                    # If we need to wait for the previous globalrev to be complete
                    # before starting a new acquisition, wait here.
                    # This can be interrupted from the other thread
                    # by calling previous_globalrev_complete_waiter.interrupt()
                    interrupted = previous_globalrev_complete_waiter.wait_interruptable()
                    aborted = False
                    with self._capture_running_cond:
                        if not interrupted:
                            # waiter satisfied
                            self._previous_globalrev_complete_waiter = None
                            pass
                                        
                        # Check for stop request
                        if self._capture_stop:
                            self._capture_running=False
                            self._capture_stop=False
                            self._capture_running_cond.notify_all()
                            aborted = True
                            pass
                        pass
                    if aborted:
                        LowLevel.abort()
                        break
                    if interrupted:
                        continue  # So we can wait again if we were interrupted and somehow didn't have a stop request

                    
                    pass
                
                try:
                    cur_acq = LowLevel.wait_frame(K4A_WAIT_INFINITE)
                    pass
                
                except IOError as e:
                    # Notify parent of failure and/or that we have stopped at their request
                    intentional_stop = False
                    with self._capture_running_cond:
                        self._capture_running=False
                        if self._capture_stop:
                            intentional_stop = True
                            self._capture_stop = False
                            pass
                        self._capture_running_cond.notify_all()
                        if LowLevel.capture_running:
                            LowLevel.abort()
                            pass
                        pass
                    if not intentional_stop:
                        snde.snde_warning(str(e)) # print out warning message
                        pass
                    
                    pass

                if self._capture_running: # Successful acquisition
                    (depth_width,depth_height) = LowLevel.get_running_depth_pixel_shape()
                    
                    transact = self.recdb.start_transaction()

                    depth_recording_ref = None
                    color_recording_ref = None
                    
                    if self.result_depth_channel_ptr is not None:
                        if self._depth_data_mode == "IMAGE":
                            if self._depth_data_type == "INT":
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_INT16)
                                pass
                            else: # FLOAT
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_FLOAT32)
                                pass

                            pass
                        else: #  self._depth_data_mode == "POINTCLOUD":
                            if self._depth_data_type == "INT":
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_SNDE_COORD3_INT16)
                                pass
                            else: # FLOAT
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_SNDE_COORD3)
                                pass
                            pass
                        pass
                    
                    if self.result_color_channel_ptr is not None:
                        # Assign color_recording ref...
                        pass

                    depth_recording_ref.rec.recording_needs_dynamic_metadata()

                    assert(self._previous_globalrev_complete_waiter is None)

                    transobj = transact.run_in_background_and_end_transaction(self.dynamic_metadata.Snapshot().Acquire,(depth_recording_ref.rec,))
                    if self._calcsync:
                        with self._capture_running_cond:
                            #print(dir(transobj))
                            self._previous_globalrev_complete_waiter = transobj.get_transaction_globalrev_complete_waiter()
                            pass
                        pass
                    
                    if self.result_depth_channel_ptr is not None:
                        calibration = LowLevel.calibration
                        # Starting from OpenCV calibration documentation
                        # u = u'/w' = (fx*Xc + cx*Zc)/Zc
                        # u = (fx*Xc/Zc + cx)
                        # u/fx = Xc/Zc + cx/fx
                        # Therefore steps are 1.0/fx, 1.0/fy
                        # Units are tangent_ray_angle
                        # Initial value is -cx/fx
                        
                                               
                        metadata = snde.constructible_metadata()
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis0_scale",1.0/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fx,"tan_horiz_angle"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis1_scale",-1.0/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fy,"tan_vert_angle")) # negative step because our coordinate frames start at lower left corner but camera data starts at upper left
                        #sys.stderr.write("Azure Kinect: dy=%f\n" %(1.0/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fy))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis0_offset",-LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.cx/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fx,"tan_horiz_angle"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis1_offset",(depth_height-LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.cy-1)/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fy,"tan_vert_angle"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis0_coord","X Position"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis1_coord","Y Position"))
                      
                        if self._depth_data_type == "INT":
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_units","mm"))
                            pass
                        else:
                            # we scale by k4a_meters_per_lsb when generating floats
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_units","m"))
                            pass

                        if self._depth_data_mode!="IMAGE": # POINTCLOUD
                            # Enable point cloud style rendering
                            metadata.AddMetaDatum(snde.metadatum("snde_render_goal","SNDE_SRG_POINTCLOUD"))
                            metadata.AddMetaDatum(snde.metadatum("snde_render_goal_3d","SNDE_SRG_POINTCLOUD"))
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_coord","Position"))
                            pass
                        else:
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_coord","Z Position"))
                            pass
                        
                        depth_recording_ref.rec.metadata = metadata 

                        
                        depth_recording_ref.rec.mark_metadata_done()
                        depth_recording_ref.allocate_storage([depth_width,depth_height],True) # Fortran mode

                        depth_data_array = depth_recording_ref.data()
                        #sys.stderr.write("depth shape=%s; depth dtype=%s; depth nbytes=%d\n" % (str(depth_data_array.shape),str(depth_data_array.dtype),depth_data_array.nbytes))
                        if self._depth_data_type == "INT":
                            depth_data_array_view= depth_data_array.T.view(np.int16).T
                            pass
                        else: # FLOAT
                            depth_data_array_view= depth_data_array.T.view(np.float32).T
                            pass

                        #sys.stderr.write("depth view shape=%s; depth dtype=%s; depth nbytes=%d\n" % (str(depth_data_array_view.shape),str(depth_data_array_view.dtype),depth_data_array_view.nbytes))
                        if self._depth_data_mode=="IMAGE":
                            depth_data_array_view = depth_data_array_view.reshape(depth_width,depth_height,order='F')
                            pass
                        else: # POINTCLOUD                            
                            depth_data_array_view = depth_data_array_view.reshape(3,depth_width,depth_height,order='F')
                            pass

                        with LowLevel.config_lock:
                            depth_mode = LowLevel.config.depth_mode
                            pass
                        
                        cur_acq.get_depth_data(depth_mode,LowLevel.transformation,depth_data_array_view, depth_width, depth_height,self._depth_data_mode != "IMAGE",self._depth_data_type != "INT")
                        if np.isnan(depth_data_array_view).any():
                            raise ValueError("NaN's")
                        
                    
                        depth_recording_ref.rec.mark_data_ready()
                        pass
                    pass

                cur_acq.release_buffers()
                
                #if self._calcsync:
                #    globalrev.wait_complete()
                #    #print("Globalrev %d/%d is complete" % (globalrev.globalrev,globalrev.unique_index))
                #    #import time
                #    #time.sleep(5)
                #    pass
                #pass
            
            pass
        
        pass
    pass







class K4AFile(object,metaclass=dgpy_Module):
    # dgpy_Module ensures that all calls to this are within the same thread
    module_name=None # our module name
    recdb = None # Swig-wrapped recording database
    LowLevel=None # kinect_lowlevel.K4ALowLevel object ... After initialization all access are from the capture thread. (at least so far)
    result_depth_channel_name = None 
    result_color_channel_name = None 
    result_depth_channel_ptr = None # swig-wrapped shared pointer to snde::channel
    result_color_channel_ptr = None # swig-wrapped shared pointer to snde::channel
    _capture_thread = None

    _capture_running_cond = None # dataguzzler-python Condition variable for waiting on capture_running
    _capture_running = None  # Boolean, written only by sub-thread with capture_running_cond locked
    _capture_start = None  # Boolean, set only by main thread and cleared only by sub-thread with capture_running_cond locked
    _capture_stop = None  # Boolean, set only by main thread and cleared only by sub-thread with capture_running_cond locked
    _capture_failed = None # Boolean, set only by sub thread and cleared only by main thread to indicate a failure condition

    _desired_run_state = None

    _depth_data_mode = None
    _depth_data_type = None
    _calcsync = None
    
    @property
    def running(self):
        with self._capture_running_cond:
            return self._capture_running
        pass

    @running.setter
    def running(self,value):
        self._desired_run_state = value
        with self._capture_running_cond:
            if self._capture_running and not self._desired_run_state:
                self._stop_capture_cond_locked()
                pass
            elif not self._capture_running and self._desired_run_state:
                self._start_capture_cond_locked()
                pass
            pass
        pass



    @property
    def depth_mode(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        return LowLevel.config.depth_mode
    @depth_mode.setter
    def depth_mode(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            LowLevel.config.depth_mode = value
            self._restart_if_appropriate()
            pass        
        pass



    @property
    def point_cloud_frame(self):
        cdef K4ALowLevel LowLevel = self.LowLevel
        return LowLevel.point_cloud_frame
    @point_cloud_frame.setter
    def point_cloud_frame(self,value):
        cdef K4ALowLevel LowLevel = self.LowLevel
        value=int(value)
        if value != K4A_CALIBRATION_TYPE_DEPTH and value != K4A_CALIBRATION_TYPE_COLOR:
            raise ValueError("Invalid point_cloud_frame: Must be K4A_CALIBRATION_TYPE_DEPTH or K4A_CALIBRATION_TYPE_COLOR")        
        with self._capture_running_cond:
            self._stop_temporarily()
            LowLevel.point_cloud_frame = value
            self._restart_if_appropriate()
            pass        
        pass


    
    @property
    def depth_data_mode(self):
        return self._depth_data_mode
    @depth_data_mode.setter
    def depth_data_mode(self,value):
        if value == "IMAGE" or value == "POINTCLOUD":
            with self._capture_running_cond:
                self._stop_temporarily()
                self._depth_data_mode = value;
                self._restart_if_appropriate()
                pass
            pass
        else:
            raise ValueError("Valid depth data modes are \"IMAGE\" and \"POINTCLOUD\".")
        pass
    
    @property
    def depth_data_type(self):
        return self._depth_data_type
    @depth_data_type.setter
    def depth_data_type(self,value):
        if value == "INT" or value == "FLOAT":
            with self._capture_running_cond:
                self._stop_temporarily()
                self._depth_data_type = value;
                self._restart_if_appropriate()
                pass
            pass
        else:
            raise ValueError("Valid depth data types are \"INT\" and \"FLOAT\".")
        pass
    


    @property
    def calcsync(self):
        return self._calcsync
    @calcsync.setter
    def calcsync(self,value):
        value = bool(value)
        with self._capture_running_cond:
            self._stop_temporarily()
            self._calcsync = value;
            self._restart_if_appropriate()
            pass
        pass
    


    def _stop_capture_cond_locked(self):
        # call with self._capture_running_cond locked
        self._capture_stop=True
        self.LowLevel.halt_from_other_thread()
        self._capture_running_cond.wait_for(lambda: not self._capture_running)
        pass

    def _start_capture_cond_locked(self):
        # call with self._capture_running_cond locked
        self._capture_start=True
        self._capture_running_cond.notify_all()
        self._capture_running_cond.wait_for(lambda: (self._capture_running or self._capture_failed))
        self._capture_failed = False # Accept message 
        pass

    
    def _stop_temporarily(self):
        # call with self._capture_running_cond locked
        if self._capture_running:
            self._stop_capture_cond_locked()
            pass
        pass

    def _restart_if_appropriate(self):
        # call with self._capture_running_cond locked
        if not self._capture_running and self._desired_run_state:
            self._start_capture_cond_locked()
            pass
        pass
    
    
    def __init__(self,module_name,recdb,MKVFileName,result_depth_channel_name,result_color_channel_name=None):
        """
        device_serialnumber can be a string or None if only one camera is attached. """

        cdef K4AFileLowLevel LowLevel


        if MKVFileName is None:
            raise ValueError("K4AFile: Filename given as None")
        
        self.module_name = module_name
        self.recdb = recdb
        self.LowLevel = K4AFileLowLevel(MKVFileName)
        self.result_depth_channel_name=result_depth_channel_name
        self.result_color_channel_name=result_color_channel_name

        self._capture_running_cond = Condition()
        #self.capture_running = None
        self._capture_running = False
        self._capture_start = False
        self._capture_stop = False
        self._capture_failed = False
        self._desired_run_state = False

        # Default configuration
        LowLevel = self.LowLevel
        
        self._depth_data_mode = "POINTCLOUD"
        self._depth_data_type = "FLOAT"
        self._calcsync = True
        
        
        # Transaction required to add a channel
        transact = recdb.start_transaction()

        if result_depth_channel_name is not None:
            self.result_depth_channel_ptr = recdb.reserve_channel(snde.channelconfig(result_depth_channel_name,module_name,self,False))
            pass

        if result_color_channel_name is not None: 
            self.result_color_channel_ptr = recdb.reserve_channel(snde.channelconfig(result_color_channel_name,module_name,self,False))
            pass
        
        transact.end_transaction()

        sys.stderr.write("k4afile: Creating thread object\n")
        self.capture_thread = Thread(target=self.capture_thread_code)
        sys.stderr.write("k4afile: Starting capture thread\n")
        self.capture_thread.start() # Won't actually be able to record a transaction until this one ends.
        sys.stderr.write("k4afile: Capture thread started\n")
        
        ## Wait for thread to initialize (probably not necessary)
        #with self._capture_running_cond:
        #    self._capture_running_cond.wait_for(lambda: self._capture_running is not None)
        #    pass
        
        pass
    

    def capture_thread_code(self):
        cdef K4AAcquisition cur_acq=K4AAcquisition()
        cdef size_t depth_width
        cdef size_t depth_height
        cdef k4a_calibration_t calibration
        cdef K4AFileLowLevel LowLevel
        #sys.stderr.write("k4afile: capture_thread_code started\n")
        
        InitCompatibleThread(self,"_k4a_capture_thread")
        LowLevel = self.LowLevel
        #sys.stderr.write("k4afile: capture_thread_code initialized\n")
        
        

        ## Notify parent by flipping _capture_running to False from None
        #with self._capture_running_cond:
        #    self._capture_running = False
        #    self._capture_running_cond.notify_all()
        #    pass
        
        while True:

            #sys.stderr.write("k4afile: capture_thread weaiting for start\n")
            with self._capture_running_cond:
                self._capture_running_cond.wait_for(lambda: self._capture_start)
                self._capture_running=True
                self._capture_start=False
                self._capture_running_cond.notify_all()
                pass
            #sys.stderr.write("k4afile: capture_thread got start\n")

            LowLevel.capture_running=True
            
            while self._capture_running:  # Other threads not allowed to change this variable so we are safe to read it
                
                try:
                    cur_acq = LowLevel.next_frame()
                    if cur_acq is None:
                        # EOF: stop and rewind
                        with self._capture_running_cond:
                            self._capture_running = False
                            self._capture_stop = False
                            self._capture_running_cond.notify_all()
                            pass
                        LowLevel.rewind()
                        LowLevel.capture_running=False
                        pass
                    pass
                
                except IOError as e:
                    # Notify parent of failure and/or that we have stopped at their request
                    intentional_stop = False
                    with self._capture_running_cond:
                        self._capture_running=False
                        if self._capture_stop:
                            intentional_stop = True
                            self._capture_stop = False
                            pass
                        self._capture_running_cond.notify_all()
                        pass
                    if not intentional_stop:
                        snde.snde_warning(str(e)) # print out warning message
                        pass
                    
                    pass

                if self._capture_running: # Successful acquisition
                    (depth_width,depth_height) = LowLevel.get_running_depth_pixel_shape()
                    
                    transact = self.recdb.start_transaction()

                    depth_recording_ref = None
                    color_recording_ref = None
                    
                    if self.result_depth_channel_ptr is not None:
                        if self._depth_data_mode == "IMAGE":
                            if self._depth_data_type == "INT":
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_INT16)
                                pass
                            else: # FLOAT
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_FLOAT32)
                                pass

                            pass
                        else: #  self._depth_data_mode == "POINTCLOUD":
                            if self._depth_data_type == "INT":
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_SNDE_COORD3_INT16)
                                pass
                            else: # FLOAT
                                depth_recording_ref = snde.create_ndarray_ref(self.recdb,self.result_depth_channel_ptr,self,snde.SNDE_RTN_SNDE_COORD3)
                                pass
                            pass
                        pass

                    
                    if self.result_color_channel_ptr is not None:
                        # Assign color_recording ref...
                        pass
                    globalrev = transact.end_transaction()
                    
                    if self.result_depth_channel_ptr is not None:
                        calibration = LowLevel.calibration
                        # Starting from OpenCV calibration documentation
                        # u = u'/w' = (fx*Xc + cx*Zc)/Zc
                        # u = (fx*Xc/Zc + cx)
                        # u/fx = Xc/Zc + cx/fx
                        # Therefore steps are 1.0/fx, 1.0/fy
                        # Units are tangent_ray_angle
                        # Initial value is -cx/fx
                        
                                               
                        metadata = snde.constructible_metadata()
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis0_scale",1.0/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fx,"tan_horiz_angle"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis1_scale",-1.0/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fy,"tan_vert_angle")) # negative step because our coordinate frames start at lower left corner but camera data starts at upper left
                        #sys.stderr.write("Azure Kinect: dy=%f\n" %(1.0/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fy))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis0_offset",-LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.cx/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fx,"tan_horiz_angle"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis1_offset",(depth_height-LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.cy-1)/LowLevel.calibration.depth_camera_calibration.intrinsics.parameters.param.fy,"tan_vert_angle"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis0_coord","X Position"))
                        metadata.AddMetaDatum(snde.metadatum("ande_array-axis1_coord","Y Position"))

                        if self._depth_data_type == "INT":
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_units","mm"))
                            pass
                        else:
                            # we scale by k4a_meters_per_lsb when generating floats
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_units","m"))
                            pass

                        if self._depth_data_mode!="IMAGE": # POINTCLOUD
                            # Enable point cloud style rendering
                            metadata.AddMetaDatum(snde.metadatum("snde_render_goal","SNDE_SRG_POINTCLOUD"))
                            metadata.AddMetaDatum(snde.metadatum("snde_render_goal_3d","SNDE_SRG_POINTCLOUD"))
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_coord","Position"))
                            pass
                        else:
                            metadata.AddMetaDatum(snde.metadatum("ande_array-ampl_coord","Z Position"))
                            pass
                        
                        depth_recording_ref.rec.metadata = metadata 

                        
                        depth_recording_ref.rec.mark_metadata_done()
                        depth_recording_ref.allocate_storage([depth_width,depth_height],True) # Fortran mode

                        #sys.stderr.write("drr: es=%u, bi=%u nelem=%u nbytes=%u dataaddr=0x%x\n" % (depth_recording_ref.storage.elementsize,depth_recording_ref.storage.base_index,depth_recording_ref.storage.nelem,depth_recording_ref.storage.nelem*depth_recording_ref.storage.elementsize,depth_recording_ref.storage.cur_dataaddr()))

                        
                        depth_data_array = depth_recording_ref.data()
                        
                        #sys.stderr.write("depth shape=%s; depth dtype=%s; depth nbytes=%d\n" % (str(depth_data_array.shape),str(depth_data_array.dtype),depth_data_array.nbytes))
                        if self._depth_data_type == "INT":
                            depth_data_array_view= depth_data_array.view(np.int16)
                            pass
                        else: # FLOAT
                            depth_data_array_view= depth_data_array.view(np.float32)
                            pass

                        #sys.stderr.write("depth view shape=%s; depth dtype=%s; depth nbytes=%d\n" % (str(depth_data_array_view.shape),str(depth_data_array_view.dtype),depth_data_array_view.nbytes))
                        if self._depth_data_mode=="IMAGE":
                            depth_data_array_view = depth_data_array_view.reshape(depth_width,depth_height,order='F')
                            pass
                        else: # POINTCLOUD                            
                            depth_data_array_view = depth_data_array_view.reshape(3,depth_width,depth_height,order='F')
                            pass


                        #sys.stderr.write("ddav: shape=%s strides=%s nbytes=%u data=0x%x\n" % (str(depth_data_array_view.shape),str(depth_data_array_view.strides),depth_data_array_view.nbytes,depth_data_array_view.__array_interface__.data[0]))
                        
                        cur_acq.get_depth_data(LowLevel.config.depth_mode,LowLevel.transformation,depth_data_array_view, depth_width, depth_height,self._depth_data_mode != "IMAGE",self._depth_data_type != "INT")
                        if np.isnan(depth_data_array_view).any():
                            raise ValueError("NaN's")
                        
                    
                        depth_recording_ref.rec.mark_data_ready()
                        pass
                    # !!! Need some way to tell this thread to quit when the user exits !!!***
                    pass
                if cur_acq is not None:
                    cur_acq.release_buffers()
                    pass
                
                if self._calcsync:
                    globalrev.wait_complete()
                    #print("Globalrev %d/%d is complete" % (globalrev.globalrev,globalrev.unique_index))
                    #import time
                    #time.sleep(5)
                    pass
                pass
            
            pass
        
        pass
    pass
