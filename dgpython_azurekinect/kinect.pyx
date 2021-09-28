# distutils: language = c++

import sys
import numbers
import collections
import time
import traceback
from threading import Thread

import spatialnde2 as snde

from libcpp cimport bool as bool_t
from libcpp.string cimport string
from cython.operator cimport dereference as deref

from dataguzzler_python.dgpy import Module as dgpy_Module
from dataguzzler_python.dgpy import CurContext
from dataguzzler_python.dgpy import RunInContext
from dataguzzler_python.dgpy import InitCompatibleThread


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
from libc.stdlib cimport calloc,free
from libc.string cimport memcpy

import numpy as np
cimport numpy as np



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
        pass
    
    ctypedef struct k4a_calibration_intrinsic_parameters_t:
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
    cdef object serial_number
    cdef k4a_capture_t capt
    cdef object monotonic_timestamp
    cdef object os_timestamp

    def __cinit__(self):
        
        self.serial_number = None
        self.capt = NULL
        self.monotonic_timestamp = None
        self.os_timestamp = None
        pass

    @staticmethod
    cdef K4AAcquisition create(object serial_number,k4a_capture_t capt, object monotonic_timestamp,object os_timestamp):
        cdef K4AAcquisition o = K4AAcquisition()
        o.serial_number = serial_number
        o.capt = capt
        o.monotonic_timestamp = monotonic_timestamp
        o.os_timestamp = os_timestamp
        #print("type(o)=%s o=%s" % (str(type(o)),str(o)))
        return o
    
    cdef get_point_cloud(self,K4ALowLevel lowlevel,void *buffer, int width, int height):
        """buffer should be of the image width and image
           height, with space for 3 int16's per pixel"""
        cdef k4a_depth_mode_t depth_mode = lowlevel.running_config.depth_mode
        cdef k4a_image_t depth_image=NULL
        cdef k4a_image_t point_cloud_image=NULL
        cdef k4a_result_t errcode
        cdef size_t stride
        
        assert(depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED or 
               depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED or 
               depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED or 
               depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED)
        
        with nogil:
            
            
            depth_image = k4a_capture_get_depth_image(self.capt);
            if depth_image is NULL:
                with gil:
                    raise IOError("k4a_device_get_depth_image failed on serial number %s" % (self.serial_number))
                pass

            if not(k4a_image_get_width_pixels(depth_image)==width and k4a_image_get_height_pixels(depth_image)==height):
                with gil:
                    raise ValueError("Image dimensions (%d,%d) do not match expected value (%d,%d)" % (k4a_image_get_width_pixels(depth_image),k4a_image_get_height_pixels(depth_image),width,height))
                pass
            
            stride = width*3*sizeof(int16_t)

            errcode = k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_CUSTOM,
                                                   width,
                                                   height,
                                                   stride,
                                                   <uint8_t *>buffer,
                                                   stride*height,
                                                   NULL,
                                                   NULL,
                                                   &point_cloud_image)
            if errcode != K4A_RESULT_SUCCEEDED:
                with gil:
                    raise RuntimeError("Error creating point cloud image from buffer")
                pass
            
            
            errcode = k4a_transformation_depth_image_to_point_cloud(lowlevel.transformation,depth_image,K4A_CALIBRATION_TYPE_DEPTH,point_cloud_image)
            if errcode != K4A_RESULT_SUCCEEDED:
                with gil:
                    raise RuntimeError("Error transforming depth image into point cloud")
                pass

            k4a_image_release(point_cloud_image)
            k4a_image_release(depth_image)
            pass

        pass


    cdef get_depth_image(self,K4ALowLevel lowlevel,void *buffer, int width, int height):
        """buffer should be of the image width and image
           height, with space for 1 int16's per pixel"""
        cdef k4a_depth_mode_t depth_mode = lowlevel.running_config.depth_mode
        cdef k4a_image_t depth_image=NULL
        cdef k4a_image_t point_cloud_image=NULL
        cdef k4a_result_t errcode
        cdef size_t stride
        cdef void *imgbuf;
        cdef uint64_t *longbuf;
        
        assert(depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED or 
               depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED or 
               depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED or 
               depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED)
        
        with nogil:
            
            
            depth_image = k4a_capture_get_depth_image(self.capt);
            if depth_image is NULL:
                with gil:
                    raise IOError("k4a_device_get_depth_image failed on serial number %s" % (self.serial_number))
                pass

            if not(k4a_image_get_width_pixels(depth_image)==width and k4a_image_get_height_pixels(depth_image)==height):
                with gil:
                    raise ValueError("Image dimensions (%d,%d) do not match expected value (%d,%d)" % (k4a_image_get_width_pixels(depth_image),k4a_image_get_height_pixels(depth_image),width,height))
                pass

            imgbuf = <void*>k4a_image_get_buffer(depth_image);
            longbuf=<uint64_t *>imgbuf;
            
            with gil:
                assert(k4a_image_get_stride_bytes(depth_image)==width*sizeof(int16_t));
                #assert(longbuf[0] != 0)
                
                pass
            
            memcpy(buffer,imgbuf,width*height*sizeof(int16_t));
            k4a_image_release(depth_image)
            pass

        pass


    
    def __del__(self):
        k4a_capture_release(self.capt)
        self.capt=NULL
        pass
    
cdef class K4ALowLevel:
    cdef k4a_device_t dev
    #cdef object result_chan_ptr # swig-wrapped shared_ptr[channel] 
    cdef object serial_number # Python string
    cdef k4a_calibration_t calibration  # Only valid when capture_running
    cdef k4a_transformation_t transformation
    cdef bool_t capture_running
    cdef k4a_device_configuration_t running_config

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
        self.serial_number = None
        #self.calibration=NULL
        self.transformation=NULL
        self.capture_running=False
        self.running_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
        
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
    
    def get_running_pixel_shape(self):
        cdef k4a_depth_mode_t depth_mode = self.running_config.depth_mode
        assert(self.capture_running)
        
        if depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED :
            return (320,288)
        elif depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED:
            return (640,576)
        elif depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED:
            return (512,512)
        elif depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED:
            return (1024,1024)
        pass
    
        
    def start_capture(self,k4a_device_configuration_t config):
        cdef k4a_result_t errcode

        assert(not self.capture_running)
        
        errcode = k4a_device_get_calibration(self.dev,config.depth_mode,config.color_resolution,&self.calibration)
        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error obtaining Azure Kinect device calibration (device serial %s)" % (self.serial_number))

        if self.transformation is not NULL:
            k4a_transformation_destroy(self.transformation)
            self.transformation=NULL
            pass
        
        
        self.transformation = k4a_transformation_create(&self.calibration)
        
        errcode = k4a_device_start_cameras(self.dev,&config)

        if errcode != K4A_RESULT_SUCCEEDED:
            raise IOError("Error starting Azure Kinect device capture (device serial %s)" % (self.serial_number))

        self.running_config = config
        self.capture_running = True
        pass

    cpdef K4AAcquisition wait_frame(self,int32_t timeout_ms):
        """timeout_ms may be K4A_WAIT_INFINITE to wait forever"""
        cdef k4a_wait_result_t waitresult
        cdef k4a_capture_t capt=NULL;
        
        assert(self.capture_running)

        with nogil: 
            waitresult = k4a_device_get_capture(self.dev,&capt,timeout_ms)
            pass

        monotonic_timestamp = time.monotonic()
        os_timestamp = time.time()
        
        if waitresult == K4A_WAIT_RESULT_TIMEOUT:
            return None
        elif waitresult == K4A_WAIT_RESULT_FAILED:
            raise IOError("k4a_device_get_capture failed on serial number %s" % (self.serial_number))
        
        assert(waitresult==K4A_WAIT_RESULT_SUCCEEDED)

        return K4AAcquisition.create(self.serial_number,capt,monotonic_timestamp,os_timestamp)
    pass

class K4A(object,metaclass=dgpy_Module):
    # dgpy_Module ensures that all calls to this are within the same thread
    module_name=None # our module name
    recdb = None # Swig-wrapped recording database
    LowLevel=None # Ordered Dictionary by name (or None) of kinect_lowlevel.K4ALowLevel objects ... After initialization all access are from the capture thread. (at least so far)
    result_channel_ptrs = None # Ordered Dictionary by name of swig-wrapped shared pointer to snde::channel
    capture_thread = None
    
    def __init__(self,module_name,recdb,device_serialnumber_ordereddict):
        """
        device_serialnumber_ordereddict will eventually be an ordered dictionary indexed by
        result channel name of device serial numbers. With only one device present it is OK
        to pass { "ResultChannel": None } (and nothing beyond that it yet supported). The First device
        is presumed to be the synchronization master. """

        self.module_name = module_name
        self.recdb = recdb
        self.LowLevel = collections.OrderedDict()
        self.result_channel_ptrs={}
        
        ResultChan_Name = list(device_serialnumber_ordereddict.keys())[0]

        # Transaction required to add a channel
        transact = snde.active_transaction(recdb)
        self.result_channel_ptrs[ResultChan_Name] = recdb.reserve_channel(snde.channelconfig(ResultChan_Name,module_name,self,False))
        transact.end_transaction()
        
        self.LowLevel[ResultChan_Name] = K4ALowLevel(device_serialnumber_ordereddict[ResultChan_Name])

        self.capture_thread = Thread(target=self.capture_thread_code)
        self.capture_thread.start() # Won't actually be able to record a transaction until this one ends.        
        pass
    

    def capture_thread_code(self):
        cdef k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
        cdef K4AAcquisition cur_acq
        cdef size_t width
        cdef size_t height
        
        InitCompatibleThread(self,"_capture_thread")

        
        ResultChan_Name = list(self.LowLevel.keys())[0]
        
        config.camera_fps = K4A_FRAMES_PER_SECOND_30
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32
        config.color_resolution = K4A_COLOR_RESOLUTION_720P
        config.synchronized_images_only = True
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED
        config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE
        # will give 640x576 array
        
        self.LowLevel[ResultChan_Name].start_capture(config)

        (width,height) = self.LowLevel[ResultChan_Name].get_running_pixel_shape()

        while True:
            
            cur_acq = self.LowLevel[ResultChan_Name].wait_frame(K4A_WAIT_INFINITE)
            transact = snde.active_transaction(self.recdb)
            point_cloud_recording = snde.ndarray_recording.create_typed_recording(self.recdb,self.result_channel_ptrs[ResultChan_Name],self,snde.SNDE_RTN_COORD3_INT16)
            transact.end_transaction()
            point_cloud_recording.metadata = snde.immutable_metadata()
            point_cloud_recording.mark_metadata_done()
            point_cloud_recording.allocate_storage([height,width])

            cur_acq.get_point_cloud(self.LowLevel[ResultChan_Name],<void *>(<uintptr_t>point_cloud_recording.void_dataptr()), width, height)
            #cur_acq.get_depth_image(self.LowLevel[ResultChan_Name],<void *>(<uintptr_t>point_cloud_recording.void_dataptr()), width, height)

            point_cloud_recording.mark_as_ready()
            

            
            pass
        
        pass
    pass
