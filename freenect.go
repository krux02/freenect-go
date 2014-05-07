package freenect

//#cgo CFLAGS: -I/home/doering/gocode/src/github.com/krux02/freenect-go/include/
//#cgo LDFLAGS: -lfreenect -L/home/doering/gocode/src/github.com/krux02/freenect-go/libfreenect/build/lib/
//#include <libfreenect/libfreenect.h>
//#include <stdlib.h>
//extern void callLogCallback(freenect_context *dev, freenect_loglevel level, char *msg);
import "C"

import (
	"unsafe"
)

const (
	CountsPerG = C.FREENECT_COUNTS_PER_G /**< Ticks per G for accelerometer as set per http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf */
	/// Maximum value that a uint16_t pixel will take on in the buffer of any of the FREENECT_DEPTH_MM or FREENECT_DEPTH_REGISTERED frame callbacks
	DepthMmMaxValue = C.FREENECT_DEPTH_MM_MAX_VALUE
	/// Value indicating that this pixel has no data, when using FREENECT_DEPTH_MM or FREENECT_DEPTH_REGISTERED depth modes
	DepthMmNoValue = C.FREENECT_DEPTH_MM_NO_VALUE
	/// Maximum value that a uint16_t pixel will take on in the buffer of any of the FREENECT_DEPTH_11BIT, FREENECT_DEPTH_10BIT, FREENECT_DEPTH_11BIT_PACKED, or FREENECT_DEPTH_10BIT_PACKED frame callbacks
	DepthRawMaxValue = C.FREENECT_DEPTH_RAW_MAX_VALUE
	/// Value indicating that this pixel has no data, when using FREENECT_DEPTH_11BIT, FREENECT_DEPTH_10BIT, FREENECT_DEPTH_11BIT_PACKED, or FREENECT_DEPTH_10BIT_PACKED
	DepthRawNoValue = C.FREENECT_DEPTH_RAW_NO_VALUE
)

/// Flags representing devices to open when freenect_open_device() is called.
/// In particular, this allows libfreenect to grab only a subset of the devices
/// in the Kinect, so you could (for instance) use libfreenect to handle audio
/// and motor support while letting OpenNI have access to the cameras.
/// If a device is not supported on a particular platform, its flag will be ignored.
type DeviceFlags C.freenect_device_flags

const (
	DeviceMotor  = DeviceFlags(C.FREENECT_DEVICE_MOTOR)
	DeviceCamera = DeviceFlags(C.FREENECT_DEVICE_CAMERA)
	DeviceAudio  = DeviceFlags(C.FREENECT_DEVICE_AUDIO)
)

/// Enumeration of available resolutions.
/// Not all available resolutions are actually supported for all video formats.
/// Frame modes may not perfectly match resolutions.  For instance,
/// FREENECT_RESOLUTION_MEDIUM is 640x488 for the IR camera.
type Resolution C.freenect_resolution

const (
	ResolutionLow    = Resolution(C.FREENECT_RESOLUTION_LOW)    /**< QVGA - 320x240 */
	ResolutionMedium = Resolution(C.FREENECT_RESOLUTION_MEDIUM) /**< VGA  - 640x480 */
	ResolutionHigh   = Resolution(C.FREENECT_RESOLUTION_HIGH)   /**< SXGA - 1280x1024 */
)

/// Enumeration of video frame information states.
/// See http://openkinect.org/wiki/Protocol_Documentation#RGB_Camera for more information.
type VideoFormat C.freenect_video_format

const (
	VideoRgb           = VideoFormat(C.FREENECT_VIDEO_RGB)             /**< Decompressed RGB mode (demosaicing done by libfreenect) */
	VideoBayer         = VideoFormat(C.FREENECT_VIDEO_BAYER)           /**< Bayer compressed mode (raw information from camera) */
	VideoIr8Bit        = VideoFormat(C.FREENECT_VIDEO_IR_8BIT)         /**< 8-bit IR mode  */
	VideoIr10Bit       = VideoFormat(C.FREENECT_VIDEO_IR_10BIT)        /**< 10-bit IR mode */
	VideoIr10BitPacked = VideoFormat(C.FREENECT_VIDEO_IR_10BIT_PACKED) /**< 10-bit packed IR mode */
	VideoYuvRgb        = VideoFormat(C.FREENECT_VIDEO_YUV_RGB)         /**< YUV RGB mode */
	VideoYuvRaw        = VideoFormat(C.FREENECT_VIDEO_YUV_RAW)         /**< YUV Raw mode */
)

/// Enumeration of depth frame states
/// See http://openkinect.org/wiki/Protocol_Documentation#RGB_Camera for more information.
type DepthFormat C.freenect_depth_format

const (
	Depth11Bit       = DepthFormat(C.FREENECT_DEPTH_11BIT)        /**< 11 bit depth information in one uint16_t/pixel */
	Depth10Bit       = DepthFormat(C.FREENECT_DEPTH_10BIT)        /**< 10 bit depth information in one uint16_t/pixel */
	Depth11BitPacked = DepthFormat(C.FREENECT_DEPTH_11BIT_PACKED) /**< 11 bit packed depth information */
	Depth10PitPacked = DepthFormat(C.FREENECT_DEPTH_10BIT_PACKED) /**< 10 bit packed depth information */
	DepthRegistered  = DepthFormat(C.FREENECT_DEPTH_REGISTERED)   /**< processed depth data in mm, aligned to 640x480 RGB */
	Depth_mm         = DepthFormat(C.FREENECT_DEPTH_MM)           /**< depth to each pixel in mm, but left unaligned to RGB image */
)

/// Enumeration of flags to toggle features with freenect_set_flag()
type Flag C.freenect_flag

const (
	// values written to the CMOS register
	AutoExposure     = Flag(C.FREENECT_AUTO_EXPOSURE)
	AutoWhiteBalance = Flag(C.FREENECT_AUTO_WHITE_BALANCE)
	RawColor         = Flag(C.FREENECT_RAW_COLOR)
	MirrorDepth      = Flag(C.FREENECT_MIRROR_DEPTH)
	MirrorVideo      = Flag(C.FREENECT_MIRROR_VIDEO)
)

/// Structure to give information about the width, height, bitrate,
/// framerate, and buffer size of a frame in a particular mode, as
/// well as the total number of bytes needed to hold a single frame.
type FrameMode struct {
	reserved               uint32
	Resolution             uint32
	anon0                  [4]byte
	Bytes                  int32
	Width                  int16
	Height                 int16
	Data_bits_per_pixel    int8
	Padding_bits_per_pixel int8
	Framerate              int8
	Is_valid               int8
}

func (this *FrameMode) ptr() *C.freenect_frame_mode {
	return (*C.freenect_frame_mode)(unsafe.Pointer(this))
}

func (this *FrameMode) VideoFormat() VideoFormat {
	return *(*VideoFormat)(unsafe.Pointer(&this.anon0))
}

func (this *FrameMode) SetVideoFormat(videoFormat VideoFormat) {
	*(*VideoFormat)(unsafe.Pointer(&this.anon0)) = videoFormat
}

func (this *FrameMode) DepthFormat() DepthFormat {
	return *(*DepthFormat)(unsafe.Pointer(&this.anon0))
}

func (this *FrameMode) SetDepthFormat(depthFormat DepthFormat) {
	*(*DepthFormat)(unsafe.Pointer(&this.anon0)) = depthFormat
}

/// Enumeration of LED states
/// See http://openkinect.org/wiki/Protocol_Documentation#Setting_LED for more information.
type LedOptions C.freenect_led_options

const (
	LedOff            = LedOptions(C.LED_OFF)              /**< Turn LED off */
	LedGreen          = LedOptions(C.LED_GREEN)            /**< Turn LED to Green */
	LedRed            = LedOptions(C.LED_RED)              /**< Turn LED to Red */
	LedYellow         = LedOptions(C.LED_YELLOW)           /**< Turn LED to Yellow */
	LedBlinkGreen     = LedOptions(C.LED_BLINK_GREEN)      /**< Make LED blink Green */
	LedBlinkRedYellow = LedOptions(C.LED_BLINK_RED_YELLOW) /**< Make LED blink Red/Yellow */
)

/// Enumeration of tilt motor status
type TiltStatusCode C.freenect_tilt_status_code

const (
	StatusStopped = TiltStatusCode(C.TILT_STATUS_STOPPED) /**< Tilt motor is stopped */
	StatusLimit   = TiltStatusCode(C.TILT_STATUS_LIMIT)   /**< Tilt motor has reached movement limit */
	StatusMoving  = TiltStatusCode(C.TILT_STATUS_MOVING)  /**< Tilt motor is currently moving to new position */
)

type RawTiltState struct {
	Accelerometer_x int16
	Accelerometer_y int16
	Accelerometer_z int16
	Tilt_angle      int8
	Pad_cgo_0       [1]byte
	Tilt_status     uint32
}

func (this *RawTiltState) ptr() *C.freenect_raw_tilt_state {
	return (*C.freenect_raw_tilt_state)(unsafe.Pointer(this))
}

type Context struct{}
type Device struct{}
type UsbContext struct{}

func (this *Context) ptr() *C.freenect_context {
	return (*C.freenect_context)(unsafe.Pointer(this))
}

func (this *Device) ptr() *C.freenect_device {
	return (*C.freenect_device)(unsafe.Pointer(this))
}

func (this *UsbContext) ptr() *C.freenect_usb_context {
	return (*C.freenect_usb_context)(unsafe.Pointer(this))
}

/// Enumeration of message logging levels
type LogLevel C.freenect_loglevel

const (
	LogFatal   = LogLevel(C.FREENECT_LOG_FATAL)   /**< Log for crashing/non-recoverable errors */
	LogError   = LogLevel(C.FREENECT_LOG_ERROR)   /**< Log for major errors */
	LogWarning = LogLevel(C.FREENECT_LOG_WARNING) /**< Log for warning messages */
	LogNotice  = LogLevel(C.FREENECT_LOG_NOTICE)  /**< Log for important messages */
	LogInfo    = LogLevel(C.FREENECT_LOG_INFO)    /**< Log for normal messages */
	LogDebug   = LogLevel(C.FREENECT_LOG_DEBUG)   /**< Log for useful development messages */
	LogSpew    = LogLevel(C.FREENECT_LOG_SPEW)    /**< Log for slightly less useful messages */
	LogFlood   = LogLevel(C.FREENECT_LOG_FLOOD)   /**< Log EVERYTHING. May slow performance. */
)

/**
 * Initialize a freenect context and do any setup required for
 * platform specific USB libraries.
 *
 * @param ctx Address of pointer to freenect context struct to allocate and initialize
 * @param usb_ctx USB context to initialize. Can be NULL if not using multiple contexts.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_init(freenect_context **ctx, freenect_usb_context *usb_ctx);
func Init(usbContext *UsbContext) (context *Context, status int) {
	ctx := (**C.freenect_context)(unsafe.Pointer(&context))
	status = int(C.freenect_init(ctx, unsafe.Pointer(usbContext)))
	return
}

/**
 * Closes the device if it is open, and frees the context
 *
 * @param ctx freenect context to close/free
 *
 * @return 0 on success
 */
//FREENECTAPI int freenect_shutdown(freenect_context *ctx);
func (ctx *Context) Shutdown() int {
	return int(C.freenect_shutdown(ctx.ptr()))
}

/// Typedef for logging callback functions
//typedef void (*freenect_log_cb)(freenect_context *dev, freenect_loglevel level, const char *msg);
type LogCb func(*Context, LogLevel, string)

/**
 * Set the log level for the specified freenect context
 *
 * @param ctx context to set log level for
 * @param level log level to use (see freenect_loglevel enum)
 */
//FREENECTAPI void freenect_set_log_level(freenect_context *ctx, freenect_loglevel level);
func (ctx *Context) SetLogLevel(level LogLevel) {
	C.freenect_set_log_level(ctx.ptr(), C.freenect_loglevel(level))
}

/**
 * Callback for log messages (i.e. for rerouting to a file instead of
 * stdout)
 *
 * @param ctx context to set log callback for
 * @param cb callback function pointer
 */
//TODO
//FREENECTAPI void freenect_set_log_callback(freenect_context *ctx, freenect_log_cb cb);
/*
//export callLogCallback
func callLogCallback(dev *C.freenect_context, level C.freenect_loglevel, msg *C.char) {
	ctx := (*Context)(unsafe.Pointer(dev))
	lvl := LogLevel(level)
	gmsg := C.GoString(msg)
	logCallback(ctx, lvl, gmsg)
}

var logCallback LogCb

func (ctx *Context) SetLogCallback(cb LogCb) {
	logCallback = cb
	C.freenect_set_log_callback(ctx.ptr(), C.callLogCallback)
}
*/

/**
 * Calls the platform specific usb event processor
 *
 * @param ctx context to process events for
 *
 * @return 0 on success, other values on error, platform/library dependant
 */
//FREENECTAPI int freenect_process_events(freenect_context *ctx);
func (ctx *Context) ProcessEvents() int {
	return int(C.freenect_process_events(ctx.ptr()))
}

/**
 * Calls the platform specific usb event processor until either an event occurs
 * or the timeout parameter time has passed.  If a zero timeval is passed, this
 * function will handle any already-pending events, then return immediately.
 *
 * @param ctx Context to process events for
 * @param timeout Pointer to a timeval containing the maximum amount of time to block waiting for events, or zero for nonblocking mode
 *
 * @return 0 on success, other values on error, platform/library dependant
 */
//TODO
//FREENECTAPI int freenect_process_events_timeout(freenect_context *ctx, struct timeval* timeout);

/**
 * Return the number of kinect devices currently connected to the
 * system
 *
 * @param ctx Context to access device count through
 *
 * @return Number of devices connected, < 0 on error
 */
//FREENECTAPI int freenect_num_devices(freenect_context *ctx);
func (ctx *Context) NumDevices() int {
	return int(C.freenect_num_devices(ctx.ptr()))
}

/**
 * Scans for kinect devices and produces a linked list of their attributes
 * (namely, serial numbers), returning the number of devices.
 *
 * @param ctx Context to scan for kinect devices with
 * @param attribute_list Pointer to where this function will store the resultant linked list
 *
 * @return Number of devices connected, < 0 on error
 */
//TODO
//FREENECTAPI int freenect_list_device_attributes(freenect_context *ctx, struct freenect_device_attributes** attribute_list);

/**
 * Free the linked list produced by freenect_list_device_attributes().
 *
 * @param attribute_list Linked list of attributes to free.
 */
//TODO
//FREENECTAPI void freenect_free_device_attributes(struct freenect_device_attributes* attribute_list);

/**
 * Answer which subdevices this library supports.  This is most useful for
 * wrappers trying to determine whether the underlying library was built with
 * audio support or not, so the wrapper can avoid calling functions that do not
 * exist.
 *
 * @return Flags representing the subdevices that the library supports opening (see freenect_device_flags)
 */
//FREENECTAPI int freenect_supported_subdevices(void);
func (ctx *Context) SupportedSubdevices() int {
	return int(C.freenect_supported_subdevices())
}

/**
 * Set which subdevices any subsequent calls to freenect_open_device()
 * should open.  This will not affect devices which have already been
 * opened.  The default behavior, should you choose not to call this
 * function at all, is to open all supported subdevices - motor, cameras,
 * and audio, if supported on the platform.
 *
 * @param ctx Context to set future subdevice selection for
 * @param subdevs Flags representing the subdevices to select
 */
//FREENECTAPI void freenect_select_subdevices(freenect_context *ctx, freenect_device_flags subdevs);
func (ctx *Context) SelectSubdevices(subdevs DeviceFlags) {
	C.freenect_select_subdevices(ctx.ptr(), C.freenect_device_flags(subdevs))
}

/**
 * Returns the devices that are enabled after calls to freenect_open_device()
 * On newer kinects the motor and audio are automatically disabled for now
 *
 * @param ctx Context to set future subdevice selection for
 * @return Flags representing the subdevices that were actually opened (see freenect_device_flags)
 */
//FREENECTAPI freenect_device_flags freenect_enabled_subdevices(freenect_context *ctx);
func (ctx *Context) EnabledSubdevices() DeviceFlags {
	return DeviceFlags(C.freenect_enabled_subdevices(ctx.ptr()))
}

/**
 * Opens a kinect device via a context. Index specifies the index of
 * the device on the current state of the bus. Bus resets may cause
 * indexes to shift.
 *
 * @param ctx Context to open device through
 * @param dev Device structure to assign opened device to
 * @param index Index of the device on the bus
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_open_device(freenect_context *ctx, freenect_device **dev, int index);
func (ctx *Context) OpenDevices(index int) (dev *Device, status int) {
	status = int(C.freenect_open_device(ctx.ptr(), (**C.freenect_device)(unsafe.Pointer(&dev)), C.int(index)))
	return dev, status
}

/**
 * Opens a kinect device (via a context) associated with a particular camera
 * subdevice serial number.  This function will fail if no device with a
 * matching serial number is found.
 *
 * @param ctx Context to open device through
 * @param dev Device structure to assign opened device to
 * @param camera_serial Null-terminated ASCII string containing the serial number of the camera subdevice in the device to open
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_open_device_by_camera_serial(freenect_context *ctx, freenect_device **dev, const char* camera_serial);
func (ctx *Context) OpenDeviceByCameraSerialindex(camera_serial string) (dev *Device, status int) {
	cs := C.CString(camera_serial)
	defer C.free(unsafe.Pointer(cs))
	status = int(C.freenect_open_device_by_camera_serial(ctx.ptr(), (**C.freenect_device)(unsafe.Pointer(&dev)), cs))
	return dev, status
}

/**
 * Closes a device that is currently open
 *
 * @param dev Device to close
 *
 * @return 0 on success
 */
//FREENECTAPI int freenect_close_device(freenect_device *dev);
func (dev *Device) Close() int {
	return int(C.freenect_close_device(dev.ptr()))
}

/**
 * Set the device user data, for passing generic information into
 * callbacks
 *
 * @param dev Device to attach user data to
 * @param user User data to attach
 */
//FREENECTAPI void freenect_set_user(freenect_device *dev, void *user);
func (dev *Device) SetUser(user unsafe.Pointer) {
	C.freenect_set_user(dev.ptr(), user)
}

/**
 * Retrieve the pointer to user data from the device struct
 *
 * @param dev Device from which to get user data
 *
 * @return Pointer to user data
 */
//FREENECTAPI void *freenect_get_user(freenect_device *dev);
func (dev *Device) GetUser() unsafe.Pointer {
	return C.freenect_get_user(dev.ptr())
}

/// Typedef for depth image received event callbacks
// typedef void (*freenect_depth_cb)(freenect_device *dev, void *depth, uint32_t timestamp);
type DepthCb func(dev *Device, depth unsafe.Pointer, timestamp uint32)

/// Typedef for video image received event callbacks
//typedef void (*freenect_video_cb)(freenect_device *dev, void *video, uint32_t timestamp);
type VideoCb func(dev *Device, video unsafe.Pointer, timestamp uint32)

/// Typedef for stream chunk processing callbacks
//typedef void (*freenect_chunk_cb)(void *buffer, void *pkt_data, int pkt_num, int datalen, void *user_data);
type ChunkCb func(buffer, pkt_data unsafe.Pointer, pkt_num, datalen int /*, user_data unsafe.Pointer*/)

/**
 * Set callback for depth information received event
 *
 * @param dev Device to set callback for
 * @param cb Function pointer for processing depth information
 */
//TODO
//FREENECTAPI void freenect_set_depth_callback(freenect_device *dev, freenect_depth_cb cb);
//func (dev *Device) SetDepthCallback(cb DepthCb)

/**
 * Set callback for video information received event
 *
 * @param dev Device to set callback for
 * @param cb Function pointer for processing video information
 */
//TODO
//FREENECTAPI void freenect_set_video_callback(freenect_device *dev, freenect_video_cb cb);
//func (dev *Device) SetVideoCallback(cb VideoCb)

/**
 * Set callback for depth chunk processing
 *
 * @param dev Device to set callback for
 * @param cb Function pointer for processing depth chunk
 */
//TODO
//FREENECTAPI void freenect_set_depth_chunk_callback(freenect_device *dev, freenect_chunk_cb cb);
//func (dev *Device) SetDepthChunkCallback(cb ChunkCb)

/**
 * Set callback for video chunk processing
 *
 * @param dev Device to set callback for
 * @param cb Function pointer for processing video chunk
 */
//TODO
//FREENECTAPI void freenect_set_video_chunk_callback(freenect_device *dev, freenect_chunk_cb cb);
//func (dev *Device) setVideoChunkCallback(cb ChunkCb)

/**
 * Set the buffer to store depth information to. Size of buffer is
 * dependant on depth format. See FREENECT_DEPTH_*_SIZE defines for
 * more information.
 *
 * @param dev Device to set depth buffer for.
 * @param buf Buffer to store depth information to.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_set_depth_buffer(freenect_device *dev, void *buf);
func (dev *Device) SetDepthBuffer(buf unsafe.Pointer) int {
	return int(C.freenect_set_depth_buffer(dev.ptr(), buf))
}

/**
 * Set the buffer to store depth information to. Size of buffer is
 * dependant on video format. See FREENECT_VIDEO_*_SIZE defines for
 * more information.
 *
 * @param dev Device to set video buffer for.
 * @param buf Buffer to store video information to.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_set_video_buffer(freenect_device *dev, void *buf);
func (dev *Device) SetVideobuffer(buf unsafe.Pointer) int {
	return int(C.freenect_set_video_buffer(dev.ptr(), buf))
}

/**
 * Start the depth information stream for a device.
 *
 * @param dev Device to start depth information stream for.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_start_depth(freenect_device *dev);
func (dev *Device) StartDepth() int {
	return int(C.freenect_start_depth(dev.ptr()))
}

/**
 * Start the video information stream for a device.
 *
 * @param dev Device to start video information stream for.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_start_video(freenect_device *dev);
func (dev *Device) StartVideo() int {
	return int(C.freenect_start_video(dev.ptr()))
}

/**
 * Stop the depth information stream for a device
 *
 * @param dev Device to stop depth information stream on.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_stop_depth(freenect_device *dev);
func (dev *Device) StopDepth() int {
	return int(C.freenect_stop_depth(dev.ptr()))
}

/**
 * Stop the video information stream for a device
 *
 * @param dev Device to stop video information stream on.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_stop_video(freenect_device *dev);
func (dev *Device) StopVideo() int {
	return int(C.freenect_stop_video(dev.ptr()))
}

/**
 * Updates the accelerometer state using a blocking control message
 * call.
 *
 * @param dev Device to get accelerometer data from
 *
 * @return 0 on success, < 0 on error. Accelerometer data stored to
 * device struct.
 */
//FREENECTAPI int freenect_update_tilt_state(freenect_device *dev);
func (dev *Device) UpdateTiltState() int {
	return int(C.freenect_update_tilt_state(dev.ptr()))
}

/**
 * Retrieve the tilt state from a device
 *
 * @param dev Device to retrieve tilt state from
 *
 * @return The tilt state struct of the device
 */
//FREENECTAPI freenect_raw_tilt_state* freenect_get_tilt_state(freenect_device *dev);
func (dev *Device) GetTiltState() *RawTiltState {
	return (*RawTiltState)(unsafe.Pointer(C.freenect_get_tilt_state(dev.ptr())))
}

/**
 * Return the tilt state, in degrees with respect to the horizon
 *
 * @param state The tilt state struct from a device
 *
 * @return Current degree of tilt of the device
 */
//FREENECTAPI double freenect_get_tilt_degs(freenect_raw_tilt_state *state);
func (state *RawTiltState) GetTiltDegs() float64 {
	return float64(C.freenect_get_tilt_degs(state.ptr()))
}

/**
 * Set the tilt state of the device, in degrees with respect to the
 * horizon. Uses blocking control message call to update
 * device. Function return does not reflect state of device, device
 * may still be moving to new position after the function returns. Use
 * freenect_get_tilt_status() to find current movement state.
 *
 * @param dev Device to set tilt state
 * @param angle Angle the device should tilt to
 *
 * @return 0 on success, < 0 on error.
 */
//FREENECTAPI int freenect_set_tilt_degs(freenect_device *dev, double angle);
func (dev *Device) SetTiltDegs(angle float64) int {
	return int(C.freenect_set_tilt_degs(dev.ptr(), C.double(angle)))
}

/**
 * Return the movement state of the tilt motor (moving, stopped, etc...)
 *
 * @param state Raw state struct to get the tilt status code from
 *
 * @return Status code of the tilt device. See
 * freenect_tilt_status_code enum for more info.
 */
//FREENECTAPI freenect_tilt_status_code freenect_get_tilt_status(freenect_raw_tilt_state *state);
func (state *RawTiltState) GetTiltStatus() TiltStatusCode {
	return TiltStatusCode(C.freenect_get_tilt_status(state.ptr()))
}

/**
 * Set the state of the LED. Uses blocking control message call to
 * update device.
 *
 * @param dev Device to set the LED state
 * @param option LED state to set on device. See freenect_led_options enum.
 *
 * @return 0 on success, < 0 on error
 */
//FREENECTAPI int freenect_set_led(freenect_device *dev, freenect_led_options option);
func (dev *Device) SetLed(option LedOptions) int {
	return int(C.freenect_set_led(dev.ptr(), C.freenect_led_options(option)))
}

/**
 * Get the axis-based gravity adjusted accelerometer state, as laid
 * out via the accelerometer data sheet, which is available at
 *
 * http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf
 *
 * @param state State to extract accelerometer data from
 * @param x Stores X-axis accelerometer state
 * @param y Stores Y-axis accelerometer state
 * @param z Stores Z-axis accelerometer state
 */
//FREENECTAPI void freenect_get_mks_accel(freenect_raw_tilt_state *state, double* x, double* y, double* z);
func (this *RawTiltState) GetMksAccel() (x, y, z float64) {
	C.freenect_get_mks_accel(this.ptr(), (*C.double)(&x), (*C.double)(&y), (*C.double)(&z))
	return
}

/**
 * Get the number of video camera modes supported by the driver.  This includes both RGB and IR modes.
 *
 * @return Number of video modes supported by the driver
 */
//FREENECTAPI int freenect_get_video_mode_count();
func GetVideoModeCount() int {
	return int(C.freenect_get_video_mode_count())
}

/**
 * Get the frame descriptor of the nth supported video mode for the
 * video camera.
 *
 * @param mode_num Which of the supported modes to return information about
 *
 * @return A freenect_frame_mode describing the nth video mode
 */
//FREENECTAPI freenect_frame_mode freenect_get_video_mode(int mode_num);
func GetVideoMode(mode_num int) FrameMode {
	fm := C.freenect_get_video_mode(C.int(mode_num))
	return *(*FrameMode)(unsafe.Pointer(&fm))
}

/**
 * Get the frame descriptor of the current video mode for the specified
 * freenect device.
 *
 * @param dev Which device to return the currently-set video mode for
 *
 * @return A freenect_frame_mode describing the current video mode of the specified device
 */
//FREENECTAPI freenect_frame_mode freenect_get_current_video_mode(freenect_device *dev);
func (dev *Device) GetCurrentVideoMode() FrameMode {
	fm := C.freenect_get_current_video_mode(dev.ptr())
	return *(*FrameMode)(unsafe.Pointer(&fm))
}

/**
 * Convenience function to return a mode descriptor matching the
 * specified resolution and video camera pixel format, if one exists.
 *
 * @param res Resolution desired
 * @param fmt Pixel format desired
 *
 * @return A freenect_frame_mode that matches the arguments specified, if such a valid mode exists; otherwise, an invalid freenect_frame_mode.
 */
//FREENECTAPI freenect_frame_mode freenect_find_video_mode(freenect_resolution res, freenect_video_format fmt);
func FindVideoMode(res Resolution, format VideoFormat) FrameMode {
	fm := C.freenect_find_video_mode(C.freenect_resolution(res), C.freenect_video_format(format))
	return *(*FrameMode)(unsafe.Pointer(&fm))
}

/**
 * Sets the current video mode for the specified device.  If the
 * freenect_frame_mode specified is not one provided by the driver
 * e.g. from freenect_get_video_mode() or freenect_find_video_mode()
 * then behavior is undefined.  The current video mode cannot be
 * changed while streaming is active.
 *
 * @param dev Device for which to set the video mode
 * @param mode Frame mode to set
 *
 * @return 0 on success, < 0 if error
 */
//FREENECTAPI int freenect_set_video_mode(freenect_device* dev, freenect_frame_mode mode);
func (dev *Device) SetVideoMode(mode FrameMode) int {
	return int(C.freenect_set_video_mode(dev.ptr(), *mode.ptr()))
}

/**
 * Get the number of depth camera modes supported by the driver.  This includes both RGB and IR modes.
 *
 * @return Number of depth modes supported by the driver
 */
//FREENECTAPI int freenect_get_depth_mode_count();
func GetDepthModeCount() int {
	return int(C.freenect_get_depth_mode_count())
}

/**
 * Get the frame descriptor of the nth supported depth mode for the
 * depth camera.
 *
 * @param mode_num Which of the supported modes to return information about
 *
 * @return A freenect_frame_mode describing the nth depth mode
 */
//FREENECTAPI freenect_frame_mode freenect_get_depth_mode(int mode_num);
func GetDepthMode(mode_num int) FrameMode {
	fm := C.freenect_get_depth_mode(C.int(mode_num))
	return *(*FrameMode)(unsafe.Pointer(&fm))
}

/**
 * Get the frame descriptor of the current depth mode for the specified
 * freenect device.
 *
 * @param dev Which device to return the currently-set depth mode for
 *
 * @return A freenect_frame_mode describing the current depth mode of the specified device
 */
//FREENECTAPI freenect_frame_mode freenect_get_current_depth_mode(freenect_device *dev);
func (dev *Device) GetCurrentDepthMode() FrameMode {
	fm := C.freenect_get_current_depth_mode(dev.ptr())
	return *(*FrameMode)(unsafe.Pointer(&fm))
}

/**
 * Convenience function to return a mode descriptor matching the
 * specified resolution and depth camera pixel format, if one exists.
 *
 * @param res Resolution desired
 * @param fmt Pixel format desired
 *
 * @return A freenect_frame_mode that matches the arguments specified, if such a valid mode exists; otherwise, an invalid freenect_frame_mode.
 */
//FREENECTAPI freenect_frame_mode freenect_find_depth_mode(freenect_resolution res, freenect_depth_format fmt);
func FindDepthMode(res Resolution, format DepthFormat) FrameMode {
	fm := C.freenect_find_depth_mode(C.freenect_resolution(res), C.freenect_depth_format(format))
	return *(*FrameMode)(unsafe.Pointer(&fm))
}

/**
 * Sets the current depth mode for the specified device.  The mode
 * cannot be changed while streaming is active.
 *
 * @param dev Device for which to set the depth mode
 * @param mode Frame mode to set
 *
 * @return 0 on success, < 0 if error
 */
//FREENECTAPI int freenect_set_depth_mode(freenect_device* dev, const freenect_frame_mode mode);
func (dev *Device) SetDepthMode(mode FrameMode) int {
	return int(C.freenect_set_depth_mode(dev.ptr(), *mode.ptr()))
}

/**
 * Enables or disables the specified flag.
 *
 * @param flag Feature to set
 * @param value `FREENECT_OFF` or `FREENECT_ON`
 *
 * @return 0 on success, < 0 if error
 */
func (dev Device) SetFlag(flag Flag, value bool) int {
	if value {
		return int(C.freenect_set_flag(dev.ptr(), C.freenect_flag(flag), C.FREENECT_ON))
	} else {
		return int(C.freenect_set_flag(dev.ptr(), C.freenect_flag(flag), C.FREENECT_OFF))
	}
}

/**
 * Allows the user to specify a pointer to the audio firmware in memory for the Xbox 360 Kinect
 *
 * @param ctx Context to open device through
 * @param fw_ptr Pointer to audio firmware loaded in memory
 * @param num_bytes The size of the firmware in bytes
 */
//FREENECTAPI void freenect_set_fw_address_nui(freenect_context * ctx, unsigned char * fw_ptr, unsigned int num_bytes);
func (ctx *Context) SetFwAddressNui(fw_ptr *byte, numBytes int) {
	C.freenect_set_fw_address_nui(ctx.ptr(), (*C.uchar)(fw_ptr), C.uint(numBytes))
}

/**
 * Allows the user to specify a pointer to the audio firmware in memory for the K4W Kinect
 *
 * @param ctx Context to open device through
 * @param fw_ptr Pointer to audio firmware loaded in memory
 * @param num_bytes The size of the firmware in bytes
 */
//FREENECTAPI void freenect_set_fw_address_k4w(freenect_context * ctx, unsigned char * fw_ptr, unsigned int num_bytes);
func (ctx *Context) SetFwAddressK4w(fw_ptr *byte, numBytes int) {
	C.freenect_set_fw_address_k4w(ctx.ptr(), (*C.uchar)(fw_ptr), C.uint(numBytes))
}
