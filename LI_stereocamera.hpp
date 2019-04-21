#ifndef ROBOT_STEREO_CAMERA_H
#define ROBOT_STEREO_CAMERA_H

// uncomment to disable assert()
// #define NDEBUG
#include <cassert>
#include <cstring>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
// #include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/imgcodecs/imgcodecs.hpp>
// #include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui_c.h>

#ifndef _GLIBCXX_STRING
#error "LI_stereocamera relies on the String module of the standard C++ library."
#endif

#ifndef LIBUVC_H
#error "LI_stereocamera depends on the libuvc library (a library for communicating with USB video devices)."
#endif

#if LIBUSB_API_VERSION < 0x01000102
#error "LI_stereocamera relies on the hotplug feature of libusb-1.0" \
  " (available in version 1.0.16 and above of the library)."
#endif

enum LI_error_t {
  LI_SUCCESS = 0,
  LI_UNSUPPORTED_PLATFORM = 1,
  LI_CAMERA_UNPLUGGED = 2,
  LI_SUDO_NEEDED = 3,
  LI_UNABLE_TO_ALLOCATE_FRAME = 4,
  LI_UNABLE_TO_CONVERT_FRAME = 5,
  LI_UNSUPPORTED_CAMERA_MODE = 6,
  LI_UNSPECIFIED = 99
};

class LI_exception { 
  libusb_error code_usb;
  uvc_error_t  code_uvc;
  LI_error_t   code_other;
  
public:
  LI_exception() : 
    code_usb(LIBUSB_SUCCESS),
    code_uvc(UVC_SUCCESS),
    code_other(LI_SUCCESS) {
         
  }
  
  // Silent constructors, they fill in the relevant error codes without
  // generating an error text or raising an exception.
  LI_exception(libusb_error error_code) : LI_exception() {
    this->code_usb = error_code;
  }
  LI_exception(uvc_error_t error_code) : LI_exception() {
    this->code_uvc = error_code;
  }
  LI_exception(LI_error_t error_code) : LI_exception() {
    this->code_other = error_code;
  }
  
  // This is a loud copy constructor, it will automatically generate an
  // error text and raise an exception.
  LI_exception(const LI_exception& instance) : LI_exception() {
    *this = instance.code_other;
    *this = instance.code_uvc;
    *this = instance.code_usb;
  }
  
  operator bool() const {
    return (LIBUSB_SUCCESS != this->code_usb) ||
           (UVC_SUCCESS != this->code_uvc) ||
            (LI_SUCCESS != this->code_other);
  }
  
  // Those are mere casting operators but do not allow changing
  // the object. For that purpose assignment operator overloads
  // are provided.
  operator const libusb_error&() const {
    return (this->code_usb);
  }
  
  operator const uvc_error_t&() const {
    return (this->code_uvc);
  }
  
  operator const LI_error_t&() const {
    return (this->code_other);
  }
  
  // Loud assignment operators, will automatically raise an
  // exception.
  bool operator=(libusb_error error_code) {
    this->code_usb = error_code;
    
    if ((bool) *this)
      throw this;
    return (bool) *this;
  }
  
  bool operator=(uvc_error_t error_code) {
    this->code_uvc = error_code;
    
    if ((bool) *this)
      throw this;
    return (bool) *this;
  }
  
  bool operator=(LI_error_t error_code) {
    this->code_other = error_code;
    
    if ((bool) *this)
      throw this;
    return (bool) *this;
  }
  
  const char* error_message() const {
    if (LI_SUCCESS != this->code_other) {
      switch (this->code_other) {
        case LI_UNSUPPORTED_PLATFORM:
          return "Unsupported platform";
        case LI_CAMERA_UNPLUGGED:
          return "LI Stereo Camera is not connected";
        case LI_SUDO_NEEDED:
          return "Couldn't access the camera (please run using 'sudo')";
        case LI_UNABLE_TO_ALLOCATE_FRAME:
          return "Unable to allocate frame";
        case LI_UNABLE_TO_CONVERT_FRAME:
          return "Unable to convert frame";
        case LI_UNSUPPORTED_CAMERA_MODE:
          return "The camera did not support the requested video size"
            " and/or frame rate.";
        default:
          return "Unspecified error";
      }
    } 
    else if (UVC_SUCCESS != this->code_uvc)
      return uvc_strerror(this->code_uvc);
    else if (LIBUSB_SUCCESS != this->code_usb)
      return libusb_strerror(this->code_usb);
    else
      return "Success";
  }
  
  const char* error_category() const {
    if (LI_SUCCESS != this->code_other)
      return "[LISTEREO]";
    else if (UVC_SUCCESS != this->code_uvc)
      return "[LIBUVC]";
    else if (LIBUSB_SUCCESS != this->code_usb)
      return "[LIBUSB]";
    else
      return "[UNKNOWN]";
  }
  
  int error_code() const {
    if (LI_SUCCESS != this->code_other)
      return (int) this->code_other;
    else if (UVC_SUCCESS != this->code_uvc)
      return (int) this->code_uvc;
    else if (LIBUSB_SUCCESS != this->code_usb)
      return (int) this->code_usb;
    else
      return 0;
  }
};

std::ostream& operator<<(std::ostream& ostream, const LI_exception* error){
  ostream << error->error_category() << " " << 
    error->error_message() << " (" << error->error_code() << ")";
  return ostream;
}
std::ostream& operator<<(std::ostream& ostream, const LI_exception& error){
  ostream << error.error_category()  << " " << 
    error.error_message() << " (" << error.error_code() << ")";
  return ostream;
}

/*
 * Leopard Imaging Stereo Camera class
 */
class LI_stereocamera {

  static const int WIDTH = 640, HEIGHT = 480, FPS = 30;
  
  bool connected = false;
  
  libusb_context *usb_context;
  uvc_context_t *uvc_context;
  
  libusb_device *usb_device;
  uvc_device_t *uvc_device;
  
  uvc_device_handle_t *uvc_handle;
  uvc_stream_ctrl_t uvc_stream;
  
  // This class extends the error reporting functionality provided by 
  // the libuvc library, namely by adding a secondary error code of the
  // enumerated type LI_error_t when the primary (i.e. UVC) error code
  // is equal to UVC_ERROR_OTHER.
  LI_exception error;

  // The hotplug callback is called automatically whenever a device is
  // plugged or unplugged. If registered with the flag LIBUSB_HOTPLUG_
  // NO_FLAGS then will only be called for any plugging or unplugging
  // of devices after the program started. If registered with LIBUSB_
  // HOTPLUG_ENUMERATE, on the other hand, then will result in an 
  // emulated plugging of all the currently connected devices. For our
  // purposes, we use the default flag: LIBUSB_HOTPLUG_NO_FLAGS.
  static int hotplug_callback(
    libusb_context *context, 
    libusb_device *device,
    libusb_hotplug_event event, 
    void *user_data) {
    
    // This is a substitute for the "this" pointer in a class method.
    LI_stereocamera *self = (LI_stereocamera*) user_data;
    
    // This method is responsible of maintaining a valid handle to the
    // Leopard Imaging Stereo Camera.
    self->update_connection();
    
    // A return value of '0' tells the libusb library that we are
    // still interested in getting hotplug notifications, as opposed
    // to a return value of '1' which will cease the notifications.
    return 0;
  }
  
  // The frame callback is used to process frames from the camera
  static void frame_callback(uvc_frame *frame, void *user_data) {
    
    // This is the equivalent of "this" in a nonstatic class method
    LI_stereocamera *self = (LI_stereocamera*) user_data;
    
    frame->frame_format = UVC_FRAME_FORMAT_YUYV;

    uvc_frame_t *greyLeft;
    uvc_frame_t *greyRight;

    /* We'll convert the image from YUV/JPEG to gray8, so allocate space */
    greyLeft = uvc_allocate_frame(frame->width * frame->height);
    greyRight = uvc_allocate_frame(frame->width * frame->height);
    if (!greyLeft || !greyRight) {
      self->error = LI_UNABLE_TO_ALLOCATE_FRAME;
      return;
    }

    /* Do the BGR conversion */
    LI_exception resultLeft(uvc_yuyv2y(frame, greyLeft));
    LI_exception resultRight(uvc_yuyv2uv(frame, greyRight));
 
    if ((bool) resultLeft || (bool) resultRight) {
      uvc_free_frame(greyLeft);
      uvc_free_frame(greyRight);
    
      self->error = LI_UNABLE_TO_CONVERT_FRAME;
      return;
    }
  
    cv::Mat left(greyLeft->height, greyLeft->width, CV_8UC1, greyLeft->data);
    cv::Mat right(greyRight->height, greyRight->width, CV_8UC1, greyRight->data);
  
    // Process the frame
    self->error = self->process_frame(left, right);
    
    cv::imshow("Left", left);
    cv::imshow("Right", right);
    cvWaitKey(10);

    uvc_free_frame(greyLeft);
    uvc_free_frame(greyRight);
  }
  
  // Core functionality is provided here.
  LI_error_t process_frame(cv::Mat& left, cv::Mat& right) {
    
    std::string cascadeName = "./data/haarcascades/haarcascade_frontalface_alt.xml";
    std::string nestedCascadeName = "./data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

    cv::CascadeClassifier cascade, nestedCascade;
    
    if (!cascade.load(cascadeName) ||
        !nestedCascade.load(nestedCascadeName))
      return LI_UNSPECIFIED;
    
    double scale = 1;
    bool tryFlip = false;
    
    LI_error_t result1 = this->detectAndDraw(left, cascade, nestedCascade, scale, tryFlip);
    if (LI_SUCCESS != result1)
      return result1;
      
    LI_error_t result2 = this->detectAndDraw(right, cascade, nestedCascade, scale, tryFlip);
    if (LI_SUCCESS != result2)
      return result2;
    
    return LI_SUCCESS;
  }
  
  LI_error_t detectAndDraw(
    cv::Mat& img, 
    cv::CascadeClassifier& cascade,
    cv::CascadeClassifier& nestedCascade,
    double scale, bool tryflip)
  {
    int i = 0;
    double t = 0;
    std::vector<cv::Rect> faces, faces2;
    const static cv::Scalar colors[] =  {
      CV_RGB(0,0,255),
      CV_RGB(0,128,255),
      CV_RGB(0,255,255),
      CV_RGB(0,255,0),
      CV_RGB(255,128,0),
      CV_RGB(255,255,0),
      CV_RGB(255,0,0),
      CV_RGB(255,0,255)
    };
    cv::Mat gray, smallImg(cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1);
  
    //cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::resize(img, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR);
    cv::equalizeHist(smallImg, smallImg);
  
    t = (double) cvGetTickCount();
    cascade.detectMultiScale(
      smallImg, faces,
      1.1, 2, 0
      //|cv::CASCADE_FIND_BIGGEST_OBJECT
      //|cv::CASCADE_DO_ROUGH_SEARCH
      |cv::CASCADE_SCALE_IMAGE,
      cv::Size(30, 30));
      
    if(tryflip) {
      cv::flip(smallImg, smallImg, 1);
      cascade.detectMultiScale(
        smallImg, faces2,
        1.1, 2, 0
        //|cv::CASCADE_FIND_BIGGEST_OBJECT
        //|cv::CASCADE_DO_ROUGH_SEARCH
        |cv::CASCADE_SCALE_IMAGE,
        cv::Size(30, 30));
        
      for(std::vector<cv::Rect>::const_iterator r = faces2.begin(); r != faces2.end(); r++)
        faces.push_back(cv::Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
    }
      
    t = (double) cvGetTickCount() - t;
    for(std::vector<cv::Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ ) {
      cv::Mat smallImgROI;
      std::vector<cv::Rect> nestedObjects;
      cv::Point center;
      cv::Scalar color = colors[i%8];
      int radius;
  
      double aspect_ratio = (double)r->width/r->height;
      if( 0.75 < aspect_ratio && aspect_ratio < 1.3 ) {
        center.x = cvRound((r->x + r->width*0.5)*scale);
        center.y = cvRound((r->y + r->height*0.5)*scale);
        radius = cvRound((r->width + r->height)*0.25*scale);
        circle( img, center, radius, color, 3, 8, 0 );
      }
      else
        rectangle(img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
          cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
          color, 3, 8, 0);
        
      //if(nestedCascade.empty())
        continue;
        
      smallImgROI = smallImg(*r);
      nestedCascade.detectMultiScale(
        smallImgROI, nestedObjects,
        1.1, 2, 0
        //|cv::CASCADE_FIND_BIGGEST_OBJECT
        //|cv::CASCADE_DO_ROUGH_SEARCH
        //|cv::CASCADE_DO_CANNY_PRUNING
        |cv::CASCADE_SCALE_IMAGE,
        cv::Size(30, 30));
         
      for(std::vector<cv::Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++) {
        center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
        center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
        radius = cvRound((nr->width + nr->height)*0.25*scale);
        circle(img, center, radius, color, 3, 8, 0);
      }
    }
    
    return LI_SUCCESS; 
  }
  
  // Those methods get called automatically upon losing and gaining
  // connection to the LI Stereo Camera
  void on_connect(uvc_device_t *uvc_device) {
    // Make sure we have a valid library context and a passed device
    assert(this->usb_context != NULL && this->uvc_context != NULL);
    assert(NULL != uvc_device);
    
    std::cout << "LI Sereo Camera is now connected.\n";
    
    // Make sure the device passed is valid.
    if (NULL == uvc_device)
      this->error = LI_UNSPECIFIED;
    
    // Store the passed libuvc device pointer and increment its
    // reference count.
    this->uvc_device = uvc_device;
    uvc_ref_device(this->uvc_device);
    
    // Open the uvc device, obtaining a libuvc handle to it. If failed,
    // indicates insufficient permissions.
    try {
      LI_exception result;
      result = uvc_open(this->uvc_device, &this->uvc_handle);
    } catch (LI_exception *error) {
      this->error = LI_SUDO_NEEDED;
    }
    
    // Double check that the libuvc handle belongs to the device
    if (this->uvc_device != uvc_get_device(this->uvc_handle))
      this->error = LI_UNSPECIFIED;
    
    // Obtain a pointer to the underlying USB device, but do not 
    // increment its reference count.
    this->usb_device = libusb_get_device(
      uvc_get_libusb_handle(this->uvc_handle));
    
    // Negotiate an image size and a frame rate
    try {
      LI_exception result;
      result = uvc_get_stream_ctrl_format_size(
        this->uvc_handle, 
        &this->uvc_stream,     /* result stored in uvc_stream */
        UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
        LI_stereocamera::WIDTH, 
        LI_stereocamera::HEIGHT, 
        LI_stereocamera::FPS
      );
    } catch (LI_exception *error) {
      this->error = LI_UNSUPPORTED_CAMERA_MODE;
    }
   
    // Start streaming while registering a frame-processing callback
    // function, note that the callbck function is a static function
    // that is passed the object's "this" pointer in the user_data field.
    this->error = uvc_start_streaming(
      this->uvc_handle, 
      &this->uvc_stream,
      LI_stereocamera::frame_callback, /* frame callback */
      (void*) this, /* user_data */
      0 /* flags, set to zero */
    );
    
    // Activate auto-exposure.
    uvc_set_ae_mode(this->uvc_handle, 0);
    uvc_set_saturation(this->uvc_handle, 0xFFFF);
  }
  
  void on_disconnect() {
    // Make sure we have a valid library context
    assert(this->usb_context != NULL && this->uvc_context != NULL);
    
    std::cout << "Connection to LI Sereo Camera is lost...\n";
    
    // Stop streaming, since the camera was unplugged.
    uvc_stop_streaming(this->uvc_handle);
    // Reset stream control structure
    std::memset((void*) &this->uvc_stream, 0, sizeof(uvc_stream_ctrl_t));
    
    // If valid, close the libuvc handle
    if (NULL != this->uvc_handle) {
      uvc_close(this->uvc_handle);
      this->uvc_handle = NULL;
    }
    // If valid, invalidate the libusb-1.0 device, but do not
    // unreference it
    if (NULL != this->usb_device) {
      this->usb_device = NULL;
    }
    // If valid, unref the libuvc device
    if (NULL != this->uvc_device) {
      uvc_unref_device(this->uvc_device);
      this->uvc_device = NULL;
    }
  }
  
  // This function closes and re-establishes the connection to the
  // Stereo Camera in response to hot plugging/unplugging.
  void update_connection() {
    // Make sure we have a valid library context
    assert(this->usb_context != NULL && this->uvc_context != NULL);
    
    // Obtain a list of all the connected video devices.
    uvc_device_t **uvc_device_list;
    this->error = uvc_get_device_list(this->uvc_context, &uvc_device_list);
    
    // Find the Leopard Imaging Stereo Camera using its Vendor/Product
    // ID combination (2a0b:00f5) but no specific serial number (NULL).
    unsigned int i;
    bool found = false;
    for (i = 0; NULL != uvc_device_list[i]; ++i) {
      
      uvc_device_descriptor *uvc_descriptor = NULL;
      try {
        LI_exception result;
        result = uvc_get_device_descriptor(uvc_device_list[i], &uvc_descriptor);

        if ((uint16_t) 0x2A0B == uvc_descriptor->idVendor &&
            (uint16_t) 0x00F5 == uvc_descriptor->idProduct)     
          found = true;
        
        uvc_free_device_descriptor(uvc_descriptor);
      }
      catch (LI_exception *error) {
      }
      
      if (true == found)
        break;
    }
    
    // There are two possibilities here which are uninteresting to us,
    // namely when an LI Stereo camera is found but we are already
    // connected, and when an LI Stereo camera is not found but we are
    // also disconnected. In which cases there is basically nothing to
    // be done so we just deallocate the device list and end the call.
    if (found == this->connected) {
      uvc_free_device_list(uvc_device_list, 1 /* unref_devices */ );
      return;
    }
    
    // If the camera was not found, and we are connected (implicit),
    // then call OnDisconnect, change connected to false and return.
    if (false == found) {
      // Lost connection to the Stereo Camera
      this->on_disconnect();
      this->connected = false;
      
      uvc_free_device_list(uvc_device_list, 1 /* unref_devices */ );
      return;
    }
    
    // Otherwise, the stereo camera is found and we are not connected
    // to it, so we call OnConnect, change connected to true and return.
    this->on_connect(uvc_device_list[i]);
    this->connected = true;
      
    uvc_free_device_list(uvc_device_list, 1 /* unref_devices */ );
    return;
  }
  
public:
  LI_stereocamera() : 
    usb_context(NULL),
    uvc_context(NULL),
    uvc_device(NULL),
    usb_device(NULL),
    uvc_handle(NULL),
    error() {
    
    // Initialize the libusb-1.0 library and set its reporting level to
    // WARNING unless we are compiling for production.
    try {
      this->error = (libusb_error) libusb_init(&this->usb_context);
#ifndef NDEBUG
      libusb_set_debug(this->usb_context, LIBUSB_LOG_LEVEL_WARNING);
#endif
    } catch (LI_exception* error) {
      this->usb_context = NULL;
      throw error;
    }
    
    // Initialize the libuvc library
    try {
      this->error = uvc_init(&this->uvc_context, this->usb_context);
    } catch (LI_exception* error) {
      this->uvc_context = NULL;
      throw error;
    }
    
    // Inquire the hotplog capability from the underlying libusb library
    // (nonzero if the running library has the capability, 0 otherwise)
    if (0 == libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      this->error = LI_UNSUPPORTED_PLATFORM;
    }
    
    // Register the hotplug callback, note that we are passing NULL in
    // place of the libusb context (which is different from the libuvc
    // context) and NULL in place of the handler (last argument), which
    // is simply an integer that allows us to refer to this particular
    // callback to deregister it later.
    this->error = (libusb_error) libusb_hotplug_register_callback(
      this->usb_context, 
      LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT_OR_ARRIVED, 
      LIBUSB_HOTPLUG_NO_FLAGS, 
      LIBUSB_HOTPLUG_MATCH_ANY, 
      LIBUSB_HOTPLUG_MATCH_ANY,
      LIBUSB_HOTPLUG_MATCH_ANY, 
      LI_stereocamera::hotplug_callback,
      (void*) this /* user_data */, NULL /* handle */);
    
    this->update_connection();
  }
  
  ~LI_stereocamera() {
    
    // Stop streaming and deallocate devices/handles.
    this->on_disconnect();
    
    // Deinitialize the libuvc library.
    if (NULL != this->uvc_context) {
      uvc_exit(this->uvc_context);
    }
    
    // Deinitialize the libusb-1.0 library
    if (NULL != this->usb_context) {
      // Deregister the hotplug callback. Note that we are passing a 
      // callback handle of 0 (second argument) since we did not use a 
      // "handle" to register our callback (last argument of libusb_
      // hotplug_register_callback).
      libusb_hotplug_deregister_callback(this->usb_context, 0);
      libusb_exit(this->usb_context);
    }
  }
  
  void main_loop() {
    libusb_handle_events_completed(this->usb_context, NULL);
  }
};

#endif
