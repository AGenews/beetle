/*  ####################################################
    Beetle Header File.
    Here we have all the variable & function definitions.
    AndreasGenewsky (2016)
    ####################################################
*/
#include "flycapture/FlyCapture2.h"
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <ctime>
#include <stdio.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
//Libserial: sudo apt-get install libserial-dev
#include <SerialStream.h>
#define PORT "/dev/ttyUSB0" //This is system-specific

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
#include <iostream>
#include <iomanip>

using namespace FlyCapture2;
using namespace std;
using namespace LibSerial;



/// Defin a lot of variables --- still not sorted SORRY
cv::Mat image, background, background_gray, mod, gray, color, thresh, fgmask, bgmask, colored_gray, combined, combined2, blobs, threshold_frame, gray_ori, gray_contrast, contrast_thresh, hsv;
const char* src_window = "Bettle";
int drag = 0, select_flag = 0, background_flag = 0, get_background = 0, recording_flag = 0, blobs_flag = 0, minTargetRadius = 5, beetle_flag = 0, object_flag = 0;
int auto_exposure_flag = 0, auto_gn_flag = 0, auto_shtr_flag = 0;
int erosion_size = 2, dilation_size = 2;
int animal_color = 255;
const Mode k_fmt7Mode = MODE_2;
const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
bool enableRadiusCulling = false;
bool grid_flag = false;

std::vector<cv::KeyPoint> keypoints;
std::vector<vector<cv::Point> > object_contours;
std::vector<vector<cv::Point> > beetle_contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> approxShape;
std::vector<cv::Vec4i> heirarchy;
std::vector<cv::Point2i> center;
std::vector<int> radius;
int largest_beetle_area=0;
int largest_beetle_index=0;
int largest_object_area=0;
int largest_object_index=0;
cv::Point2i point1, point2, placeholder;
cv::Point2f object_coordinate, absolute_object;
cv::Point2f beetle_coordinate, absolute_beetle;

/// Initialize values
int lthreshval = 60;
int min_contour_m = 5;
int max_contour_m = 250;
int exp_val = 1;
int gn_val = 5;
int shtr_val = 5;

/// MOUSE
int hue_min_m = 100;
int sat_min_m = 0;
int vol_min_m = 0;
int hue_max_m = 165;
int sat_max_m = 360;
int vol_max_m = 360;
int min_contrast = 30;

/// BEETLE
int hue_min_b = 0;
int sat_min_b = 6;
int vol_min_b = 6;
int hue_max_b = 40;
int sat_max_b = 360;
int vol_max_b = 360;
int beetle_contrast = 25;
int min_contour_b = 0;
int max_contour_b = 250;

bool callback = false;

class Camera cam;
class Error error;
class BusManager busMgr;
class PGRGuid guid;
class StrobeControl mStrobe;
class CameraInfo camInfo;
class TriggerModeInfo triggerModeInfo;
class TriggerMode triggerMode;
class Format7Info fmt7Info;
class Format7ImageSettings fmt7ImageSettings;
class Format7PacketInfo fmt7PacketInfo;
class Property frmRate;
class Property autoexp;
class Property shtr;
class Property gn;

struct timespec start_t, end_t;


/// DEFINE some Fuctions

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
        point1 = cv::Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        cv::Mat img1 = image.clone();
        point2 = cv::Point(x, y);
        cv::rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 2, 8, 0);
        cv::imshow(src_window, img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag && !select_flag)
    {
        cv::Mat img2 = image.clone();
        point2 = cv::Point(x, y);
        drag = 0;
        select_flag = 1;
        cv::imshow(src_window, img2);
        callback = true;
        /// Here we need to make the comparison
        if( (point1.x>point2.x) || (point1.y>point2.y) ){
        placeholder = point1;
        point1 = point2;
        point2 = placeholder;
        }

    }
}

void callbackButton(int, void*){
    cout << "click" << endl;
}
void cb_auto_exp_ON(int, void*){
    auto_exposure_flag = 1;
}
void cb_auto_exp_OFF(int, void*){
    auto_exposure_flag = 0;
}
void cb_auto_gn_ON(int, void*){
    auto_gn_flag = 1;
}
void cb_auto_gn_OFF(int, void*){
    auto_gn_flag = 0;
}
void cb_auto_shtr_ON(int, void*){
    auto_shtr_flag = 1;
}
void cb_auto_shtr_OFF(int, void*){
    auto_shtr_flag = 0;
    }


void decorate_app_window(){
    cv::namedWindow(src_window, CV_GUI_EXPANDED);
    cv::setMouseCallback(src_window,mouseHandler,0);
    //cv::createButton(nameb1,callbackButton,nameb1,CV_CHECKBOX,0);
    //cvCreateButton(nameb2,callbackButton,nameb2,CV_CHECKBOX,0);
    cvCreateTrackbar( "Exposure", NULL, &exp_val, 50, NULL);
    cvCreateButton("Auto Exposure OFF",cb_auto_exp_OFF,NULL,CV_RADIOBOX,0);
    cvCreateButton("Auto Exposure ON",cb_auto_exp_ON,NULL,CV_RADIOBOX,1);
    cvCreateTrackbar( "Gain", NULL, &gn_val, 25, NULL);
    cvCreateButton("Auto Gain OFF",cb_auto_gn_OFF,NULL,CV_RADIOBOX,0);
    cvCreateButton("Auto Gain ON",cb_auto_gn_ON,NULL,CV_RADIOBOX,1);
    cvCreateTrackbar( "Shutter", NULL, &shtr_val, 25, NULL);
    cvCreateButton("Auto Shutter OFF",cb_auto_shtr_OFF,NULL,CV_RADIOBOX,0);
    cvCreateButton("Auto Shutter ON",cb_auto_shtr_ON,NULL,CV_RADIOBOX,1);

    cvCreateTrackbar( "HUE_min_B", NULL, &hue_min_b, 360, NULL);
    cvCreateTrackbar( "SAT_min_B", NULL, &sat_min_b, 360, NULL);
    cvCreateTrackbar( "VOL_min_B", NULL, &vol_min_b, 360, NULL);
    cvCreateTrackbar( "HUE_max_B", NULL, &hue_max_b, 360, NULL);
    cvCreateTrackbar( "SAT_max_B", NULL, &sat_max_b, 360, NULL);
    cvCreateTrackbar( "VOL_max_B", NULL, &vol_max_b, 360, NULL);
    cvCreateTrackbar( "MinContour_B", NULL, &min_contour_b, 500, NULL);
    cvCreateTrackbar( "MaxContour_B", NULL, &max_contour_b, 1000, NULL);
    cvCreateTrackbar( "Beetle Contrast", NULL, &beetle_contrast, 255, NULL);

    /// Create Trackbars
    cv::createTrackbar( "Sensitivity", src_window, &lthreshval, 255, NULL);
    cv::createTrackbar( "MinContour_M", src_window, &min_contour_m, 5000, NULL);
    cv::createTrackbar( "MaxContour_M", src_window, &max_contour_m, 5000, NULL);
    cv::createTrackbar( "HUE_min_M", src_window, &hue_min_m, 360, NULL);
    cv::createTrackbar( "SAT_min_M", src_window, &sat_min_m, 360, NULL);
    cv::createTrackbar( "VOL_min_M", src_window, &vol_min_m, 360, NULL);
    cv::createTrackbar( "HUE_max_M", src_window, &hue_max_m, 360, NULL);
    cv::createTrackbar( "SAT_max_M", src_window, &sat_max_m, 360, NULL);
    cv::createTrackbar( "VOL_max_M", src_window, &vol_max_m, 360, NULL);
    cv::createTrackbar( "Contrast",src_window, &min_contrast, 255, NULL);
}

/// SOME CAMERA FUNCTIONS
// mostly stolen from the FlyCapture Examples
void PrintError(const Error error) { error.PrintErrorTrace(); }

void PrintBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion( &fc2Version );

	ostringstream version;
	version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
	cout << version.str() << endl;

	ostringstream timeStamp;
	timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
	cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
	cout << endl;
	cout << "*** CAMERA INFORMATION ***" << endl;
	cout << "Serial number -" << pCamInfo->serialNumber << endl;
	cout << "Camera model - " << pCamInfo->modelName << endl;
	cout << "Camera vendor - " << pCamInfo->vendorName << endl;
	cout << "Sensor - " << pCamInfo->sensorInfo << endl;
	cout << "Resolution - " << pCamInfo->sensorResolution << endl;
	cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
	cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
}

void PrintFormat7Capabilities( Format7Info fmt7Info )
{
	cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
	cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
	cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
	cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
}

void camera_setup(){
    unsigned int numCameras;
    const unsigned int k_cameraPower = 0x610;
    const unsigned int k_powerVal = 0x80000000;

    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK){PrintError( error );}
    cout << "Number of cameras detected: " << numCameras << endl;
    if ( numCameras < 1 ){cout << "Insufficient number of cameras... exiting" << endl;}


    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK){PrintError( error );}

    /// Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK){PrintError( error );}

    /// Power on the camera

    error  = cam.WriteRegister( k_cameraPower, k_powerVal );
    if (error != PGRERROR_OK){PrintError( error );}

    ///Setting Strobe Output to Strobe at every Frame
    mStrobe.source = 1;
    mStrobe.onOff = true;
    mStrobe.polarity = 1;
    mStrobe.delay = 0;
    mStrobe.duration = 5.0f;
    cam.SetStrobe(&mStrobe);

    /// Query for available Format 7 modes
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = cam.GetFormat7Info( &fmt7Info, &supported );
    if (error != PGRERROR_OK){PrintError( error );}

    PrintFormat7Capabilities( fmt7Info );
    /// Pixel format not supported!
    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 ){cout << "Pixel format is not supported" << endl;}

    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

    bool valid;
    /// Validate the settings to make sure that they are valid
    error = cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo );
    if (error != PGRERROR_OK){PrintError( error );}
     /// Settings are not valid
    if ( !valid ){cout << "Format7 settings are not valid" << endl;}

    /// Set the settings to the camera
    error = cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket );
    if (error != PGRERROR_OK){PrintError( error );}

    /// Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration( &config );
    if (error != PGRERROR_OK){PrintError( error );}
    /// Set the grab timeout to 1 seconds
    config.grabTimeout = 1000;

    /// Set the camera configuration
    error = cam.SetConfiguration( &config );
    if (error != PGRERROR_OK){PrintError( error );}

    /// Camera is ready, start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK){PrintError( error );}

    /// FRAME RATE
    frmRate.type = FRAME_RATE;
    error = cam.GetProperty( &frmRate );
    if (error != PGRERROR_OK){PrintError( error );}
    frmRate.onOff = true;
    frmRate.autoManualMode = false;
    frmRate.absControl = true;
    frmRate.absValue = 60.0; // you may set to your desired fixed frame rate
    error = cam.SetProperty(&frmRate);
    if (error != PGRERROR_OK){PrintError(error);}
    //cout << "Frame rate is " << fixed << setprecision(2) << frmRate.absValue << " fps" << endl;
    /// AUTO_EXPOSURE
    autoexp.type = AUTO_EXPOSURE;
    error = cam.GetProperty( &autoexp );
    if (error != PGRERROR_OK){PrintError( error );}
    autoexp.onOff = true;
    autoexp.autoManualMode = true;
    autoexp.absControl = false;
    autoexp.absValue = 1.0;
    error = cam.SetProperty(&autoexp);
    if (error != PGRERROR_OK){PrintError(error);}
    //cout << "EXPOSURE is set to " << fixed << setprecision(2) << autoexp.absValue << " EV"<< endl;

    /// SHUTTER
    shtr.type = SHUTTER;
    error = cam.GetProperty( &shtr );
    if (error != PGRERROR_OK){PrintError( error );}
    shtr.onOff = true;
    shtr.autoManualMode = true;
    shtr.absControl = true;
    shtr.absValue = 15.0;
    error = cam.SetProperty(&shtr);
    if (error != PGRERROR_OK){PrintError(error);}
    //cout << "SHUTTER is set to " << fixed << setprecision(2) << shtr.absValue << " ms"<< endl;

    /// GAIN
    gn.type = GAIN;
    error = cam.GetProperty( &gn );
    if (error != PGRERROR_OK){PrintError( error );}
    gn.onOff = true;
    gn.autoManualMode = true;
    gn.absControl = true;
    gn.absValue = 5.0;
    error = cam.SetProperty(&gn);
    if (error != PGRERROR_OK){PrintError(error);}
    //cout << "GAIN is set to " << fixed << setprecision(2) << gn.absValue << " dB"<< endl;

    /// Set camera to trigger mode 0
    triggerMode.onOff = false;
    triggerMode.mode = 0;
    triggerMode.parameter = 0;
    triggerMode.source = 0;

    ///Setting Strobe Output to Strobe at every Frame
    mStrobe.source = 1;
    mStrobe.onOff = true;
    mStrobe.polarity = 1;
    mStrobe.delay = 0;
    mStrobe.duration = 5.0f;
    cam.SetStrobe(&mStrobe);

    error = cam.SetTriggerMode( &triggerMode );
    if (error != PGRERROR_OK){PrintError( error );}
}




