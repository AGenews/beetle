/*  ####################################################
    This is a minimal example to start capturing frames
    from a PointGrey BlackFly Camera using Flycapture2
    on Linux. It is based on Kevin Hughes script:
    https://gist.github.com/kevinhughes27/5543668
    But it lacks all eroor handling to make the script
    slick and understandable.

    AndreasGenewsky (2016)
    ####################################################
*/


/// KNOWN BUGS
/// (1) The Temporal performance is still weak, which was tobe expected as nothing was time optimized.
///     We start with cleaning up
/// (2) as it turns out ... resizing causes a dramatic decrease in performance

/// (3) Moreover we have no idea when a frame has been captured, thats why we need Software Triggering
///     This will all cause the framerate to drop, but idealy we have something like 40Hz aka 25 ms bins

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
#include "stdio.h"
#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
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
cv::Mat image, background,background_gray, mod, gray, color, thresh, thresh2, fgmask, bgmask, bgmask2, colored_gray, combined, combined2, blobs, threshold_frame, gray_ori, gray_contrast, contrast_thresh, hsv, test;
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

//std::vector<vector<cv::Point> > contours;
std::vector<cv::Vec4i> heirarchy;
std::vector<cv::Point2i> center;
std::vector<int> radius;
int largest_beetle_area=0;
int largest_beetle_index=0;
int largest_object_area=0;
int largest_object_index=0;

int samplingintervall = 25000;  /// in microseconds

cv::RNG rng(12345);


cv::Point2i point1, point2, placeholder;
bool callback = false;
void PrintError(const Error error) { error.PrintErrorTrace(); }
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

bool CheckSoftwareTriggerPresence( Camera* pCam )
{
	const unsigned int k_triggerInq = 0x530;

	Error error;
	unsigned int regVal = 0;

	error = pCam->ReadRegister( k_triggerInq, &regVal );

	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return false;
	}

	if( ( regVal & 0x10000 ) != 0x10000 )
	{
		return false;
	}

	return true;
}

bool PollForTriggerReady( Camera* pCam )
{
	const unsigned int k_softwareTrigger = 0x62C;
	Error error;
	unsigned int regVal = 0;

	do
	{
		error = pCam->ReadRegister( k_softwareTrigger, &regVal );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return false;
		}

	} while ( (regVal >> 31) != 0 );

	return true;
}

bool FireSoftwareTrigger( Camera* pCam )
{
	const unsigned int k_softwareTrigger = 0x62C;
	const unsigned int k_fireVal = 0x80000000;
	Error error;

	error = pCam->WriteRegister( k_softwareTrigger, k_fireVal );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return false;
	}

	return true;
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


int main(int /*argc*/, char** /*argv*/)
{
Camera cam;
Error error;
BusManager busMgr;
unsigned int numCameras;
error = busMgr.GetNumOfCameras(&numCameras);
if (error != PGRERROR_OK)
{
    PrintError( error );
	return -1;
}
cout << "Number of cameras detected: " << numCameras << endl;
if ( numCameras < 1 )
{
    cout << "Insufficient number of cameras... exiting" << endl;
	return -1;
}

PGRGuid guid;
error = busMgr.GetCameraFromIndex(0, &guid);
if (error != PGRERROR_OK)
{
    PrintError( error );
	return -1;
}

/// Connect to a camera
error = cam.Connect(&guid);
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

/// Power on the camera
const unsigned int k_cameraPower = 0x610;
const unsigned int k_powerVal = 0x80000000;
error  = cam.WriteRegister( k_cameraPower, k_powerVal );
if (error != PGRERROR_OK)
{
    PrintError( error );
	return -1;
}

const unsigned int millisecondsToSleep = 100;
unsigned int regVal = 0;
unsigned int retries = 10;

/// Wait for camera to complete power-up
do
{
#if defined(_WIN32) || defined(_WIN64)
    Sleep(millisecondsToSleep);
#elif defined(LINUX)
	struct timespec nsDelay;
	nsDelay.tv_sec = 0;
	nsDelay.tv_nsec = (long)millisecondsToSleep * 1000000L;
	nanosleep(&nsDelay, NULL);
#endif
	error = cam.ReadRegister(k_cameraPower, &regVal);
	if (error == PGRERROR_TIMEOUT)
	{
	// ignore timeout errors, camera may not be responding to
	// register reads during power-up
	}
	else if (error != PGRERROR_OK)
	{
			PrintError( error );
			return -1;
	}

	retries--;
} while ((regVal & k_powerVal) == 0 && retries > 0);

/// Check for timeout errors after retrying
if (error == PGRERROR_TIMEOUT)
{
    PrintError( error );
    return -1;
}


/// Get the camera information
CameraInfo camInfo;
error = cam.GetCameraInfo(&camInfo);
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

PrintCameraInfo(&camInfo);

#ifndef SOFTWARE_TRIGGER_CAMERA
/// Check for external trigger support
TriggerModeInfo triggerModeInfo;
error = cam.GetTriggerModeInfo( &triggerModeInfo );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

if ( triggerModeInfo.present != true )
{
	cout << "Camera does not support external trigger! Exiting..." << endl;
	return -1;
}
#endif

///Setting Strobe Output to Strobe at every Frame
StrobeControl mStrobe;
mStrobe.source = 1;
mStrobe.onOff = true;
mStrobe.polarity = 1;
mStrobe.delay = 0;
mStrobe.duration = 1.0f;
cam.SetStrobe(&mStrobe);

/// Get current trigger settings
TriggerMode triggerMode;
error = cam.GetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

/// Set camera to trigger mode 0
triggerMode.onOff = true;
triggerMode.mode = 0;
triggerMode.parameter = 0;

#ifdef SOFTWARE_TRIGGER_CAMERA
	// A source of 7 means software trigger
	triggerMode.source = 7;
#else
	// Triggering the camera externally using source 0.
	triggerMode.source = 0;
#endif

	error = cam.SetTriggerMode( &triggerMode );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

/// Poll to ensure camera is ready
bool retVal = PollForTriggerReady( &cam );
if( !retVal )
{
	cout << endl;
	cout << "Error polling for trigger ready!" << endl;
	return -1;
}




/// Query for available Format 7 modes
Format7Info fmt7Info;
bool supported;
fmt7Info.mode = k_fmt7Mode;
error = cam.GetFormat7Info( &fmt7Info, &supported );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

PrintFormat7Capabilities( fmt7Info );
if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
{
/// Pixel format not supported!
	cout << "Pixel format is not supported" << endl;
	return -1;
}

Format7ImageSettings fmt7ImageSettings;
fmt7ImageSettings.mode = k_fmt7Mode;
fmt7ImageSettings.offsetX = 0;
fmt7ImageSettings.offsetY = 0;
fmt7ImageSettings.width = fmt7Info.maxWidth;
fmt7ImageSettings.height = fmt7Info.maxHeight;
fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

bool valid;
Format7PacketInfo fmt7PacketInfo;

/// Validate the settings to make sure that they are valid
error = cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}
if ( !valid )
{
/// Settings are not valid
	cout << "Format7 settings are not valid" << endl;
	return -1;
}

/// Set the settings to the camera
error = cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

/// Get the camera configuration
FC2Config config;
error = cam.GetConfiguration( &config );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

/// Set the grab timeout to 5 seconds
config.grabTimeout = 50;

/// Set the camera configuration
error = cam.SetConfiguration( &config );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

/// Camera is ready, start capturing images
error = cam.StartCapture();
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

#ifdef SOFTWARE_TRIGGER_CAMERA
if (!CheckSoftwareTriggerPresence( &cam ))
{
	cout << "SOFT_ASYNC_TRIGGER not implemented on this camera!  Stopping application" << endl ;
	return -1;
}
#else
	cout << "Trigger the camera by sending a trigger pulse to GPIO" << triggerMode.source << endl;

#endif


/*
/// Start capturing images
error = cam.StartCapture();
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}
*/

/// FRAME RATE
Property frmRate;
frmRate.type = FRAME_RATE;
error = cam.GetProperty( &frmRate );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}
frmRate.autoManualMode = false;
frmRate.absControl = true;
frmRate.absValue = 60.0; // you may set to your desired fixed frame rate
error = cam.SetProperty(&frmRate);
if (error != PGRERROR_OK)
{
    PrintError(error);
    return -1;
}
//cout << "Frame rate is " << fixed << setprecision(2) << frmRate.absValue << " fps" << endl;

/// AUTO_EXPOSURE
Property autoexp;
autoexp.type = AUTO_EXPOSURE;
error = cam.GetProperty( &autoexp );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}
autoexp.autoManualMode = false;
autoexp.absControl = true;
autoexp.absValue = 1.0;
error = cam.SetProperty(&autoexp);
if (error != PGRERROR_OK)
{
    PrintError(error);
    return -1;
}

//cout << "EXPOSURE is set to " << fixed << setprecision(2) << autoexp.absValue << " EV"<< endl;

/// SHUTTER
Property shtr;
shtr.type = SHUTTER;
error = cam.GetProperty( &shtr );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
shtr.autoManualMode = false;
shtr.absControl = true;
shtr.absValue = 15.0;
error = cam.SetProperty(&shtr);
if (error != PGRERROR_OK)
{
    PrintError(error);
    return -1;
}

//cout << "SHUTTER is set to " << fixed << setprecision(2) << shtr.absValue << " ms"<< endl;

/// GAIN
Property gn;
gn.type = GAIN;
error = cam.GetProperty( &gn );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}
gn.autoManualMode = false;
gn.absControl = true;
gn.absValue = 5.0;
error = cam.SetProperty(&gn);
if (error != PGRERROR_OK)
{
    PrintError(error);
    return -1;
}

//cout << "GAIN is set to " << fixed << setprecision(2) << gn.absValue << " dB"<< endl;
/*
StrobeControl mStrobe;
mStrobe.source = 1;
mStrobe.onOff = false;
mStrobe.polarity = 1;
mStrobe.delay = 0;
mStrobe.duration = 1.0f;
cam.SetStrobe(&mStrobe);
*/


/// Initialize values
int lthreshval = 60;
int min_contour_m = 75;
int max_contour_m = 1000;
int exp_val = 1;
int gn_val = 5;
int shtr_val = 15;

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
int sat_min_b = 45;
int vol_min_b = 45;
int hue_max_b = 25;
int sat_max_b = 360;
int vol_max_b = 360;
int beetle_contrast = 5;
int beetle_size = 5;
int min_contour_b = 125;
int max_contour_b = 250;

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
cvCreateTrackbar( "Beetle Size", NULL, &beetle_size, 100, NULL);


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


/// ### BENCHMARK STUFF
long frameCounter = 0;
std::time_t timeBegin = std::time(0);
int tick = 0;
/// ###



char key = '0';


struct timeval start_t, end_t;
gettimeofday(&start_t, NULL);

while(key != 'q')
    {
    if(key == 's'){
    recording_flag = 1;
    mStrobe.onOff = true;
    cam.SetStrobe(&mStrobe);
    }
    if(key == 'e'){
    recording_flag = 0;
    mStrobe.onOff = false;
    cam.SetStrobe(&mStrobe);
    }

    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime(buffer,80,"%c",timeinfo);

    if(key == 'd'){
        select_flag = 0;
    }

    if(key == 'g'){
        grid_flag = !grid_flag;
    }

    if(auto_exposure_flag == 1){
        autoexp.autoManualMode = true;
        autoexp.absControl = false;
        //autoexp.absValue = 1.0;
        cam.SetProperty(&autoexp);
     }
    if(auto_exposure_flag == 0){
        autoexp.autoManualMode = false;
        autoexp.absControl = true;
        autoexp.absValue = exp_val;
        cam.SetProperty(&autoexp);
    }
    if(auto_gn_flag == 1){
        gn.autoManualMode = true;
        gn.absControl = false;
        //gn.absValue = 5.0;
        cam.SetProperty(&gn);
    }
    if(auto_gn_flag == 0){
        gn.autoManualMode = false;
        gn.absControl = true;
        gn.absValue = gn_val;
        cam.SetProperty(&gn);
    }
    if(auto_shtr_flag == 1){
        shtr.autoManualMode = true;
        shtr.absControl = false;
        cam.SetProperty(&shtr);
    }
    if(auto_shtr_flag == 0){
        shtr.autoManualMode = false;
        shtr.absControl = true;
        shtr.absValue = shtr_val;
        cam.SetProperty(&shtr);
    }


    Image rawImage;
//#ifdef SOFTWARE_TRIGGER_CAMERA
    /// Check that the trigger is ready
	PollForTriggerReady( &cam);
//	cout << "Press the Enter key to initiate a software trigger" << endl;
//	cin.ignore();

/// Time Sensitive Loop
/// Here we basically put our frame rate limiter
    gettimeofday(&end_t, NULL);
    if( (end_t.tv_usec-start_t.tv_usec)<samplingintervall){
    continue;
    }
    else{
    gettimeofday(&start_t, NULL);
    }


	/// Fire software trigger
	bool retVal = FireSoftwareTrigger( &cam );
	if ( !retVal )
	{
        cout << endl;
		cout << "Error firing software trigger" << endl;

		return -1;
    }
//#endif
    //continue;

	/// Grab image
	error = cam.RetrieveBuffer( &rawImage );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		break;
	}

    /// convert to rgb
    Image rgbImage;
    rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
    /// convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    color = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
//  if(k_fmt7Mode == MODE_3){
//    cv::resize(color, color, cv::Size(color.cols*2, color.rows*2), cv::INTER_NEAREST);
//    }
    cv::cvtColor(color.clone(), gray, CV_BGR2GRAY);
    image = color;
    ///BACKGROUND
    if(key == 'b'){
        background_flag = 1;
        get_background = 1;
    }
    if(key == 'c'){
        background_flag = 0;
        get_background = 0;
    }
    if(get_background == 1){
    get_background = 0;
    image.copyTo(background);
    image.copyTo(background_gray);
    cv::cvtColor(background_gray,background_gray, CV_BGR2GRAY);
    }

    /// ########################
    /// ## FINDING THE BEETLE ##
    /// ########################
    cv::cvtColor(image.clone(),hsv, CV_BGR2HSV);
    cv::Scalar   min(hue_min_b, sat_min_b, vol_min_b);
    cv::Scalar   max(hue_max_b, sat_max_b, vol_max_b);
    ///Find our lovely HSV objects
    cv::inRange( hsv, min, max, threshold_frame);
    /// Do some cleaning
    /// Create a structuring erode element
    cv::Mat erode_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );
    cv::erode(threshold_frame,threshold_frame,erode_element);
    /// Create a structuring dilation element
    cv::Mat dilate_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size) );
    cv::dilate(threshold_frame,threshold_frame,dilate_element);
    cv::dilate(threshold_frame,threshold_frame,dilate_element);
    cv::dilate(threshold_frame,threshold_frame,dilate_element);
    cv::dilate(threshold_frame,threshold_frame,dilate_element);
    cv::threshold(threshold_frame,threshold_frame, beetle_contrast,255,CV_THRESH_BINARY);

    /// Now we have a mask
    gray.clone().copyTo(threshold_frame,threshold_frame);
    cv::threshold(threshold_frame,threshold_frame, beetle_contrast,255,CV_THRESH_BINARY);
    /// Find contours
    cv::findContours( threshold_frame, beetle_contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    ///Get rid of contours which are either too small or too huge
    for (vector<vector<cv::Point> >::iterator it = beetle_contours.begin(); it!=beetle_contours.end(); )
    {
        if ( (it->size()<min_contour_b) || (it->size()>max_contour_b) )
            it=beetle_contours.erase(it);
        else
            ++it;
    }
    /// Look for the largest contour and rember that index
    for( int i = 0; i<beetle_contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( beetle_contours[i],false);  //  Find the area of contour
        if(a>largest_beetle_area){
        largest_beetle_area=a;
        largest_beetle_index=i;                //Store the index of largest contour
        }
    }

    if(beetle_contours.size()>0){
        beetle_flag = 1;
    }
    else{
        beetle_flag = 0;
    }


    /// Get the moments
    vector<cv::Moments> mub(beetle_contours.size() );
    for( int i = 0; i < beetle_contours.size(); i++ )
    {
        mub[i] = moments( beetle_contours[i], false );
    }


    ///  Get the mass centers:
    vector<cv::Point2f> mcb( beetle_contours.size() );
    for( int i = 0; i < beetle_contours.size(); i++ )
    {
        mcb[i] = cv::Point2f( mub[i].m10/mub[i].m00 , mub[i].m01/mub[i].m00 );
    }
    cv::Point2f beetle_coordinate, absolute_beetle;
    if(beetle_flag == 1){
    absolute_beetle = mcb[largest_beetle_index];
    beetle_coordinate = absolute_beetle;
    }

    if( (select_flag == 1) && (beetle_flag == 1) ){
        beetle_coordinate = cv::Point2f( (mcb[largest_beetle_index].x-float(point1.x) ), (mcb[largest_beetle_index].y-float(point1.y) ) );
    }


    /// ##########################
    /// # BACKGROUND SUBTRACTION #
    /// ##########################

    if(background_flag == 1){
        ///Generate background subtracted image in color space to increase contrast
        cv::absdiff(background,image,mod);
        /// ######################
        /// ## Contrasting Code ##
        /// ######################

        /// Here is the NEW contrasting code
        image.copyTo(gray_ori);
        cv::cvtColor(gray_ori,gray_ori, CV_BGR2GRAY);
        cv::absdiff(background_gray,gray_ori,gray_contrast);
        cv::threshold(gray_contrast, contrast_thresh, min_contrast,255,cv::THRESH_BINARY);
        /// now contrast_thresh is as mask which simply blacks out areas with bad contrast
        /// This mask we have to apply to the movement sensitive mask
        if(beetle_flag == 1)
        {
        cv::Scalar color(0,0,0);
        cv::drawContours( contrast_thresh, beetle_contours, largest_beetle_index, color, -1, 8, hierarchy, 0, cv::Point() );
        }

        ///Convert to grayscale
        /// This generates a very motion sensitive image which we use to find objects
        cv::cvtColor(mod.clone(), gray, CV_BGR2GRAY);
        if(select_flag == 1){
            cv::Mat ROI_inv = cv::Mat::zeros(gray.size(), gray.type());
            cv::Mat new_gray = cv::Mat::zeros(gray.size(), gray.type());
            cv::rectangle(ROI_inv, point1, point2, CV_RGB(255, 255, 255), -1, 0, 0);
            gray.copyTo(new_gray,ROI_inv);
            new_gray.copyTo(gray);
        }
        ///Thresholding to make every movement clearly visible
        cv::threshold(gray.clone(), thresh, 5,255,cv::THRESH_BINARY);
        /// >>> Here we fuse the contrast & and the motion sensitive masks.
        cv::Mat new_mod = cv::Mat::zeros(thresh.size(), thresh.type());
        thresh.copyTo(new_mod,contrast_thresh);

        mod = new_mod;
        ///Cleaning up the detected changed pixels with erosion and dilation
        /// Create a structuring erode element
        cv::Mat erode_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );
        cv::erode(mod,mod,erode_element);
        cv::erode(mod,mod,erode_element);
        /// Create a structuring dilation element
        cv::Mat dilate_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size) );
        cv::dilate(mod,mod,dilate_element);
        cv::dilate(mod,mod,dilate_element);
        cv::dilate(mod,mod,dilate_element);
        cv::dilate(mod,mod,dilate_element);
        /// Now we look in the original image only at those moving pixels and additionally to a second thresholding
        /// operation which should at the end allow to detect either animals which are darker or whiter than the background
        /// we use a slider to adjust the parameters
        cv::cvtColor(color, gray, CV_BGR2GRAY);
        cv::Mat dstImage = cv::Mat::zeros(gray.size(), gray.type());
        dstImage = cv::Scalar::all(animal_color);
        gray.copyTo(dstImage,mod);
        /// We generate two masks one for the background and one for the foreground
        cv::threshold(dstImage, bgmask,lthreshval,255,cv::THRESH_BINARY);
        cv::threshold(dstImage, fgmask,lthreshval,255,cv::THRESH_BINARY_INV);
        /// Now I would like overlay the detected pixels will become blueishly colorized with the original video
        /// like anymaze is doing
        /// fgmask is also the mask we use to detect the black blobs

        /// but we also want to use our ROI to e.g. limit reflections
        /// therefore we simly blackout everything around our ROI
        fgmask.copyTo(blobs);

        /// We need a grayscale original
        cv::cvtColor(color, gray, CV_BGR2GRAY);
        cv::cvtColor(gray, gray, CV_GRAY2BGR);
        cv::Mat bgImage = cv::Mat::zeros(gray.size(), gray.type());
        /// And we apply the bgmask
        gray.copyTo(bgImage,bgmask);

        /// Now we colorize the grayscale image and apply the foreground mask
        cv::applyColorMap(gray, colored_gray, cv::COLORMAP_WINTER);
        cv::Mat fgImage = cv::Mat::zeros(colored_gray.size(), colored_gray.type());
        colored_gray.copyTo(fgImage,fgmask);

        /// Finally we need to combine both pictures
        cv::Mat combined = cv::Mat::zeros(image.size(), image.type());
        combined = fgImage+bgImage;
        combined.copyTo(combined2);
        /// DONE nice JOB!

        ///Let's work on this nicely false colored images
        cv::cvtColor(combined,combined, CV_BGR2HSV);
        cv::Scalar   min(hue_min_m, sat_min_m, vol_min_m);
        cv::Scalar   max(hue_max_m, sat_max_m, vol_max_m);

        ///Find our lovely HSV objects
        cv::inRange( combined, min, max, threshold_frame);
        //erode_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );
        cv::erode(threshold_frame,threshold_frame,erode_element);
        //dilate_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size) );
        cv::dilate(threshold_frame,threshold_frame,dilate_element);

        /// I suspect morphologyEx to be quite time consuming.
        //cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        /// Doe some cleaning
        //cv::morphologyEx(threshold_frame, threshold_frame, cv::MORPH_OPEN, str_el);
        //cv::morphologyEx(threshold_frame, threshold_frame, cv::MORPH_CLOSE, str_el);
        /// Find contours
        cv::findContours( threshold_frame, object_contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
        ///Get rid of contours which are either too small or too huge

        for (vector<vector<cv::Point> >::iterator it = object_contours.begin(); it!=object_contours.end(); )
        {
            if ( (it->size()<min_contour_m) || (it->size()>max_contour_m) )
                it=object_contours.erase(it);
            else
                ++it;
        }

        /// Look for the largest contour and rember that index
        for( int i = 0; i<object_contours.size(); i++ ) // iterate through each contour.
        {
            double a=contourArea( object_contours[i],false);  //  Find the area of contour
            if(a>largest_object_area){
                largest_object_area=a;
                largest_object_index=i;                //Store the index of largest contour
            }

        }

        if(object_contours.size()>0){
            object_flag = 1;
        }
        else{
            object_flag = 0;
        }

        /// Get the moments
        vector<cv::Moments> mum(object_contours.size() );
        for( int i = 0; i < object_contours.size(); i++ )
        {
            mum[i] = moments( object_contours[i], false );
        }
        ///  Get the mass centers:
        vector<cv::Point2f> mcm( object_contours.size() );
        for( int i = 0; i < object_contours.size(); i++ )
        {
            mcm[i] = cv::Point2f( mum[i].m10/mum[i].m00 , mum[i].m01/mum[i].m00 );
        }
        /// Display the number of detected objects
        std::string contours_number = "Objects detected "+patch::to_string(object_contours.size());
        cv::putText(combined2,contours_number, cv::Point(25, 50), 1, 1, CV_RGB(7, 166, 245), 1, 8, false);
        cv::Scalar color = cv::Scalar(33,57,217);

        /// Lets draw some stuff and calculate the object coordinate also with respect to our rectangle

        cv::Point2f object_coordinate, absolute_object;
        for( int i = 0; i< object_contours.size(); i++ )
        {
            absolute_object = mcm[largest_object_index];
            object_coordinate = absolute_object;
            if( (select_flag == 1) && (object_flag == 1) ){
                object_coordinate = cv::Point2f( (mcm[largest_object_index].x-float(point1.x) ), (mcm[largest_object_index].y-float(point1.y) ) );
            }

            cv::drawContours( combined2, object_contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
            cv::circle(combined2, mcm[largest_object_index], 2, color, -1, 8, 0 );
            std::string object_coordinate_str = "["+patch::to_string(round(object_coordinate.x))+","+patch::to_string(round(object_coordinate.y))+"]";;
            cv::putText(combined2,object_coordinate_str,mcm[largest_object_index],1, 2, CV_RGB(7, 166, 245), 1, 8, false);
        }
        image = combined2;
    }

    ///Decorate
    if (select_flag == 1){
        cv::rectangle(image, point1, point2, CV_RGB(255, 0, 0), 2, 8, 0);
    }
    if(beetle_flag == 1){
        cv::Scalar color = cv::Scalar(68,204,18);
        cv::circle(image, absolute_beetle, 2, color, -1, 8, 0 );
        std::string beetle_coordinate_str = "["+patch::to_string(round(beetle_coordinate.x))+","+patch::to_string(round(beetle_coordinate.y))+"]";
        cv::putText(image,beetle_coordinate_str,absolute_beetle,1, 2, CV_RGB(18, 204, 68), 1, 8, false);
        cv::drawContours( image, beetle_contours, largest_beetle_index, color, -1, 8, hierarchy, 0, cv::Point() );
    }



    if(grid_flag == 1){
        int stepSize = 50;
        int width = image.size().width;
        int height = image.size().height;
        for (int i = 0; i<height; i += stepSize){
            cv::line(image, cv::Point(0, i), cv::Point(width, i), cv::Scalar(255, 204, 0),2,8,0);
        }
        for (int i = 0; i<width; i += stepSize){
        cv::line(image, cv::Point(i, 0), cv::Point(i, height), cv::Scalar(255, 204, 0),2,8,0);
        }
    }

    cv::putText(image, buffer, cv::Point(25, 25), 1, 2, CV_RGB(7, 166, 245), 1, 8, false);


    ///DISPLAY
    cv::imshow(src_window, image);
    key = cv::waitKey(1);
    largest_beetle_area=0;
    largest_object_area = 0;
    largest_beetle_index = 0;
    largest_object_index = 0;

    /// ### BENCHMARK STUFF
    frameCounter++;
    std::time_t timeNow = std::time(0) - timeBegin;
    if (timeNow - tick >= 1)
    {
        tick++;
        cout << "Frames per second: " << frameCounter << endl;
        frameCounter = 0;
    }
    /// ###

    usleep(25*1000);

} /// END of CAPTURING LOOP


/// Turn trigger mode off.
triggerMode.onOff = false;
error = cam.SetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}
cout << endl;
cout << "Finished grabbing images" << endl;

/// Stop capturing images
error = cam.StopCapture();
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

mStrobe.onOff = false;
cam.SetStrobe(&mStrobe);


/// Disconnect the camera
error = cam.Disconnect();
if (error != PGRERROR_OK)
{
	PrintError( error );
	return -1;
}

return 0;
}



/// CODE SNIPPET DUMPSTER

/*
    cv::Mat threshold_frame = color.clone();
    std::vector<cv::Mat> channels (3);
    cv::split(threshold_frame,channels);
    threshold_frame = channels[2];
    cv::threshold(threshold_frame,threshold_frame,190,255,CV_THRESH_BINARY);
*/

/*
        /// ############################################
        /// ## This is the ROI specific Contrast code ##
        /// ############################################


        /// Here we need to stick in the code to check for maximal contrast

        /// 1) make a vector to store the contrasts as long as our contours
        std::vector<int> contrasts(contours.size());

        /// 2) make a frame for every
        cv::Mat labels = cv::Mat::zeros(threshold_frame.size(), CV_8UC1);
        cv::Mat diff_frame = cv::Mat::zeros(gray.size(), gray.type());
        cv::cvtColor(gray,gray,CV_BGR2GRAY);
        for( int i = 0; i<contours.size();i++)
        {
            cv::Rect roi = cv::boundingRect(contours[i]);
            cv::absdiff(gray(roi),background_gray(roi),diff_frame);
            cv::Scalar diff = cv::mean(diff_frame);
            contrasts[i] = diff[0];

        }

        int i = 0;
        while ( i < contours.size() )
        {
            if ( contrasts[i]<min_contrast ) {
                contours.erase(contours.begin() + i);
            continue;
            }
        ++i;
        }
*/

/*
        if(select_flag == 2){
            cv::Mat ROI_inv = cv::Mat::zeros(fgmask.size(), fgmask.type());
            cv::Mat new_blobs = cv::Mat::zeros(fgmask.size(), fgmask.type());
            cv::rectangle(ROI_inv, point1, point2, CV_RGB(255, 255, 255), -1, 0, 0);
            blobs.copyTo(new_blobs,ROI_inv);
            new_blobs.copyTo(blobs);
        }
*/
