/*  ####################################################
    This little app allows the user to detect two moving
    objects. An orange beetle as well as a dark/light mouse.
    AndreasGenewsky (2016)
    ####################################################
*/

/// KNOWN BUGS
/// waiKey, imshow, inRange, too much cv::Mat, limit Drawing and waitKey to e.g. every 10th frame

#include "beetle.h"

using namespace FlyCapture2;
using namespace std;

uint64_t prev_time_value, time_value;
uint64_t time_diff;

uint64_t interval = 10000;  /// in microseconds

uint64_t get_posix_clock_time ()
{
    struct timespec ts;

    if (clock_gettime (CLOCK_MONOTONIC, &ts) == 0)
        return (uint64_t) (ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
    else
        return 0;
}

int main(int /*argc*/, char** /*argv*/)
{
/// Setting up the Camera
camera_setup();
/// Generate Trackbars and Buttons
decorate_app_window();

/// ### BENCHMARK STUFF
long frameCounter = 0;
std::time_t timeBegin = std::time(0);
int tick = 0;
/// ###

char key = '0';

prev_time_value = get_posix_clock_time ();

while(key != 'q')
    {
    time_value = get_posix_clock_time ();
    time_diff = time_value - prev_time_value;
    if(time_diff<=interval){
    continue;
    }
    //cout << time_diff <<endl;
    prev_time_value = get_posix_clock_time ();
    time_diff = time_value - prev_time_value;


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

    if(key == 'd'){select_flag = 0;}

    if(key == 'g'){grid_flag = !grid_flag;}

    if(auto_exposure_flag == 1){
        autoexp.autoManualMode = true;
        autoexp.absControl = false;
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
/// Check that the trigger is ready
	PollForTriggerReady( &cam);

/// Time Sensitive Loop
/// Here we basically put our frame rate limiter


	/// Fire software trigger
	bool retVal = FireSoftwareTrigger( &cam );
	if ( !retVal ){cout << "Error firing software trigger" << endl;}

	/// Grab image
	error = cam.RetrieveBuffer( &rawImage );
	if (error != PGRERROR_OK){PrintError( error );break;}

    /// convert to rgb
    Image rgbImage;
    rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
    /// convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    color = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
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
	// MORPH_RECT is said to be faster!
    cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );
    cv::erode(threshold_frame,threshold_frame,erode_element);
    /// Create a structuring dilation element
    cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size) );
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
        if ( (it->size()<unsigned(min_contour_b)) || (it->size()>unsigned(max_contour_b)) )
            it=beetle_contours.erase(it);
        else
            ++it;
    }
    /// Look for the largest contour and rember that index
    for( unsigned int i = 0; i<beetle_contours.size(); i++ ) // iterate through each contour.
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
    for( unsigned int i = 0; i < beetle_contours.size(); i++ )
    {
        mub[i] = moments( beetle_contours[i], false );
    }

    ///  Get the mass centers:
    vector<cv::Point2f> mcb( beetle_contours.size() );
    for( unsigned int i = 0; i < beetle_contours.size(); i++ )
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
        //cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );
        cv::erode(mod,mod,erode_element);
        cv::erode(mod,mod,erode_element);
        /// Create a structuring dilation element
        //cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size) );
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
        cv::erode(threshold_frame,threshold_frame,erode_element);
        cv::dilate(threshold_frame,threshold_frame,dilate_element);
        /// Find contours
        cv::findContours( threshold_frame, object_contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
        ///Get rid of contours which are either too small or too huge

        for (vector<vector<cv::Point> >::iterator it = object_contours.begin(); it!=object_contours.end(); )
        {
            if ( (it->size()<unsigned(min_contour_m)) || (it->size()>unsigned(max_contour_m)) )
                it=object_contours.erase(it);
            else
                ++it;
        }

        /// Look for the largest contour and rember that index
        for( unsigned int i = 0; i<object_contours.size(); i++ ) // iterate through each contour.
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
        for( unsigned int i = 0; i < object_contours.size(); i++ )
        {
            mum[i] = moments( object_contours[i], false );
        }
        ///  Get the mass centers:
        vector<cv::Point2f> mcm( object_contours.size() );
        for( unsigned int i = 0; i < object_contours.size(); i++ )
        {
            mcm[i] = cv::Point2f( mum[i].m10/mum[i].m00 , mum[i].m01/mum[i].m00 );
        }
        /// Display the number of detected objects
        std::string contours_number = "Objects detected "+patch::to_string(object_contours.size());
        cv::putText(combined2,contours_number, cv::Point(25, 50), 1, 1, CV_RGB(7, 166, 245), 1, 8, false);
        cv::Scalar color = cv::Scalar(33,57,217);

        /// Lets draw some stuff and calculate the object coordinate also with respect to our rectangle

        cv::Point2f object_coordinate, absolute_object;
        for( unsigned int i = 0; i< object_contours.size(); i++ )
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
        unsigned int width = image.size().width;
        unsigned int height = image.size().height;
        for (unsigned int i = 0; i<height; i += stepSize){
            cv::line(image, cv::Point(0, i), cv::Point(width, i), cv::Scalar(255, 204, 0),2,8,0);
        }
        for (unsigned int i = 0; i<width; i += stepSize){
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

/*
    Camera cam;
    Error error;
    BusManager busMgr;
    PGRGuid guid;
    StrobeControl mStrobe;
    CameraInfo camInfo;
    TriggerModeInfo triggerModeInfo;
    TriggerMode triggerMode;
    Format7Info fmt7Info;
    Format7ImageSettings fmt7ImageSettings;
    Format7PacketInfo fmt7PacketInfo;
    Property frmRate;
    Property autoexp;
    Property shtr;
    Property gn;


    Camera *ptrCamera;
    Error *ptrError;
    BusManager *ptrBusManager;
    PGRGuid *ptrPGRGuid;
    StrobeControl *ptrStrobeControl;
    CameraInfo *ptrCameraInfo;
    TriggerModeInfo *ptrTriggerModeInfo;
    TriggerMode *ptrTriggerMode;
    Format7Info *ptrFormat7Info;
    Format7ImageSettings *ptrFormat7ImageSettings;
    Format7PacketInfo *ptrFormat7PacketInfo;
    Property *ptrFR, *ptrAE, *ptrSHTR, *ptrGN;

    ptrCamera = &cam;
    ptrError = &error;
    ptrBusManager = &busMgr;
    ptrPGRGuid = &guid;
    ptrStrobeControl = &mStrobe;
    ptrCameraInfo = &camInfo;
    ptrTriggerModeInfo = &triggerModeInfo;
    ptrTriggerMode = &triggerMode;
    ptrFormat7Info = &fmt7Info;
    ptrFormat7ImageSettings = &fmt7ImageSettings;
    ptrFormat7PacketInfo = &fmt7PacketInfo;
    ptrFR = &frmRate;
    ptrAE = &autoexp;
    ptrSHT = &shtr;
    ptrGN = &gn;
*/
