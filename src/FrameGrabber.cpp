// Implementation FINISHED
// Documentation FINISHED
// 11.01.2018, Daniel Gaida

#include "../include/FrameGrabber.h"         // this class
  
#include "../include/Debug.h"                // for cvCTRACE



/*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

/**
* if useRealCam is true, then opens the VideoCapture cap. If cap cannot be opened
* then the application exits.
*
* @param useRealCam : if true then grab frames from a real camera,
* else frames are grabbed from files
* @param fullpath_imgs : path to images, if grabbing images from file. It needs to have a
* place holder such as "%i" or "%d" for the frame counter, as an example:
* "D://NewTsukuba//image%04d.jpg"
* @param camera_id : camera id that is opened if you set useRealCam to true, if you have more than
* one camera connected to your computer this parameter is important. Default: 0
*
*/
FrameGrabber::FrameGrabber(bool useRealCam, const char* fullpath_imgs, int camera_id) :
  useRealCam(useRealCam)
{
  if (useRealCam)
  {
    cap.open(camera_id);                      // open webcam at id 0

    if (!cap.isOpened())   // check if we succeeded
    {
      cvCTRACE << "Could not open the camera device" << std::endl;
      exit(-1);
    }
  }
  else // copy given path 
    strcpy(this->fullpath_imgs, fullpath_imgs);
}

/**
* standard destructor releases VideoCapture device if read frames from camera
*/
FrameGrabber::~FrameGrabber()
{
  if (useRealCam)
    // release cap, seems to be also called by destructor of cap, 
    // so actually no need to call it here
    cap.release();
}



/*=============================         PUBLIC METHODS         =============================*/

/**
* Grabs new frame and undistorts it. Either grabs it from file or from real camera.
* Depends on useRealCam. If image cannot be read from file, then application exits.
*
* @param frame : this is the new frame
* @param cameraMatrix : camera matrix of camera, see CamParams class
* @param distCoeffs : distortion coefficients of camera, see CamParams class
*
* @return true, if frame could be grabbed, else false
*/
bool FrameGrabber::grabNextUndistortedFrame(cv::Mat& frame,
  const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
  counter++;

  bool success = false;

  if (useRealCam)
    success= cap.read(frame);
  else
  {
    sprintf(filename1, fullpath_imgs, counter + img_offset);

    try
    {
      frame = cv::imread(filename1);
      success = true;
    }
    catch (...)
    {
      cvCERROR << "Could not read image from file: " << filename1 << std::endl;
      exit(-1);
    }
  }

  //cv::Mat frameGray;
  // ändert nichts am Ergebnis ob mit Grauwert oder Farbwert Bildern gearbeitet wird
  //cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

  // undistort image
  if (success)
  {
    cv::Mat temp = frame/*Gray*/.clone();
    cv::undistort(temp, frame, cameraMatrix, distCoeffs);
  }

  return success;
}


