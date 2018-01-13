// Implementation FINISHED
// Documentation FINISHED
// 11.01.2018, Daniel Gaida

#ifndef FRAMEGRABBER_H_
#define FRAMEGRABBER_H_

#include <time.h>         // for calculating fps

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
* Grabs frames from a camera or from files. 
*
* Call the constructor with the appropriate value for useRealCam to either read frames from
* a camera or read frames from file. To grab the next frame call
* grabNextUndistortedFrame(...). To get the fps the frames are obtianed in the for loop, call 
* getFPS(). Your program might look like:
*
* FrameGrabber myframegrabber(...);
* cv::Mat frame;
*
* myframegrabber.startTimer();
*
* while(myframegrabber.grabNextUndistortedFrame(frame, ...))
* {
*   ... do something with frame
*
*   myframegrabber.getFPS();
*
* }
* 
*/
class FrameGrabber
{

  /*=============================         PUBLIC METHODS         =============================*/

public:

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
  FrameGrabber(bool useRealCam, const char* fullpath_imgs, int camera_id= 0);

  /**
  * standard destructor releases VideoCapture device if read frames from camera
  */
  virtual ~FrameGrabber();



  /*=============================         PUBLIC METHODS         =============================*/

public:

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
  bool grabNextUndistortedFrame(cv::Mat& frame,
    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);

  /**
  * starts a timing event, saves current time in start. needed to calc FPS for each frame.
  * Call this method before the for loop (loop grabbing frames). Inside the loop you then can call 
  * getFPS(). 
  */
  void startTimer()
  {
    // start the clock
    time(&start);
  }



  /*=====================    PUBLIC METHODS - SETTERS FOR PARAMS    =======================*/

  /**
  * set image offset. used when reading images from file. 
  * set the number of the first image here, if it is not imgxx0.
  *
  * @param img_offset : number of the first image
  *
  */
  void setImageOffset(int img_offset)
  {
    this->img_offset = img_offset;
  }



  /*=====================    PUBLIC METHODS - GETTERS FOR PARAMS    =======================*/

  /**
  * gets the current time, gets difference to start time and calculates fps.
  * before calling this you need to call startTimer(). 
  *
  * @return FPS : frames per second
  */
  double getFPS() // not const, as it stops timer
  {
    // see how much time has elapsed
    time(&end);

    // calculate current FPS
    sec = difftime(end, start);

    fps = counter / sec;

    return fps;
  }

  /**
  * returns value of counter, being the current frame index. 
  *
  * @return counter
  */
  int getCounter() const
  {
    return counter;
  }



  /*=============================        PUBLIC VARIABLES        =============================*/


  /*===========================        PROTECTED VARIABLES        ===========================*/

protected:



  /*=============================       PRIVATE VARIABLES        =============================*/

private:

  /**
  * path to the image sequence when reading frames from file, including name of file
  */
  char fullpath_imgs[170];
  
  /**
  * full path and filename of the read image, if reading from file
  * only used in grabNextUndistortedFrame. could also be created there, maybe faster when it is 
  * created here once
  */
  char filename1[200];      // if reading images from file

  /**
   * for reading images, ETH starts at image 1 and virtual world at image 0
   */
  int img_offset = 0;

  /**
   * frame counter, before grabbing frame it is increased by 1, so will be 0 for first frame
   */
  int counter = -1;

  /**
   * VideoCapture used when reading frames from real camera
   */
  cv::VideoCapture cap;                           

  /**
  * true when reading frames from real camera, else read frames from files
  */
  const bool useRealCam;

  // start and end times needed to calculate FPS
  time_t start, end;

  // fps calculated using number of frames / seconds
  // floating point seconds elapsed since start
  double fps, sec;

};

#endif


