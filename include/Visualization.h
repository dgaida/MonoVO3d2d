// Implementation OK
// Documentation OK
// 11.01.2018, Daniel Gaida

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

/**
* Visualizes the current frame as well as the trajectory
*/
class Visualization
{
  /*=============================         PUBLIC METHODS         =============================*/

public:

  /*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

  /**
  * standard constructor creates the Mat that visualizes the trajectory and opens
  * two windows
  */
  Visualization();
  
  /**
  * standard destructor destroys windows opened in constructor
  */
  virtual ~Visualization();



  /*=====================    PUBLIC METHODS    =======================*/

public:

  /**
  * Displays the current frame and writes some text on the frame with fps and
  * ratio of inliers/outliers. 
  *
  * @param frame_vis : current frame that is drawn on in this function and which is 
  * displayed in this function
  * @param fps : frames per second gotten from framegrabber
  * @param inliers_idx : inliers of PnP method
  * @param good_matches_size : size of vector of good_matches between frame k-1 and k
  *
  */
  void displayFrame(cv::Mat &frame_vis, double fps, const cv::Mat &inliers_idx, 
    int good_matches_size);

  /**
  * Displays the camera trajectory seen from the top (x-z-plane) in a window
  * named Trajectory. Also shows text on the window: position and orientation
  *
  * @param pose : current pose of camera
  * @param pnts3D_vec : 3D points in current frame
  *
  */
  void displayTrajectory(const cv::Mat &pose, const std::vector<cv::Point3f> &pnts3D_vec,
    const cv::Point3f &GT_loc);



  /*=============================        PUBLIC VARIABLES        =============================*/


  /*===========================        PROTECTED VARIABLES        ===========================*/

protected:



  /*=============================       PRIVATE VARIABLES        =============================*/

private:

  // matrix on which the trajectory is drawn
  cv::Mat traj;

  // text containing the coordinates of the trajectory
  char text_traj[100];

  // font style of the text written on the trajectory window
  const int fontFace = cv::FONT_HERSHEY_PLAIN;
  const double fontScale = 1;
  const int thickness = 1;
  const cv::Point textOrg= cv::Point(10, 50);

  // Some basic colors
  const cv::Scalar green = cv::Scalar(0, 255, 0);
  const cv::Scalar yellow = cv::Scalar(0, 255, 255);
  const cv::Scalar red= cv::Scalar(0, 0, 255);
  const cv::Scalar blue= cv::Scalar(255, 0, 0);

  // at this pixel value the x axis has its origin
  int x_origin = 300;
  // at this pixel value the z axis has its origin
  int z_origin = 100;

};

#endif


