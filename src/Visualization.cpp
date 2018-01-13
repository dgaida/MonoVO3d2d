// Implementation OK
// Documentation OK
// 11.01.2018, Daniel Gaida

#include "../include/Visualization.h"          // this class
#include "../include/utils.h"                  // drawText

#include <opencv2/highgui.hpp>      // for namedWindow
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>      // for circle

#include "../include/Debug.h"



/**
 * standard constructor creates the Mat that visualizes the trajectory and opens
 * two windows
 */
Visualization::Visualization()
{
  traj = cv::Mat::zeros(600, 600, CV_8UC3);

  // Create & Open Window
  cv::namedWindow("Real-time frame", cv::WINDOW_KEEPRATIO);
  cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);// Create a window for display.

}

/**
 * standard destructor destroys windows opened in constructor
 */
Visualization::~Visualization()
{
  // Close and Destroy Window
  cv::destroyWindow("Real-time frame");
  cv::destroyWindow("Trajectory");
}

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
void Visualization::displayFrame(cv::Mat &frame_vis, double fps, const cv::Mat &inliers_idx,
  int good_matches_size)
{

  drawFPS(frame_vis, fps, yellow); // frame ratio
  double detection_ratio = ((double)inliers_idx.rows / (double)good_matches_size) * 100;
  drawConfidence(frame_vis, detection_ratio, blue);

  // -- Step X: Draw some debugging text

  // Draw some debug text
  int inliers_int = inliers_idx.rows;
  int outliers_int = good_matches_size - inliers_int;
  std::string inliers_str = IntToString(inliers_int);
  std::string outliers_str = IntToString(outliers_int);
  std::string n = IntToString(good_matches_size);
  std::string text = "Found " + inliers_str + " of " + n + " matches";
  std::string text2 = "Inliers: " + inliers_str + " - Outliers: " + outliers_str;

  drawText(frame_vis, text, green);
  drawText2(frame_vis, text2, red);

  cv::imshow("Real-time frame", frame_vis);
  cv::waitKey(1);

}

/**
* Displays the camera trajectory seen from the top (x-z-plane) in a window
* named Trajectory. Also shows text on the window: position and orientation
*
* @param pose : current pose of camera
* @param pnts3D_vec : 3D points in current frame
*
*/
void Visualization::displayTrajectory(const cv::Mat &pose, const std::vector<cv::Point3f> &pnts3D_vec, 
  const cv::Point3f &GT_loc)
{
  cv::Mat t= PnPProblem::get_trans_part(pose);

  cv::Mat euler = rot2euler(PnPProblem::get_rot_part(pose));

  cv::Mat points3D_xz = cv::Mat::zeros(600, 600, CV_8UC3);

  // TODO: could try to plot every 2nd point, tried it, does not make things faster
  for (int ipnt = 0; ipnt < pnts3D_vec.size(); ipnt++)
  {
    int xp = int(pnts3D_vec.at(ipnt).x * 1.0f) + x_origin;
    int zp = int(pnts3D_vec.at(ipnt).z * 1.0f) + z_origin;

    // prüfen warum manchmal alle punkte auf einen punkt zusammen schrumpfen
    // passiert, wenn pose nicht stimmt
    //std::cout << xp << ", " << zp << std::endl;

    cv::circle(points3D_xz, cv::Point(xp, zp), 1, CV_RGB(0, 255, 0), 2);
  }

  int x = int(t.at<double>(0) * 1.0f) + x_origin;
  int z = int(t.at<double>(2) * 1.0f/*/ 10.0f*/) + z_origin;
  
  //std::cout << z << std::endl;

  // we look at the x-z plane from the top
  cv::circle(traj, cv::Point(x, z), 1, CV_RGB(255, 0, 0), 2);

  // plot ground truth

  cv::circle(traj, cv::Point(GT_loc.x + x_origin, GT_loc.z + z_origin), 
    1, CV_RGB(0, 0, 255), 2);


  //drawArrow(traj, cv::Point(x, y), cv::Point(x, y));

  // draw background for text, have to redraw all teh time, because traj is never deleted
  // so that the history of the trajectory is drawn
  cv::rectangle(traj, cv::Point(10, 30), cv::Point(580, 80), CV_RGB(0, 0, 0), CV_FILLED);
  sprintf(text_traj, "Coordinates: x = %02f y = %02f z = %02f",
    t.at<double>(0), t.at<double>(1), t.at<double>(2));
  cv::putText(traj, text_traj, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

  sprintf(text_traj, "Orientation: %02f, %02f, %02f",
    euler.at<double>(0), euler.at<double>(1), euler.at<double>(2));
  cv::putText(traj, text_traj, textOrg + cv::Point(0, 25), 
    fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

  cv::putText(traj, "Estimated trajectory", textOrg + cv::Point(0, 50),
    fontFace, fontScale, cv::Scalar(0,0,255), thickness, 8);

  // add both images and display
  cv::imshow("Trajectory", traj + points3D_xz);
  cv::waitKey(1);

}

