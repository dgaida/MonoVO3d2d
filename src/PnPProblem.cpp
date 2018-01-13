/*
 * PnPProblem.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */
// Implementation OK
// Documentation OK
// 13.01.2018, Daniel Gaida

#include <iostream>
#include <sstream>

#include "../include/PnPProblem.h"       // this class
#include "../include/Debug.h"            // for cvIN_FCT

#include <opencv2/calib3d/calib3d.hpp>

/* Functions headers */
cv::Point3f CROSS(cv::Point3f v1, cv::Point3f v2);
double DOT(cv::Point3f v1, cv::Point3f v2);
cv::Point3f SUB(cv::Point3f v1, cv::Point3f v2);
cv::Point3f get_nearest_3D_point(std::vector<cv::Point3f> &points_list, cv::Point3f origin);


/* Functions for Möller-Trumbore intersection algorithm */

cv::Point3f CROSS(cv::Point3f v1, cv::Point3f v2)
{
  cv::Point3f tmp_p;
  tmp_p.x =  v1.y*v2.z - v1.z*v2.y;
  tmp_p.y =  v1.z*v2.x - v1.x*v2.z;
  tmp_p.z =  v1.x*v2.y - v1.y*v2.x;
  return tmp_p;
}

double DOT(cv::Point3f v1, cv::Point3f v2)
{
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

cv::Point3f SUB(cv::Point3f v1, cv::Point3f v2)
{
  cv::Point3f tmp_p;
  tmp_p.x =  v1.x - v2.x;
  tmp_p.y =  v1.y - v2.y;
  tmp_p.z =  v1.z - v2.z;
  return tmp_p;
}

/* End functions for Möller-Trumbore intersection algorithm
 *  */

// Function to get the nearest 3D point to the Ray origin
cv::Point3f get_nearest_3D_point(std::vector<cv::Point3f> &points_list, cv::Point3f origin)
{
  cv::Point3f p1 = points_list[0];
  cv::Point3f p2 = points_list[1];

  double d1 = std::sqrt( std::pow(p1.x-origin.x, 2) + std::pow(p1.y-origin.y, 2) + std::pow(p1.z-origin.z, 2) );
  double d2 = std::sqrt( std::pow(p2.x-origin.x, 2) + std::pow(p2.y-origin.y, 2) + std::pow(p2.z-origin.z, 2) );

  if(d1 < d2)
  {
      return p1;
  }
  else
  {
      return p2;
  }
}



/*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

/**
* standard constructor given the intrinsic camera parameters. Creates the matrices.
*
* @param cameraMatrix : intrinsic camera parameters
*/
PnPProblem::PnPProblem(const cv::Mat& cameraMatrix) : _A_matrix(cameraMatrix)
{
  _R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
  _t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
  _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix

}

PnPProblem::~PnPProblem()
{
  // Auto-generated destructor stub
}



/*=============================         PUBLIC METHODS         =============================*/

/**
 * Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use. 
 * Calls cv::solvePnPRansac with 3d/3d correspondences which calculates the camera pose. The camera
 * pose is saved in the _P_matrix of the calling object. 
 *
 * TODO: könnte noch vorhereige pose übergeben und schleife dann abbrechen, wenn alte und neue pose
 * nicht total verschieden sind. 
 *
 * @param list_points3d : list with model 3D coordinates
 * @param list_points2d : list with scene 2D coordinates : projections of 3D points
 * @param inliers : inliers container
 * @param iterationsCount : numbe rof iterations, used in solvePnPRansac
 * @param reprojectionError : threshold for reprojection error used in the RANSAC method
 * @param confidence : confidence used in solvePnPRansac
 * @param distCoeffs : camera distortion coefficients
 * @param inliers_ratio : ratio of inliers vs. total number of points
 * @param rvec : if useExtrinsicGuess would be set to true inside this method, then you could use this
 * as an initial vector for the rotation vector (not yet implemented properly). the returned vector
 * is the rotation vector calculated in cv::solvePnPRansac. 
 * @param tvec : if useExtrinsicGuess would be set to true inside this method, then you could use this
 * as an initial vector for the translation vector (not yet implemented properly). the returned vector
 * is the translation vector calculated in cv::solvePnPRansac. 
 *
 */
void PnPProblem::estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, 
                                     const std::vector<cv::Point2f> &list_points2d, 
                                     cv::Mat &inliers, int iterationsCount, 
                                     float reprojectionError, double confidence, cv::Mat& distCoeffs, 
                                     float &inliers_ratio, cv::Mat &rvec, cv::Mat &tvec)  
{
  cvIN_FCT;

  //cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
  //cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector

  bool useExtrinsicGuess = false;// false;   // if true the function uses the provided rvec and tvec values as
            // initial approximations of the rotation and translation vectors

  // only needed if we useExtrinsicGuess= true
  //Rodrigues(rvec, _R_matrix);
  //tvec = -_R_matrix * tvec;

  // das kostet richtig viel 40% der laufzeit des gesamtprogramms!!!
  cv::solvePnPRansac(list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec,
    useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
    inliers, /*flags*/cv::SOLVEPNP_ITERATIVE);// cv::SOLVEPNP_ITERATIVE/*cv::SOLVEPNP_P3P*/);

  // ratio of inliers vs. total number of points
  inliers_ratio = (float)inliers.rows / (float)list_points2d.size();

  // Die Gleichungen sind 100 % korrrekt, verglichen mit Matlab
  Rodrigues(rvec,_R_matrix);      // converts Rotation Vector to Matrix
  _t_matrix = -_R_matrix.t() * tvec;       // set translation matrix
  
  /*std::cout << tvec << std::endl;
  std::cout << _t_matrix << std::endl;
  std::cout << _R_matrix << std::endl;
  std::cout << -_R_matrix * tvec << std::endl;*/

  this->set_P_matrix(_R_matrix, _t_matrix); // set rotation-translation matrix

  cvOUT_FCT;
}

//// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use
//// TODO: könnte noch vorhereige pose übergeben und schleife dann abbrechen, wenn alte und neue pose
//// nicht total verschieden sind. 
//void PnPProblem::estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
//  const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
//  /*int flags,*/ cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
//  float reprojectionError, double confidence, cv::Mat& distCoeffs,
//  float &inliers_ratio, float &reproErrOut/*,
//                                          const cv::Mat &t*/)    // Ransac parameters
//{
//  cvIN_FCT;
//
//  //cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
//  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
//  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector
//
//                                                    //_A_matrix= cv::Mat::ones(3, 3, CV_64FC1);
//
//  bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
//                                    // initial approximations of the rotation and translation vectors
//
//  reproErrOut = reprojectionError;
//
//  cv::Mat tvec_prev = cv::Mat::zeros(3, 1, CV_64FC1);
//  cv::Mat rvec_prev = cv::Mat::zeros(3, 1, CV_64FC1);
//  cv::Mat inliers_prev;
//  cv::Mat tvec_prev2 = cv::Mat::zeros(3, 1, CV_64FC1);
//  cv::Mat rvec_prev2 = cv::Mat::zeros(3, 1, CV_64FC1);
//
//  bool break_OK = false;
//
//  float norm_r = 100000.0f;
//
//  // in iteration 3 gibt es ein problem wenn ich 0.15f wähle, muss 0.14f sein
//  while (!break_OK ||
//    (float)inliers_prev.rows / (float)list_points2d.size() < /*0.14f*/0.14f
//    /* || inliers.rows < 70*/)
//  {
//    //tvec = -_R_matrix * _t_matrix;
//    //Rodrigues(_R_matrix, rvec);
//
//    // if not first iteration, copy tvec and rvec from first run of solvepnpransac
//    if (norm_r < 100000.0f)
//    {
//      tvec.copyTo(tvec_prev2);
//      rvec.copyTo(rvec_prev2);
//    }
//
//    //cvCTRACE << "iter: " << reproErrOut << std::endl;
//
//    // TODO
//    // das kostet richtig viel 40% der laufzeit des gesamtprogramms!!!
//    cv::solvePnPRansac(list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec,
//      useExtrinsicGuess, iterationsCount, reproErrOut, confidence,
//      inliers, /*flags*/cv::SOLVEPNP_ITERATIVE);// cv::SOLVEPNP_ITERATIVE/*cv::SOLVEPNP_P3P*/);
//
//                                                // then in the second iteration the result for rotation got worse, then
//                                                // break immediately no matter what. the prev results stored in the first iteration
//                                                // will be used after the loop. that's why I defined prev2 here
//                                                //if (cv::norm(rvec - rvec_prev2) > norm_r)
//                                                // break;
//
//                                                // if we get a very good result immediately, then break
//    if ((float)inliers_prev.rows / (float)list_points2d.size() > 0.25f)
//      break;
//
//    reproErrOut += 0.1f;
//
//    tvec.copyTo(tvec_prev);
//    rvec.copyTo(rvec_prev);
//    inliers.copyTo(inliers_prev);
//
//    cv::solvePnPRansac(list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec,
//      useExtrinsicGuess, iterationsCount, reproErrOut, confidence,
//      inliers, /*flags*/cv::SOLVEPNP_ITERATIVE);// cv::SOLVEPNP_ITERATIVE/*cv::SOLVEPNP_P3P*/);
//
//                                                //std::cout << tvec_prev << std::endl;
//                                                //std::cout << tvec << std::endl;
//#ifndef FULLVERSION_D
//    cvCTRACE << cv::norm(tvec - tvec_prev) << std::endl;
//    cvCTRACE << cv::norm(rvec - rvec_prev) << std::endl;
//#endif
//
//    norm_r = cv::norm(rvec - rvec_prev);
//
//    // zu klein darf der nicht sein, macht bei anfangsframes probleme
//    if (norm_r < 0.45f)
//      break_OK = true;
//
//    reproErrOut += 0.1;
//
//    if (reproErrOut >= 1.0)
//      break;
//  }
//
//  inliers_ratio = (float)inliers.rows / (float)list_points2d.size();
//
//  // to redo the last +0.1, which was not performed anymore
//  if (reproErrOut != reprojectionError)
//  {
//    // get copied results of previous run of solvepnpransac
//    rvec = rvec_prev;
//    tvec = tvec_prev;
//    inliers = inliers_prev;
//
//    reproErrOut -= 0.2f;
//  }
//  //reprojectionError = reprojectionError_copy;
//
//#ifndef FULLVERSION_D
//  cvCTRACE << "reproErrOut: " << reproErrOut << std::endl;
//#endif
//
//  // Die Gleichungen sind 100 % korrrekt, verglichen mit Matlab
//  Rodrigues(rvec, _R_matrix);      // converts Rotation Vector to Matrix
//  _t_matrix = -_R_matrix.t() * tvec;       // set translation matrix
//                                           //_t_matrix = -_R_matrix/*.t()*/ * tvec;       // set translation matrix
//
//                                           /*std::cout << tvec << std::endl;
//                                           std::cout << _t_matrix << std::endl;
//                                           std::cout << _R_matrix << std::endl;
//                                           std::cout << -_R_matrix * tvec << std::endl;*/
//
//
//  this->set_P_matrix(_R_matrix, _t_matrix); // set rotation-translation matrix
//
//  cvOUT_FCT;
//}

/**
* backproject the given 3d point point3d back to the image plane using the estimated pose. 
* the point has to be measured
* in world coordinates (I guess). Method not yet tested.
*
* @param point3d : a 3 dimensional point measured in world coordinates
*
* @return point3d projected on image plane
*/
cv::Point2f PnPProblem::backproject3DPoint(const cv::Point3f &point3d) const
{
  // 3D point vector [x y z 1]'
  cv::Mat point3d_vec = cv::Mat(4, 1, CV_64FC1);
  point3d_vec.at<double>(0) = point3d.x;
  point3d_vec.at<double>(1) = point3d.y;
  point3d_vec.at<double>(2) = point3d.z;
  point3d_vec.at<double>(3) = 1;

  // 2D point vector [u v 1]'
  cv::Mat point2d_vec = cv::Mat(4, 1, CV_64FC1);
  point2d_vec = _A_matrix * _P_matrix * point3d_vec;

  // Normalization of [u v]'
  cv::Point2f point2d;
  point2d.x = (float)(point2d_vec.at<double>(0) / point2d_vec.at<double>(2));
  point2d.y = (float)(point2d_vec.at<double>(1) / point2d_vec.at<double>(2));

  return point2d;
}

/**
* triangulates given features (2d points in image) features_k_1 and features_k seen at the given
* camera poses pose_k_1 and pose_k to 3d points. the 3d points are returned in world coordinate
* systems.
*
* @param pose_k_1 : pose of the camera in world coordinates at frame k-1
* @param pose_k : pose of the camera in world coordinates at frame k
* @param cameraMatrix : camera matrix with instrinsic camera parameters (see CamParams)
* @param features_k_1 : 2d points (features) detected in frame k-1
* @param features_k : 2d points (features) detected in frame k
*
* @return : triangulated 3d points measured in world coordinate system
*/
std::vector<cv::Point3f> PnPProblem::triangulatePointsConvEuclidian(
  const cv::Mat& pose_k_1, const cv::Mat& pose_k, const cv::Mat &cameraMatrix, 
  const std::vector<cv::Point2f>& features_k_1, const std::vector<cv::Point2f>& features_k)
{
  cvIN_FCT;
  
  cv::Mat R = PnPProblem::get_rot_part(pose_k);
  cv::Mat t = -R * PnPProblem::get_trans_part(pose_k);
  
  // TODO: laut Matlab müsste hier eigentlich stehen:
  //cv::Mat t = -R.t() * PnPProblem::get_trans_part(pose_k);
  //R = R.t();
  // weiter unten genauso!!! ergibt nur furchtbare ergebnisse!

  cv::Mat pnts3D_temp;

  cv::Mat newpose22 = PnPProblem::get_P_matrix(R, t);

  //std::cout << "new pose: " << newpose22 << std::endl;

  R = PnPProblem::get_rot_part(pose_k_1);
  t = -R * PnPProblem::get_trans_part(pose_k_1);
  
  cv::Mat pose_temp = PnPProblem::get_P_matrix(R, t);

  //std::cout << pose_k_1 << std::endl;
  //std::cout << pose_k << std::endl;

  cv::Mat camMatpose = cameraMatrix * newpose22;
  // cmaMatpose ist korrekt, mit Matlab verglichen
  //std::cout << camMatpose << std::endl;

  // Triangulates the 3d position of 2d correspondences between several images
  cv::triangulatePoints(cameraMatrix * pose_temp, camMatpose,
    features_k_1, features_k, pnts3D_temp);

  std::vector<cv::Point3f> pnts3D_vec;
  pnts3D_vec.reserve(pnts3D_temp.size().width);

  for (int ipoint = 0; ipoint < pnts3D_temp.size().width; ipoint++)
  {
    cv::Point3f mypoint;

    /*std::cout 
      << pnts3D_temp.at<float>(0, ipoint) << ", " 
      << pnts3D_temp.at<float>(1, ipoint) << ", "
      << pnts3D_temp.at<float>(2, ipoint) << ", "
      << pnts3D_temp.at<float>(3, ipoint) << std::endl;*/

    // same as convertPointsFromHomogeneous()
    mypoint.x = pnts3D_temp.at<float>(0, ipoint) / pnts3D_temp.at<float>(3, ipoint);
    // muss hier float sein
    //mypoint.x = pnts3D_temp.at<double>(0, ipoint) / pnts3D_temp.at<double>(3, ipoint);
    mypoint.y = pnts3D_temp.at<float>(1, ipoint) / pnts3D_temp.at<float>(3, ipoint);
    mypoint.z = pnts3D_temp.at<float>(2, ipoint) / pnts3D_temp.at<float>(3, ipoint);

    pnts3D_vec.push_back(mypoint);
    
  }

  cvOUT_FCT;

  return pnts3D_vec;
}

/**
* Calculates pose * reltrans and returns the resulting transformation matrix
*
* @param pose : transformation matrix (3x4)
* @param reltrans : relative transformation matrix (3x4)
*
* @return pose * reltrans
*/
cv::Mat PnPProblem::multiplyTransformations(const cv::Mat &pose, const cv::Mat &reltrans)
{
  cvIN_FCT;

  cv::Mat R = pose.colRange(0,3);
  cv::Mat t = pose.col(3);

  cv::Mat relR = reltrans.colRange(0, 3);
  cv::Mat relt = reltrans.col(3);

  cv::Mat Rf = R * relR;
  cv::Mat tf = R * relt + t;

  /*std::cout << t << std::endl;
  std::cout << relt << std::endl;
  std::cout << tf << std::endl;*/

  cvOUT_FCT;

  return get_P_matrix(Rf, tf);
}