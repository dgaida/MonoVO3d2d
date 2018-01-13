/*
 * PnPProblem.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */
// Implementation OK
// Documentation OK
// 13.01.2018, Daniel Gaida

#ifndef PNPPROBLEM_H_
#define PNPPROBLEM_H_

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * This class contains two important methods:
 *
 * - estimatePoseRANSAC : Calculates the camera pose given 3d/2d point correspondences by calling 
 * cv::solvePnPRansac
 * - triangulatePointsConvEuclidian : calculates 3d points by triangulation given features and camera poses
 * from two frames calling cv::triangulatePoints. 
 *
 * This class also saves the in estimatePoseRANSAC() calculated transformation matrix. You can get it
 * and also change its rotation and translation parts using the class its public methods. 
 *
 * For more information see:
 *
 * https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
class PnPProblem
{
  /*=============================         PUBLIC METHODS         =============================*/

public:

  /*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

  /**
  * standard constructor given the intrinsic camera parameters. Creates the matrices. 
  *
  * @param cameraMatrix : intrinsic camera parameters, see CamParams class
  */
  explicit PnPProblem(const cv::Mat& cameraMatrix);

  /**
  * standard destructor does nothing
  */
  virtual ~PnPProblem();



  /*=============================         PUBLIC METHODS         =============================*/

public:

  /**
   * backproject the given 3d point point3d back to the image plane using the estimated pose. 
   * the point has to be measured
   * in world coordinates (I guess). Method not yet tested. 
   *
   * @param point3d : a 3 dimensional point measured in world coordinates
   *
   * @return point3d projected on image plane
   */
  cv::Point2f backproject3DPoint(const cv::Point3f &point3d) const;
  
  
  //void estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, 
  //                         const std::vector<cv::Point2f> &list_points2d,
  //                         /*int flags,*/ cv::Mat &inliers,
  //                         int iterationsCount, float reprojectionError, double confidence, 
  //                         cv::Mat& distCoeffs, float &inliers_ratio, float &reproErrOut);

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
  void estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d,
    const std::vector<cv::Point2f> &list_points2d, cv::Mat &inliers,
    int iterationsCount, float reprojectionError, double confidence,
    cv::Mat& distCoeffs, float &inliers_ratio, cv::Mat &rvec, cv::Mat &tvec);

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
  static std::vector<cv::Point3f> triangulatePointsConvEuclidian(const cv::Mat& pose_k_1, 
    const cv::Mat& pose_k, const cv::Mat &cameraMatrix,
    const std::vector<cv::Point2f>& features_k_1, const std::vector<cv::Point2f>& features_k);

  /**
  * Calculates pose * reltrans and returns the resulting transformation matrix
  *
  * @param pose : transformation matrix (3x4)
  * @param reltrans : relative transformation matrix (3x4)
  *
  * @return pose * reltrans
  */
  static cv::Mat multiplyTransformations(const cv::Mat &pose, const cv::Mat &reltrans);



  /*=====================    PUBLIC METHODS - SETTERS FOR PARAMS    =======================*/

public:

  /**
  * Combines the given rotation matrix (3x3) and translation matrix (3x1) (vector)
  * to a transformation matrix and saves it in _P_matrix.
  *
  * @param R_matrix : rotation matrix (3x3)
  * @param t_matrix : translation matrix (3x1)
  *
  */
  void set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix);

  /**
  * Sets the given translation matrix (3x1) (vector)
  * in the given transformation matrix P_matrix (aka replaces the translation in P_matrix).
  *
  * @param P_matrix : transformation matrix (3x4) whose translation part after the call is
  * the one in t_matrix
  * @param t_matrix : translation matrix (3x1)
  *
  */
  static void set_t_vector(cv::Mat &P_matrix, const cv::Mat &t_matrix);

  /**
  * Sets the given rotation matrix (3x3)
  * in the given transformation matrix P_matrix (aka replaces the rotation in P_matrix).
  *
  * @param P_matrix : transformation matrix (3x4) whose rotation part after the call is
  * the one in R_matrix
  * @param R_matrix : rotation matrix (3x3)
  *
  */
  static void set_R_matrix(cv::Mat &P_matrix, const cv::Mat &R_matrix);



  /*=====================    PUBLIC METHODS - GETTERS FOR PARAMS    =======================*/

public:

  /**
  * returns the camera matrix with the intrinsic parameters
  *
  * [fx 0 u0]
  * [0 fy v0]
  * [0  0  1]
  *
  * @return : _A_matrix, camera matrix
  */
  cv::Mat get_A_matrix() const 
  { 
    return _A_matrix; 
  }
  
  /**
  * returns the rotation matrix 
  *
  * @return : _R_matrix, rotation matrix
  */
  cv::Mat get_R_matrix() const 
  { 
    return _R_matrix; 
  }
  
  /**
  * returns the translation matrix (vector)
  *
  * @return : _t_matrix, translation matrix
  */
  cv::Mat get_t_matrix() const 
  { 
    return _t_matrix; 
  }
  
  /**
  * returns the transformation matrix
  *
  * @return : _P_matrix, transformation matrix
  */
  cv::Mat get_P_matrix() const 
  { 
    return _P_matrix; 
  }

  /**
  * combines the given rotation matrix (3x3) and translation matrix (3x1) (vector) 
  * to a transformation matrix and returns it. 
  *
  * @param R_matrix : rotation matrix (3x3)
  * @param t_matrix : translation matrix (3x1)
  *
  * @return : transformation matrix (3x4)
  */
  static cv::Mat get_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix);

  /**
  * returns the rotation part of the given transformation matrix
  *
  * @param T_matrix : transformation matrix (3x4)
  *
  * @return : rotation part (3x3) of transformation matrix
  */
  static cv::Mat get_rot_part(const cv::Mat &T_matrix);
  
  /**
  * returns the translation part of the given transformation matrix
  *
  * @param T_matrix : transformation matrix (3x4)
  *
  * @return : translation part (3x1) of transformation matrix
  */
  static cv::Mat get_trans_part(const cv::Mat &T_matrix);

  

  /*=============================        PUBLIC VARIABLES        =============================*/


  /*===========================        PROTECTED VARIABLES        ===========================*/

protected:



  /*=============================       PRIVATE VARIABLES        =============================*/

private:

  /** The calibration matrix
   *
   * [fx 0 u0]
   * [0 fy v0]
   * [0  0  1]
   */
  const cv::Mat _A_matrix;
  /** The computed rotation matrix */
  cv::Mat _R_matrix;
  /** The computed translation matrix */
  cv::Mat _t_matrix;
  /** The computed projection matrix */
  cv::Mat _P_matrix;

};

// Functions for Möller-Trumbore intersection algorithm
cv::Point3f CROSS(cv::Point3f v1, cv::Point3f v2);
double DOT(cv::Point3f v1, cv::Point3f v2);
cv::Point3f SUB(cv::Point3f v1, cv::Point3f v2);

#endif /* PNPPROBLEM_H_ */
