// Implementation FINISHED
// Documentation OK could be imporved, by specifying that matrices and vectors have to be double
// 11.01.2018, Daniel Gaida

#include "../include/PnPProblem.h"         // this class
#include "../include/Debug.h"



/*=====================    PUBLIC METHODS - SETTERS FOR PARAMS    =======================*/

/**
* Combines the given rotation matrix (3x3) and translation matrix (3x1) (vector)
* to a transformation matrix and saves it in _P_matrix.
*
* @param R_matrix : rotation matrix (3x3)
* @param t_matrix : translation matrix (3x1)
*
*/
void PnPProblem::set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
  _P_matrix = get_P_matrix(R_matrix, t_matrix);
}

/**
* Sets the given translation matrix (3x1) (vector)
* in the given transformation matrix P_matrix (aka replaces the translation in P_matrix).
*
* @param P_matrix : transformation matrix (3x4) whose translation part after the call is
* the one in t_matrix
* @param t_matrix : translation matrix (3x1)
*
*/
void PnPProblem::set_t_vector(cv::Mat &P_matrix, const cv::Mat &t_matrix)
{
  P_matrix.at<double>(0, 3) = t_matrix.at<double>(0);
  P_matrix.at<double>(1, 3) = t_matrix.at<double>(1);
  P_matrix.at<double>(2, 3) = t_matrix.at<double>(2);
}

/**
* Sets the given rotation matrix (3x3)
* in the given transformation matrix P_matrix (aka replaces the rotation in P_matrix).
*
* @param P_matrix : transformation matrix (3x4) whose rotation part after the call is
* the one in R_matrix
* @param R_matrix : rotation matrix (3x3)
*
*/
void PnPProblem::set_R_matrix(cv::Mat &P_matrix, const cv::Mat &R_matrix)
{

  P_matrix.at<double>(0, 0) = R_matrix.at<double>(0, 0);
  P_matrix.at<double>(0, 1) = R_matrix.at<double>(0, 1);
  P_matrix.at<double>(0, 2) = R_matrix.at<double>(0, 2);
  P_matrix.at<double>(1, 0) = R_matrix.at<double>(1, 0);
  P_matrix.at<double>(1, 1) = R_matrix.at<double>(1, 1);
  P_matrix.at<double>(1, 2) = R_matrix.at<double>(1, 2);
  P_matrix.at<double>(2, 0) = R_matrix.at<double>(2, 0);
  P_matrix.at<double>(2, 1) = R_matrix.at<double>(2, 1);
  P_matrix.at<double>(2, 2) = R_matrix.at<double>(2, 2);

}



/*=====================    PUBLIC METHODS - GETTERS FOR PARAMS    =======================*/

/**
* combines the given rotation matrix (3x3) and translation matrix (3x1) (vector)
* to a transformation matrix and returns it.
*
* @param R_matrix : rotation matrix (3x3)
* @param t_matrix : translation matrix (3x1)
*
* @return : transformation matrix (3x4)
*/
cv::Mat PnPProblem::get_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
  cv::Mat P_matrix= cv::Mat::zeros(3, 4, CV_64FC1);

  // Rotation-Translation Matrix Definition
  P_matrix.at<double>(0, 0) = R_matrix.at<double>(0, 0);
  P_matrix.at<double>(0, 1) = R_matrix.at<double>(0, 1);
  P_matrix.at<double>(0, 2) = R_matrix.at<double>(0, 2);
  P_matrix.at<double>(1, 0) = R_matrix.at<double>(1, 0);
  P_matrix.at<double>(1, 1) = R_matrix.at<double>(1, 1);
  P_matrix.at<double>(1, 2) = R_matrix.at<double>(1, 2);
  P_matrix.at<double>(2, 0) = R_matrix.at<double>(2, 0);
  P_matrix.at<double>(2, 1) = R_matrix.at<double>(2, 1);
  P_matrix.at<double>(2, 2) = R_matrix.at<double>(2, 2);
  P_matrix.at<double>(0, 3) = t_matrix.at<double>(0);
  P_matrix.at<double>(1, 3) = t_matrix.at<double>(1);
  P_matrix.at<double>(2, 3) = t_matrix.at<double>(2);

  return P_matrix;
}

/**
* returns the rotation part of the given transformation matrix
*
* @param T_matrix : transformation matrix (3x4)
*
* @return : rotation part (3x3) of transformation matrix
*/
cv::Mat PnPProblem::get_rot_part(const cv::Mat &P_matrix)
{
  cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);

  R_matrix.at<double>(0, 0)= P_matrix.at<double>(0, 0);
  R_matrix.at<double>(0, 1)= P_matrix.at<double>(0, 1);
  R_matrix.at<double>(0, 2)= P_matrix.at<double>(0, 2);
  R_matrix.at<double>(1, 0)= P_matrix.at<double>(1, 0);
  R_matrix.at<double>(1, 1)= P_matrix.at<double>(1, 1);
  R_matrix.at<double>(1, 2)= P_matrix.at<double>(1, 2);
  R_matrix.at<double>(2, 0)= P_matrix.at<double>(2, 0);
  R_matrix.at<double>(2, 1)= P_matrix.at<double>(2, 1);
  R_matrix.at<double>(2, 2)= P_matrix.at<double>(2, 2);

  return R_matrix;
}

/**
* returns the translation part of the given transformation matrix
*
* @param T_matrix : transformation matrix (3x4)
*
* @return : translation part (3x1) of transformation matrix
*/
cv::Mat PnPProblem::get_trans_part(const cv::Mat &P_matrix)
{
  cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);

  t_matrix.at<double>(0) = P_matrix.at<double>(0, 3);
  t_matrix.at<double>(1) = P_matrix.at<double>(1, 3);
  t_matrix.at<double>(2) = P_matrix.at<double>(2, 3);
  
  return t_matrix;
}


