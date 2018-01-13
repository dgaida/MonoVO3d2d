// Implementation FINISHED
// Documentation FINISHED
// 11.01.2018, Daniel Gaida

#include "../include/CamParams.h"        // this class

#include "../include/Debug.h"            // for SPEED_OPTIM


// TODO - #ifndef SPEED_OPTIM, then do everything with float instead of double?
// would this help with speed?



/*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

/**
* standard destructor creates eye camera matrix and no distortion
*/
CamParams::CamParams()
{
  cameraMatrix = cv::Mat::eye(cv::Size(3, 3), CV_64F);

  distCoeffs = cv::Mat::zeros(cv::Size(1, 5), CV_64F);
}

/**
* standard destructor does nothing
*/
CamParams::~CamParams()
{
}



/*=====================    PUBLIC METHODS - SETTERS FOR PARAMS    =======================*/

/**
* set values of camera matrix
*
* [fx 0 u0]
* [0 fy v0]
* [0  0  1]
*
* @param fx : focal length in x-direction
* @param fy : focal length in y-direction
* @param u0 : horizontal component of image center [pixels]
* @param v0 : vertical component of image center [pixels]
*
*/
void CamParams::setCameraMatrix(float fx, float fy, float u0, float v0)
{
  cameraMatrix.at<double>(0, 0) = fx;
  cameraMatrix.at<double>(1, 1) = fy;
  cameraMatrix.at<double>(0, 2) = u0;
  cameraMatrix.at<double>(1, 2) = v0;

#ifndef SPEED_OPTIM
  cvCTRACE << "camera Matrix: " << cameraMatrix << std::endl;
#endif
}

/**
* set distortion coefficients of camera. Creates the column vector
* [k1 k2 p1 p2 k3]
*
* @param k1 : radial distortion parameter
* @param k2 : radial distortion parameter
* @param p1 : tangential distortion parameter 
* @param p2 : tangential distortion parameter 
* @param k3 : radial distortion parameter 
*
*/
void CamParams::setDistCoeffs(float k1, float k2, float p1, float p2, float k3)
{
  distCoeffs.at<double>(0, 0) = k1;
  distCoeffs.at<double>(1, 0) = k2;
  distCoeffs.at<double>(2, 0) = p1;
  distCoeffs.at<double>(3, 0) = p2;
  distCoeffs.at<double>(4, 0) = k3;

#ifndef SPEED_OPTIM
  cvCTRACE << "distortion coefficients: " << distCoeffs << std::endl;
#endif
}


