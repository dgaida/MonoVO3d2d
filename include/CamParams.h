// Implementation FINISHED
// Documentation FINISHED
// 11.01.2018, Daniel Gaida

#ifndef CAMPARAMS_H_
#define CAMPARAMS_H_

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

/**
* stores the camera parameters of a pinhole camera model. These are:
*
* - camera matrix (3x3 double matrix)
*
* [fx 0 u0]
* [0 fy v0]
* [0  0  1]
*
* - distortion coefficients, a 5-dimensional column vector [k1 k2 p1 p2 k3]
*
* The class provides methods to get and set the parameters. 
*
* For a real camera you have to find these parameters during camera calibration. 
* For more information about camera calibration see:
* https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
*
*/
class CamParams
{
  /*=============================         PUBLIC METHODS         =============================*/

public:
  
  /*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

  /**
  * standard constructor creates eye camera matrix and no distortion
  */
  CamParams();

  /**
  * standard destructor does nothing
  */
  virtual ~CamParams();



  /*=====================    PUBLIC METHODS - SETTERS FOR PARAMS    =======================*/

public:

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
  void setCameraMatrix(float fx, float fy, float u0, float v0);

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
  void setDistCoeffs(float k1, float k2, float p1, float p2, float k3);



  /*=====================    PUBLIC METHODS - GETTERS FOR PARAMS    =======================*/

  /**
  * gets the camera matrix, a 3x3 double matrix
  *
  * [fx 0 u0]
  * [0 fy v0]
  * [0  0  1]
  *
  * @return camera matrix
  */
  void getCameraMatrix(cv::Mat &cameraMatrix) const
  {
    cameraMatrix = this->cameraMatrix;
  }

  /**
  * gets the vector of distortion coefficients of camera. A 5-dim double vector. 
  *
  * @return distortion coefficients
  */
  void getDistCoeffs(cv::Mat &distCoeffs) const
  {
    distCoeffs = this->distCoeffs;
  }



  /*=============================        PUBLIC VARIABLES        =============================*/


  /*===========================        PROTECTED VARIABLES        ===========================*/

protected:



  /*=============================       PRIVATE VARIABLES        =============================*/

private:

  // camera matrix. A 3x3 double matrix
  cv::Mat cameraMatrix;
  
  // distortion coefficients. A 5 dim. double vector
  cv::Mat distCoeffs;



};

#endif

